// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#include <aloam_velodyne/utility.hpp>

class ImageProjection{
private:

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;

    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;

    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr segmentedCloudIsolated;
    pcl::PointCloud<PointType>::Ptr segmentedPole;
    pcl::PointCloud<PointType>::Ptr poleLocation;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    PointType nanPoint; // fill in fullCloud at each iteration

    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;

    Eigen::Matrix3f segCloudCovMat;
    Eigen::VectorXi poleIdxPair;

    float startOrientation;
    float endOrientation;

    Params params_;

    cloud_msgs::cloud_info segMsg; // info of segmented cloud
    std_msgs::Header cloudHeader;
    odom_stream_msg::odom_stream poleOdomStream;
    float poleOdom[7];

    std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;

    boost::circular_buffer<float> segXMeanBuff;
    boost::circular_buffer<float> segYMeanBuff;

public:
    ImageProjection():
            nh("~"){



        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &ImageProjection::cloudHandler, this);

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        segmentedCloudIsolated.reset(new pcl::PointCloud<PointType>());
        segmentedPole.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());
        poleCentreCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

        segXMeanBuff.resize(3);
        segYMeanBuff.resize(3);

        segMsg.startRingIndex.assign(N_SCAN, 0);
        segMsg.endRingIndex.assign(N_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    }

    void resetParameters(){
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        segmentedCloudIsolated->clear();
        segmentedPole->clear();
        outlierCloud->clear();
        poleCentreCloud->clear();
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        segYMeanBuff.clear();
        segXMeanBuff.clear();

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    ~ImageProjection(){}

    void copyPointCloudANDIniParams(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        // parameters initialize
        YAML::Node yaml = YAML::LoadFile("/home/xingyu/gitRes/SegTest/parameters.yaml");

        params_.segDirThd = yaml["params_dirThd"].as<double>();
        params_.segRtThd = yaml["params_rtThd"].as<double>();
        params_.segSpacingThd = yaml["params_SpacingThd"].as<double>();
        params_.segHeightThd = yaml["params_HeightThd"].as<double>();
        params_.wThd = yaml["params_wThd"].as<float>();
        params_.startFrame = yaml["params_startFrame"].as<size_t>();
        params_.endFrame = yaml["params_endFrame"].as<size_t>();
        float Theta = yaml["params_segTheta"].as<float>();
        params_.segmentTheta = (Theta / 180 )* M_PI;


        cloudHeader = laserCloudMsg->header;
        cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        // have "ring" channel in the cloud
        if (useCloudRing == true){
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            if (laserCloudInRing->is_dense == false) {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }
        }
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        // 1. Convert ros message to pcl point cloud & initialize params
        copyPointCloudANDIniParams(laserCloudMsg);
        // 2. Start and end angle of a scan
        findStartEndAngle();
        // 3. Range image projection
        projectPointCloud();
        // 4. Mark ground points
        groundRemoval();
        // 5. Point cloud segmentation
        cloudSegmentation();
        // 6. Publish all clouds
        publishCloud();
        // 7. Reset parameters for next iteration
        resetParameters();
    }

    void findStartEndAngle(){
        // start and end orientation of this cloud
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                         laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
        if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
            segMsg.endOrientation -= 2 * M_PI;
        } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
            segMsg.endOrientation += 2 * M_PI;
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
    }

    void projectPointCloud(){
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize;
        PointType thisPoint;

        cloudSize = laserCloudIn->points.size();

        for (size_t i = 0; i < cloudSize; ++i){

            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            // find the row and column index in the iamge for this point
            if (useCloudRing == true){
                rowIdn = laserCloudInRing->points[i].ring;
            }
            else{
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            }
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < sensorMinimumRange)
                continue;

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
        }
    }


    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            for (size_t i = 0; i < groundScanInd; ++i){

                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    // no info to check, invalid points
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }

                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                if (abs(angle - sensorMountAngle) <= 10){
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }
        // extract ground cloud (groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }
        if (pubGroundCloud.getNumSubscribers() != 0){
            for (size_t i = 0; i <= groundScanInd; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (groundMat.at<int8_t>(i,j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                }
            }
        }
    }

    void cloudSegmentation(){
        // segmentation process
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                if (labelMat.at<int>(i,j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        // extract segmented cloud for lidar odometry
        for (size_t i = 0; i < N_SCAN; ++i) {

            segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;

            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
                    // outliers that will not be used for optimization (always continue)
                    if (labelMat.at<int>(i,j) == 999999){
                        if (i > groundScanInd && j % 5 == 0){
                            outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                            continue;
                        }else{
                            continue;
                        }
                    }
                    // majority of ground points are skipped
                    if (groundMat.at<int8_t>(i,j) == 1){
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                            continue;
                    }
                    // mark ground points so they will not be considered as edge features later
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    // mark the points' column index for marking occlusion later
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    // save range info
                    segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                    // save seg cloud
                    segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

            segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;
        }

        // extract segmented cloud for visualization
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < N_SCAN; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                }
            }
        }
    }

    void labelComponents(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY;
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;

        while(queueSize > 0){
            // Pop point
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            // Mark popped point
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;
            // Loop through all the neighboring grids of popped grid
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;
                // prevent infinite loop (caused by put already examined point back)
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                if (angle > params_.segmentTheta){

                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }

        // check if this segment is valid
        bool feasibleSegment = false;
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum){
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;
        }
        // segment is valid, mark these points
        if (feasibleSegment == true){
            // obtained the isolated segmented Cloud
            for(size_t i = 0; i < allPushedIndSize; ++i){
                segmentedCloudIsolated->push_back(fullCloud->points[allPushedIndY[i] + allPushedIndX[i] * Horizon_SCAN]);
                segmentedCloudIsolated->points.back().intensity = labelMat.at<int>(allPushedIndX[i],allPushedIndY[i]);
//                segmentedPole->push_back();
            }
            segCloudPCA(allPushedIndSize);
            segmentedCloudIsolated->clear();
            ++labelCount;
        }else{ // segment is invalid, mark these points
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    void segCloudPCA(int cloudSize){
        int label = segmentedCloudIsolated->points[0].intensity;
        float px,py,pz;
        std::vector<std::pair<float, int>>  zpairVec;
        float *zArray = new float[cloudSize];
        float *aftSortZ;
        std::pair<float, int> pairTmp;
        Eigen::RowVectorXf Pxyz(3);

        for (int i = 0; i < cloudSize; ++i) {
            pz = segmentedCloudIsolated->points[i].z;
            pairTmp.first = pz;
            pairTmp.second = i;
            zpairVec.push_back(pairTmp);
            zArray[i] = pz;
        }
        std::sort(zArray,zArray + cloudSize);
        std::vector<size_t> zidxVec;
        for (size_t i = 0; i < cloudSize; ++i) {
            if (zpairVec.at(i).first - zArray[0] < params_.segHeightThd){
                zidxVec.push_back(zpairVec.at(i).second);
            }
        }

        Eigen::MatrixXf segCloudMat(zidxVec.size(),3);
        pcl::PointCloud<PointType>::Ptr topCutCloud(new pcl::PointCloud<PointType>);
        topCutCloud->clear();

        for (int i = 0; i < zidxVec.size(); ++i) {
            topCutCloud->push_back(segmentedCloudIsolated->points[zidxVec.at(i)]);
            px = segmentedCloudIsolated->points[zidxVec.at(i)].x;
            py = segmentedCloudIsolated->points[zidxVec.at(i)].y;
            pz = segmentedCloudIsolated->points[zidxVec.at(i)].z;
            Pxyz << px, py, pz;
            segCloudMat.row(i) = Pxyz;
        }

        // decentralize the SegMat
        Eigen::MatrixXf deCentredSegMat = segCloudMat.rowwise() - segCloudMat.colwise().mean();
        // Covariance Mat
        Eigen::MatrixXf CovSegMat = (deCentredSegMat.adjoint() * deCentredSegMat) / (double)(segCloudMat.rows());
        //the EigenValue and EigenVector are sorted ascending
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> EigenSolv(CovSegMat);
        //Eigen value in INCREASING order
        Eigen::RowVectorXf eigenVal = EigenSolv.eigenvalues().cast<float>();
        //Eigen vector corresponds to Eigen value
        Eigen::Matrix3f eigenVec = EigenSolv.eigenvectors().cast<float>();

        float SegDir, SegRatio;
        float  width = sqrt(eigenVal[0] * eigenVal[0] + eigenVal[1] * eigenVal[1]);
        SegRatio = eigenVal[2] / width;
        SegDir = atan(eigenVec.col(2)[2] / sqrt(eigenVec.col(2)[0] * eigenVec.col(2)[0] +
                                                eigenVec.col(2)[1] * eigenVec.col(2)[1])) * 180 / PI;

        if (abs(abs(SegDir) - 90) < params_.segDirThd && SegRatio > params_.segRtThd ){
            // the segmentation is pole, check the distance between the last 1,2 pole
            segXMeanBuff.push_back(segCloudMat.colwise().mean().x());
            segYMeanBuff.push_back(segCloudMat.colwise().mean().y());
            double resR = float(2) * PI * sqrt(segCloudMat.colwise().mean().x()*segCloudMat.colwise().mean().x() +
                                               segCloudMat.colwise().mean().y()*segCloudMat.colwise().mean().y())/ float(1024);

            if (segXMeanBuff.size() == 3){
                float spacing20 = std::sqrt((segXMeanBuff.at(2)-segXMeanBuff.at(0))*(segXMeanBuff.at(2)-segXMeanBuff.at(0))
                                            +(segYMeanBuff.at(2)-segYMeanBuff.at(0))*(segYMeanBuff.at(2)-segYMeanBuff.at(0)));
                float spacing21 = std::sqrt((segXMeanBuff.at(2)-segXMeanBuff.at(1))*(segXMeanBuff.at(2)-segXMeanBuff.at(1))
                                            +(segYMeanBuff.at(2)-segYMeanBuff.at(1))*(segYMeanBuff.at(2)-segYMeanBuff.at(1)));
                float spacing10 = std::sqrt((segXMeanBuff.at(0)-segXMeanBuff.at(1))*(segXMeanBuff.at(0)-segXMeanBuff.at(1))
                                            +(segYMeanBuff.at(0)-segYMeanBuff.at(1))*(segYMeanBuff.at(0)-segYMeanBuff.at(1)));
                if (spacing20 > params_.segSpacingThd && spacing21 > params_.segSpacingThd && spacing10 > params_.segSpacingThd){
                    for (int i = 0; i < zidxVec.size(); ++i) {
                        segmentedPole -> push_back(segmentedCloudIsolated->points[zidxVec.at(i)]);
                    }
                    //check whether need to fit the cylinder
                    if (width > params_.wThd){
                        pcl::PointCloud<PointType>::Ptr project2EigenCloud(new pcl::PointCloud<PointType>);
                        project2EigenCloud->clear();
                        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
                        T.block<3,3>(0,0) = eigenVec;
                        T.block<3,1>(0,3) = segCloudMat.colwise().mean().transpose();
                        pcl::transformPointCloud(*topCutCloud, *project2EigenCloud, T.inverse());
                        project2EigenCloud->clear();
                        topCutCloud->clear();
                        PointType stretchPoint;
                        stretchPoint.x = 1.5 * sin(atan(T(1,3)/T(0,3))) * resR + T(0,3);
                        stretchPoint.y = 1.5 * cos(atan(T(1,3)/T(0,3))) * resR + T(1,3);
                        stretchPoint.z = T(2,3);
                        topCutCloud->push_back(stretchPoint);
                        pcl::transformPointCloud(*topCutCloud, *project2EigenCloud, T.inverse());
                        stretchPoint = project2EigenCloud->points[0];
                        // project the segCloudMat to the eigen space, and only use the space perpendicular to the main dir
                        for (int i = 0; i < zidxVec.size(); ++i) {
                            px = project2EigenCloud->points[zidxVec.at(i)].x;
                            py = project2EigenCloud->points[zidxVec.at(i)].y;
                            pz = project2EigenCloud->points[zidxVec.at(i)].z;
                            Pxyz << px, py, pz;
                            segCloudMat.row(i) = Pxyz;
                        }
                        Eigen::MatrixXf project2EigenSpaceMat(zidxVec.size(),2);
                        project2EigenSpaceMat = segCloudMat.leftCols(2);
                        // initialize the params
                        float meanX, meanY;
                        meanX = project2EigenSpaceMat.col(0).mean();
                        meanY = project2EigenSpaceMat.col(1).mean();
                        // save ini value for comparision
                        double xIni = meanX + stretchPoint.x;
                        double yIni = meanY + stretchPoint.y;
                        double rIni = resR;
                    }

                }
            } else{// if the buff haven't reached 3, fill in
                for (int i = 0; i < zidxVec.size(); ++i) {
                    poleOdom[0] = segCloudMat.colwise().mean().x();
                    poleOdom[1] = segCloudMat.colwise().mean().y();
                    poleOdom[2] = segCloudMat.colwise().mean().z();
                    poleOdom[3] = 2;
                    poleOdom[4] = segCloudMat.colwise().mean().x();
                    poleOdom[5] = segCloudMat.colwise().mean().y();
                    poleOdom[6] = segCloudMat.colwise().mean().z();
                    segmentedPole -> push_back(segmentedCloudIsolated->points[zidxVec.at(i)]);
                }
            }
        }

    }



    void publishCloud(){
        // 1. Publish Seg Cloud Info
        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);
        // 2. Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubOutlierCloud.publish(laserCloudTemp);
        // segmented cloud with ground
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubSegmentedCloud.publish(laserCloudTemp);
        // projected full cloud
        if (pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullCloud.publish(laserCloudTemp);
        }
        // original dense ground cloud
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubGroundCloud.publish(laserCloudTemp);
        }
        // segmented cloud without ground
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }
        // projected full cloud info
        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");

    ImageProjection IP;

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin();
    return 0;
}
