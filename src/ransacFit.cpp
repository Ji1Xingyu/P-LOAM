//
// Created by xingyu on 20/7/22.
//
#include "aloam_velodyne/ransacFit.hpp"

ransacFit::ransacFit(pcl::PointCloud<PointType>::Ptr segCloudPtr, const uint8_t fittingType) {
    switch (fittingType) {
        case 0: modelParam = cylinderFit(segCloudPtr);
        case 1: modelParam = planeFit(segCloudPtr);
        case 2: modelParam = stickFit(segCloudPtr);
        case 3: modelParam = lineFit(segCloudPtr);
        case 4: modelParam = circleFit(segCloudPtr);

    }
}

std::shared_ptr<float> ransacFit::cylinderFit(pcl::PointCloud<PointType>::Ptr segCloudPtr) {

    using namespace std::chrono_literals;

        // initialize PointClouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

        std::vector<int> inliers;

        // created RandomSampleConsensus object and compute the appropriated model
        pcl::SampleConsensusModelCylinder<>
        pcl::SampleConsensusModelCylinder<PointType>::Ptr
                model_c(new pcl::SampleConsensusModelSphere<pcl::PointType> (segCloudPtr));
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
                model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
        if(pcl::console::find_argument (argc, argv, "-f") >= 0)
        {
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
            ransac.setDistanceThreshold (.01);
            ransac.computeModel();
            ransac.getInliers(inliers);
        }
        else if (pcl::console::find_argument (argc, argv, "-sf") >= 0 )
        {
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
            ransac.setDistanceThreshold (.01);
            ransac.computeModel();
            ransac.getInliers(inliers);
        }

        // copies all inliers of the model computed to another PointCloud
        pcl::copyPointCloud (*cloud, inliers, *final);

        // creates the visualization object and adds either our original cloud or all of the inliers
        // depending on the command line arguments specified.
        pcl::visualization::PCLVisualizer::Ptr viewer;
        if (pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
            viewer = simpleVis(final);
        else
            viewer = simpleVis(cloud);
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            std::this_thread::sleep_for(100ms);
        }
        return 0;
    }
    return nullptr;
}

std::shared_ptr<float> ransacFit::planeFit(pcl::PointCloud<PointType>::Ptr segCloudPtr) {
    return nullptr;
}

std::shared_ptr<float> ransacFit::stickFit(pcl::PointCloud<PointType>::Ptr segCloudPtr) {
    return nullptr;
}

std::shared_ptr<float> ransacFit::lineFit(pcl::PointCloud<PointType>::Ptr segCloudPtr) {
    return nullptr;
}

std::shared_ptr<float> ransacFit::circleFit(pcl::PointCloud<PointType>::Ptr segCloudPtr) {
    return nullptr;
}

