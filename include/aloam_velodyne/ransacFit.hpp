//
// Created by xingyu on 20/7/22.
//

#ifndef ALOAM_VELODYNE_RANSACFIT_HPP
#define ALOAM_VELODYNE_RANSACFIT_HPP

#include "utility.hpp"

class ransacFit{
private:

    std::shared_ptr<float> modelParam;
public:

    const uint8_t CYLINDER_FIT = 0;
    const uint8_t PLANE_FIT = 1;
    const uint8_t STICK_FIT = 2;
    const uint8_t LINE_FIT = 3;
    const uint8_t CIRCLE_FIT = 4;

    ransacFit(pcl::PointCloud<PointType>::Ptr segCloudPtr, uint8_t fittingType);

    std::shared_ptr<float> sharedPtrFit(pcl::PointCloud<PointType>::Ptr segCloudPtr);

    std::shared_ptr<float> cylinderFit(pcl::PointCloud<PointType>::Ptr segCloudPtr);
    std::shared_ptr<float> planeFit(pcl::PointCloud<PointType>::Ptr segCloudPtr);
    std::shared_ptr<float> stickFit(pcl::PointCloud<PointType>::Ptr segCloudPtr);
    std::shared_ptr<float> lineFit(pcl::PointCloud<PointType>::Ptr segCloudPtr);
    std::shared_ptr<float> circleFit(pcl::PointCloud<PointType>::Ptr segCloudPtr);

};

#endif //ALOAM_VELODYNE_RANSACFIT_HPP
