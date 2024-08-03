#ifndef LIDAR_ODOMETRY_H
#define LIDAR_ODOMETRY_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class LidarOdometry {
public:
    LidarOdometry();
    void processPointClouds(const std::string& path);
    void saveOdometry(const std::string& save_directory);
    void setParameters(const YAML::Node& config);
    PointCloudT::Ptr generateTrajectoryCloud(const std::vector<Eigen::Matrix4f> &odometry_);


private:
    void downsamplePointCloud(const PointCloudT::Ptr& input, PointCloudT::Ptr& output);
    Eigen::Matrix4f registerPointClouds(const PointCloudT::Ptr& target, const PointCloudT::Ptr& source);
    
    float leaf_size_;
    std::string registration_method_;
    pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
    pcl::IterativeClosestPoint<PointT, PointT> icp_;
    PointCloudT::Ptr target_cloud_;
    std::vector<Eigen::Matrix4f> odometry_;
};

#endif // LIDAR_ODOMETRY_H
