#include <lidar_odometry/lidar_odometry.h>

LidarOdometry::LidarOdometry()
{
}

void LidarOdometry::setParameters(const YAML::Node &config)
{
    leaf_size_ = config["leaf_size"].as<float>();
    registration_method_ = config["registration_method"].as<std::string>();

    // Configure NDT
    if (registration_method_ == "ndt")
    {
        ndt_.setTransformationEpsilon(config["ndt"]["trans_eps"].as<double>());
        ndt_.setStepSize(config["ndt"]["step_size"].as<double>());
        ndt_.setResolution(config["ndt"]["res"].as<float>());
        ndt_.setMaximumIterations(config["ndt"]["max_iter"].as<int>());
    }

    // Configure ICP
    if (registration_method_ == "icp")
    {
        icp_.setTransformationEpsilon(config["icp"]["trans_eps"].as<double>());
        icp_.setMaxCorrespondenceDistance(config["icp"]["max_corr_dist"].as<double>());
        icp_.setMaximumIterations(config["icp"]["max_iter"].as<int>());
    }
}

void LidarOdometry::downsamplePointCloud(const PointCloudT::Ptr &input, PointCloudT::Ptr &output)
{
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxel_grid.setInputCloud(input);
    voxel_grid.filter(*output);
}

Eigen::Matrix4f LidarOdometry::registerPointClouds(const PointCloudT::Ptr &target, const PointCloudT::Ptr &source)
{
    if (registration_method_ == "NDT")
    {
        ndt_.setInputTarget(target);
        ndt_.setInputSource(source);

        PointCloudT::Ptr output_cloud(new PointCloudT);
        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
        ndt_.align(*output_cloud, initial_guess);

        return ndt_.getFinalTransformation();
    }
    else if (registration_method_ == "ICP")
    {
        icp_.setInputTarget(target);
        icp_.setInputSource(source);

        PointCloudT::Ptr output_cloud(new PointCloudT);
        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
        icp_.align(*output_cloud, initial_guess);

        return icp_.getFinalTransformation();
    }
    else
    {
        throw std::runtime_error("Unknown registration method: " + registration_method_);
    }
}

void LidarOdometry::processPointClouds(const std::string &path)
{
    int idx = 0;
    target_cloud_ = PointCloudT::Ptr(new PointCloudT());

    while (true)
    {
        std::string filename = path + "/" + std::to_string(idx) + ".pcd";
        PointCloudT::Ptr source_cloud(new PointCloudT);
        if (pcl::io::loadPCDFile<PointT>(filename, *source_cloud) == -1)
        {
            break; // No more files to process
        }

        PointCloudT::Ptr filtered_source_cloud(new PointCloudT);
        downsamplePointCloud(source_cloud, filtered_source_cloud);

        if (idx == 0)
        {
            *target_cloud_ = *filtered_source_cloud;
            odometry_.emplace_back(Eigen::Matrix4f::Identity());
        }
        else
        {
            Eigen::Matrix4f transform = registerPointClouds(target_cloud_, filtered_source_cloud);
            odometry_.emplace_back(odometry_.back() * transform);
            *target_cloud_ = *filtered_source_cloud;
            std::cout << "Registered " << idx-1 << " and " << idx << " clouds" << std::endl;
        }

        idx++;
    }
    std::cout << "Odometry generated" << std::endl;
}

void LidarOdometry::saveOdometry(const std::string &save_directory)
{
    // Open the CSV file for writing
    std::ofstream csv_file(save_directory + "/odometry.csv");
    if (!csv_file.is_open())
    {
        std::cerr << "Failed to open CSV file for writing" << std::endl;
        return;
    }

    // Create an empty point cloud for the result
    PointCloudT::Ptr result_cloud(new PointCloudT);

    for (const auto &position : odometry_)
    {
        // Extract the translation part from the transformation matrix
        float x = position(0, 3);
        float y = position(1, 3);
        float z = position(2, 3);

        // Write the position to the CSV file
        csv_file << x << "," << y << "," << z << "\n";

        // Create a point cloud with a single point for the current position
        PointCloudT::Ptr single_point_cloud(new PointCloudT);
        PointT point;
        point.x = x;
        point.y = y;
        point.z = z;
        single_point_cloud->points.push_back(point);

        // Add the single point to the result cloud
        *result_cloud += *single_point_cloud;
    }

    // Close the CSV file
    csv_file.close();

    // Save the result point cloud to a PCD file
    pcl::io::savePCDFileBinary(save_directory + "/odometry.pcd", *result_cloud);

    std::cout << "Odometry saved" << std::endl;
}
