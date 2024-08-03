#ifndef LIDAR_ODOM_VIEW_H
#define LIDAR_ODOM_VIEW_H

#include <lidar_odometry/lidar_odometry.h>
#include <viz3d/vtk_window.h>
#include <viz3d/vtk_actors.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <memory>
#include <portable-file-dialogs.h>

namespace viz3d {

class LidarOdomView : public viz3d::VTKWindow {
public:
    LidarOdomView(std::string title);

    void RenderLeftPanel() override;  // Override the RenderLeftPanel function
    void DrawImGUIContent() override;  // Ensure this function is overridden

    // Method to initialize the window with actors
    void InitializeWindow();

private:
    // ImGui function to create the dropdown menu
    void ShowRegistrationDropdown();

    // ImGui function to create text boxes for the parameters
    void ShowParameterInputs();

    // ImGui function to create the folder loading button and textbox
    void ShowLoadFolder();

    // Function to start registration
    void ShowLidarOdom();

    // Convert lidar odometry parameters to yaml node
    YAML::Node paramsToYaml() const;

    // Private members to store actors or additional settings if needed
    vtkSmartPointer<vtkActor> pointCloudActor;
    vtkSmartPointer<vtkActor> lineActor;

    // LidarOdometry object
    std::shared_ptr<LidarOdometry> lidar_odometry_;

    //
    // Enumeration for registration methods
    enum RegistrationMethod_ {
        ICP,
        ICP_N,
        ICP_NL,
        GICP,
        NDT,
        REGISTRATION_METHOD_COUNT // Keep this as the last element
    };

    // Array of method names
    const char* registration_methods_[REGISTRATION_METHOD_COUNT] = {
        "ICP",
        "ICP-N",
        "ICP-NL",
        "GICP",
        "NDT"
    };

    // Variable to hold the selected method
    int selected_reg_method_ = ICP;

    // Parameters for registration methods
    float icp_trans_eps_ = 1e-6f;
    float icp_max_corr_dist_ = 0.05f;
    int icp_max_iter_ = 50;

    float ndt_trans_eps_ = 0.01f;
    float ndt_step_size_ = 0.1f;
    float ndt_res_ = 1.0f;
    int ndt_max_iter_ = 35;

    float leaf_size_ = 0.1;

    // Variable to hold the selected folder path
    std::string pcds_folder_;

};

} // namespace viz3d

#endif // LIDAR_ODOM_VIEW_H
