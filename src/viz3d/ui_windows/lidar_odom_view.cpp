#include <viz3d/ui_windows/lidar_odom_view.h>

namespace viz3d
{

    LidarOdomView::LidarOdomView(std::string title)
        : VTKWindow(std::move(title))
    {
        InitializeWindow();
        render_left_panel = true; // Enable left panel rendering
        lidar_odometry_ = std::make_shared<LidarOdometry>();
    }

    void LidarOdomView::RenderLeftPanel()
    {
        // Configure the left panel using VTKWindow settings
        ConfigureLeftPanel();

        // Begin the floating window with additional flags to make it non-movable and non-resizable
        ImGui::Begin("Lidar Odometry Configurations", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

        ShowRegistrationDropdown();
        ShowParameterInputs();
        ShowLoadFolder();
        ShowLidarOdom();

        ImGui::End();
    }

    void viz3d::LidarOdomView::DrawImGUIContent()
    {
        VTKWindow::DrawImGUIContent(); // Call the base class implementation
    }

    void LidarOdomView::InitializeWindow()
    {
        // Initialize the actors
        pointCloudActor = GetPointCloudActor(); // Assume GetPointCloudActor is available
        lineActor = GetLineActor();             // Assume GetLineActor is available

        // Add actors to the window
        AddActor(pointCloudActor);
        AddActor(lineActor);
    }

    void LidarOdomView::ShowRegistrationDropdown()
    {
        if (ImGui::CollapsingHeader("Registration Methods", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Combo("Select Method", &selected_reg_method_, registration_methods_, REGISTRATION_METHOD_COUNT);
            ImGui::Separator(); // Add a divider
        }
    }

    void LidarOdomView::ShowParameterInputs()
    {

        if (ImGui::CollapsingHeader("Parameters", ImGuiTreeNodeFlags_DefaultOpen))
        {
            switch (selected_reg_method_)
            {
            case ICP:
                ImGui::InputFloat("Transformation Epsilon", &icp_trans_eps_, 0.0f, 0.0f, "%.7f");
                ImGui::InputFloat("Max Correspondence Distance", &icp_max_corr_dist_, 0.0f, 0.0f, "%.7f");
                ImGui::InputInt("Maximum Iterations", &icp_max_iter_);
                break;
            case ICP_N:
                ImGui::InputFloat("Transformation Epsilon", &icp_trans_eps_, 0.0f, 0.0f, "%.7f");
                ImGui::InputFloat("Max Correspondence Distance", &icp_max_corr_dist_, 0.0f, 0.0f, "%.7f");
                ImGui::InputInt("Maximum Iterations", &icp_max_iter_);
                break;
            case ICP_NL:
                ImGui::InputFloat("Transformation Epsilon", &icp_trans_eps_, 0.0f, 0.0f, "%.7f");
                ImGui::InputFloat("Max Correspondence Distance", &icp_max_corr_dist_, 0.0f, 0.0f, "%.7f");
                ImGui::InputInt("Maximum Iterations", &icp_max_iter_);
                break;
            case GICP:
                ImGui::InputFloat("Transformation Epsilon", &icp_trans_eps_, 0.0f, 0.0f, "%.7f");
                ImGui::InputFloat("Max Correspondence Distance", &icp_max_corr_dist_, 0.0f, 0.0f, "%.7f");
                ImGui::InputInt("Maximum Iterations", &icp_max_iter_);
                break;
            case NDT:
                ImGui::InputFloat("Transformation Epsilon", &ndt_trans_eps_, 0.0f, 0.0f, "%.7f");
                ImGui::InputFloat("Step Size", &ndt_step_size_, 0.0f, 0.0f, "%.7f");
                ImGui::InputFloat("Resolution", &ndt_res_, 0.0f, 0.0f, "%.7f");
                ImGui::InputInt("Maximum Iterations", &ndt_max_iter_);
                break;
            }
            ImGui::InputFloat("Downsample Leaf Size", &leaf_size_, 0.0f, 0.0f, "%.7f");
            ImGui::Separator(); // Add a divider
        }
    }

    void LidarOdomView::ShowLoadFolder()
    {
        static char buffer[256] = ""; // Buffer to hold the folder path

        // Load Graph Section
        if (ImGui::CollapsingHeader("Load pointclouds", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Button("Select"))
            {
                auto dir = pfd::select_folder("Select a folder").result();
                if (!dir.empty())
                {
                    pcds_folder_ = dir;
                    strncpy(buffer, pcds_folder_.c_str(), sizeof(buffer));
                    buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-termination
                }
            }
            ImGui::SameLine();

            // Set the width of the InputText widget
            ImGui::InputText("##Folder Path", buffer, IM_ARRAYSIZE(buffer));

            ImGui::Separator(); // Add a divider
        }
    }

    void LidarOdomView::ShowLidarOdom()
    {
        if (ImGui::CollapsingHeader("Lidar Odometry", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Button("Start"))
            {
                YAML::Node config = paramsToYaml();
                lidar_odometry_->setParameters(config);
                lidar_odometry_->processPointClouds(pcds_folder_);
            }

            if (ImGui::Button("Save Odometry"))
            {
                auto dir = pfd::select_folder("Select a folder").result();
                lidar_odometry_->saveOdometry(dir);
            }

            ImGui::Separator(); // Add a divider
        }
    }

    YAML::Node LidarOdomView::paramsToYaml() const
    {
        YAML::Node node;
        node["registration_method"] = registration_methods_[selected_reg_method_];

        if (selected_reg_method_ == ICP || selected_reg_method_ == ICP_N || selected_reg_method_ == ICP_NL || selected_reg_method_ == GICP)
        {
            node["icp"]["trans_eps"] = icp_trans_eps_;
            node["icp"]["max_corr_dist"] = icp_max_corr_dist_;
            node["icp"]["max_iter"] = icp_max_iter_;
        }

        if (selected_reg_method_ == NDT)
        {
            node["ndt"]["trans_eps"] = ndt_trans_eps_;
            node["ndt"]["step_size"] = ndt_step_size_;
            node["ndt"]["res"] = ndt_res_;
            node["ndt"]["max_iter"] = ndt_max_iter_;
        }
        node["leaf_size"] = leaf_size_;

        node["pcds_folder"] = pcds_folder_;

        return node;
    }

} // namespace viz3d
