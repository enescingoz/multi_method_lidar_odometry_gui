#include <viz3d/ui_windows/trajectory_vis.h>

#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

namespace viz3d
{

    TrajectoryVis::TrajectoryVis(std::string title)
        : VTKWindow(std::move(title))
    {
        InitializeWindow();
        render_left_panel = true; // Enable left panel rendering
    }

    void TrajectoryVis::RenderLeftPanel()
    {
        // Configure the left panel using VTKWindow settings
        ConfigureLeftPanel();

        // Begin the floating window with additional flags to make it non-movable and non-resizable
        ImGui::Begin("LeftPanelWindow", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

        // Add new elements to the top of the left panel
        ImGui::Text("Additional elements at the top");
        if (ImGui::Button("Additional Button"))
        {
            // Button action
        }


        ImGui::End();
    }

    void viz3d::TrajectoryVis::DrawImGUIContent()
    {
        VTKWindow::DrawImGUIContent(); // Call the base class implementation
    }

    void TrajectoryVis::InitializeWindow()
    {
        // Initialize the actors
        pointCloudActor = GetPointCloudActor(); // Assume GetPointCloudActor is available
        lineActor = GetLineActor();             // Assume GetLineActor is available

        // Add actors to the window
        AddActor(pointCloudActor);
        AddActor(lineActor);
    }

} // namespace viz3d
