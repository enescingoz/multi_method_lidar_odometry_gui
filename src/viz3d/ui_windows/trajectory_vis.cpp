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

        // Begin the floating window
        ImGui::Begin("LeftPanelFloatingWindow");

        // Add a CollapsingHeader to the window
        if (ImGui::CollapsingHeader("Left Panel", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Text("This is a test text in the left panel.");
            // Add more UI elements here as needed
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
