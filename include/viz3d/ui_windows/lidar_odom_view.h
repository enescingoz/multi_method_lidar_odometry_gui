#ifndef LIDAR_ODOM_VIEW_H
#define LIDAR_ODOM_VIEW_H

#include <viz3d/vtk_window.h>
#include <viz3d/vtk_actors.h>
#include <vtkSmartPointer.h>
#include <string>

namespace viz3d {

class LidarOdomView : public viz3d::VTKWindow {
public:
    LidarOdomView(std::string title);

    void RenderLeftPanel() override;  // Override the RenderLeftPanel function
    void DrawImGUIContent() override;  // Ensure this function is overridden

    // Method to initialize the window with actors
    void InitializeWindow();

private:
    // Private members to store actors or additional settings if needed
    vtkSmartPointer<vtkActor> pointCloudActor;
    vtkSmartPointer<vtkActor> lineActor;
};

} // namespace viz3d

#endif // LIDAR_ODOM_VIEW_H
