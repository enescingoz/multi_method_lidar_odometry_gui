#include <memory>
#include <thread>
#include <viz3d/ui.h>
#include <viz3d/vtk_window.h>
#include <viz3d/vtk_actors.h>
#include <misc/cpp/imgui_stdlib.h>
#include <Eigen/Dense>
#include <iostream>  // Include for std::cerr

int main(int argc, char **argv) {
    auto &gui = viz3d::GUI::Instance();
    gui.AddWindow(std::make_shared<viz3d::TestWindow>("Test Window"));

    {
        auto vtk_window = std::make_shared<viz3d::VTKWindow>("VTK Window");
        vtk_window->AddActor(GetConeActor());
        gui.AddWindow(vtk_window);
    }

    size_t window_id;
    {
        auto vtk_window2 = std::make_shared<viz3d::VTKWindow>("VTK Window 2");
        vtk_window2->AddActor(GetPointCloudActor());
        vtk_window2->AddActor(GetLineActor());
        window_id = gui.AddWindow(vtk_window2);
    }

    {
        auto vtk_window3 = std::make_shared<viz3d::VTKWindow>("VTK Window 3");
        vtk_window3->AddActor(GetPointCloudActor());
        vtk_window3->AddActor(GetLineActor());
        window_id = gui.AddWindow(vtk_window3);
    }

    std::thread gui_thread{viz3d::GUI::LaunchMainLoop, "GUI"};
    gui_thread.join();
    gui.RemoveWindow(window_id);
    gui.ClearWindows();

    return 0;
}