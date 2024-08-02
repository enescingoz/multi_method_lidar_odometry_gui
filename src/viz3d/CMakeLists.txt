set(CPP_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})
set(H_PREFIX ${VIZ3D_INCLUDE_DIR}/viz3d)

# WINDOWS
set(WINDOWS_CPP_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/ui_windows)
set(WINDOWS_H_PREFIX ${VIZ3D_INCLUDE_DIR}/viz3d/ui_windows)


VIZ3D_ADD_SRC(
        ${CPP_PREFIX}/ui.cpp ${H_PREFIX}/ui.h
        ${H_PREFIX}/vtk_window.h ${CPP_PREFIX}/vtk_window.cpp
        ${H_PREFIX}/vtk_actors.h ${CPP_PREFIX}/vtk_actors.cpp
        ${WINDOWS_H_PREFIX}/lidar_odom_view.h ${WINDOWS_CPP_PREFIX}/lidar_odom_view.cpp
        )