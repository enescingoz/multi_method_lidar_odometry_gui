# Build Test Executables
# add_executable(example_vtk example_vtk.cxx)
# target_link_libraries(example_vtk PUBLIC
#         ${VTK_LIBRARIES} OpenGL::GL glfw glad::glad glog::glog Eigen3::Eigen imgui viz3d)
# install(TARGETS example_vtk RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})


add_executable(main main.cxx)
target_link_libraries(main PUBLIC
        ${VTK_LIBRARIES} OpenGL::GL glfw glad::glad glog::glog Eigen3::Eigen imgui viz3d)
install(TARGETS main RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})