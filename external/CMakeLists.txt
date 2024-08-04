cmake_minimum_required(VERSION 3.14)
project(VIZ3D-EXTERNAL)

option(BUILD_GLOG "Whether to build GLOG from sources" ON)
option(BUILD_EIGEN "Whether to download EIGEN sources" ON)
option(BUILD_VTK "Whether to build VTK from sources" ON)
option(BUILD_PCL "Whether to build PCL from sources" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(ExternalProject)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/global_vars.cmake)
set(GLOBAL_RUNTIME_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../bin)

if (BUILD_GLOG)
    # INSTALL GLOG
    set(GLOG_DEF "glog")
    ExternalProject_Add(
            ${GLOG_DEF}
            PREFIX ${GLOG_DEF}

            GIT_REPOSITORY https://github.com/google/glog
            GIT_TAG v0.5.0

            BUILD_ALWAYS OFF
            INSTALL_DIR ${INSTALL_ROOT}/glog
            CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${INSTALL_ROOT}/glog
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON

            STEP_TARGETS glog-build
            BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install)
endif ()

if (BUILD_EIGEN)
    # INSTALL EIGEN
    ExternalProject_Add(
            Eigen3
            PREFIX Eigen3

            GIT_REPOSITORY https://gitlab.com/libeigen/eigen
            GIT_TAG 3.3.7

            BUILD_ALWAYS OFF
            INSTALL_DIR ${INSTALL_ROOT}/Eigen3
            CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${INSTALL_ROOT}/Eigen3
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON

            BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install)
endif ()

# INSTALL GLAD
ExternalProject_Add(
        glad
        PREFIX glad

        GIT_REPOSITORY https://github.com/Dav1dde/glad
        GIT_TAG v0.1.34

        BUILD_ALWAYS OFF
        INSTALL_DIR ${INSTALL_ROOT}/glad
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${INSTALL_ROOT}/glad
        -DGLAD_INSTALL=ON
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON

        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install)


# INSTALL GLFW
ExternalProject_Add(
        glfw
        PREFIX glfw

        GIT_REPOSITORY https://github.com/glfw/glfw
        GIT_TAG 3.3.3

        BUILD_ALWAYS OFF
        INSTALL_DIR ${INSTALL_ROOT}/glfw
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${INSTALL_ROOT}/glfw
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON

        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install)

if (BUILD_VTK)
    # INSTALL VTK FROM SOURCES
    ExternalProject_Add(
            vtk
            PREFIX vtk

            GIT_REPOSITORY https://gitlab.kitware.com/vtk/vtk.git
            GIT_TAG v9.1.0

            BUILD_ALWAYS OFF
            INSTALL_DIR ${INSTALL_ROOT}/vtk
            CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${INSTALL_ROOT}/vtk
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            -DVTK_USE_CUDA=OFF
            -DVTK_BUILD_EXAMPLES=ON
            -DVTK_BUILD_DOCUMENTATION=OFF
            -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
            -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=TRUE
            -DCMAKE_INSTALL_RPATH=${INSTALL_ROOT}/vtk/lib

            BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install --parallel 6)
endif ()


if (BUILD_PCL)
    # INSTALL PCL
    ExternalProject_Add(
        pcl
        GIT_REPOSITORY https://github.com/PointCloudLibrary/pcl
        GIT_TAG pcl-1.14.1
        BUILD_ALWAYS OFF
        INSTALL_DIR ${INSTALL_ROOT}/pcl
        CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${INSTALL_ROOT}/pcl
            -DWITH_VTK=OFF
            -DWITH_OPENGL=OFF
            -DBUILD_apps=OFF
            -DBUILD_examples=OFF
            -DBUILD_common=ON
            -DBUILD_io=ON
            -DBUILD_registration=ON
            -DBUILD_sample_consensus=ON
            -DBUILD_octree=ON
            -DBUILD_filters=ON
            -DBUILD_kdtree=ON
            -DBUILD_features=OFF
            -DBUILD_geometry=OFF
            -DBUILD_segmentation=OFF
            -DBUILD_visualization=OFF
            -DBUILD_search=OFF
            -DBUILD_surface=OFF
            -DBUILD_keypoints=OFF
            -DBUILD_people=OFF
            -DBUILD_outofcore=OFF
            -DBUILD_simulation=OFF
            -DBUILD_tracking=OFF
            -DBUILD_recognition=OFF
            -DBUILD_stereo=OFF
            -DBUILD_ml=OFF
            -DBUILD_tools=OFF
            -DBUILD_test=OFF
            -DCMAKE_BUILD_TYPE=Release
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install
    )
endif ()


# INSTALL YAML-CPP
ExternalProject_Add(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp
    GIT_TAG 0.8.0
    BUILD_ALWAYS OFF
    INSTALL_DIR ${INSTALL_ROOT}/yaml-cpp
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${INSTALL_ROOT}/yaml-cpp
        -DYAML_BUILD_SHARED_LIBS=on
        -DCMAKE_BUILD_TYPE=Release
    BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install
)