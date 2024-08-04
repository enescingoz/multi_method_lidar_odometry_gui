set(CPP_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})
set(H_PREFIX ${LIOM_INCLUDE_DIR}/lidar_odometry)


LIOM_ADD_SRC(
        ${H_PREFIX}/lidar_odometry.h ${CPP_PREFIX}/lidar_odometry.cpp
        )