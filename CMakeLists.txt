cmake_minimum_required(VERSION 2.8.3)
project(robotics_math CXX)

#find_package(catkin QUIET)

if(catkin_FOUND)
    message([STATUS] "Catkin build")
else()
    message([STATUS] "Build without catkin")
endif()

find_package(Eigen3 REQUIRED Core)

add_library(robotics_math SHARED
        trj_planning/trj_planer.h
        trj_planning/besie_spline.h
        trj_planning/besie_spline.cpp
        trj_planning/planners/lspb.h
        trj_planning/planners/lspb.cpp
        trj_planning/planners/cubic_spline.h
        trj_planning/planners/cubic_spline.cpp
        )

if(catkin_FOUND)
    include_directories(
            ${catkin_INCLUDE_DIRS})
endif()
target_link_libraries(robotics_math Eigen3::Eigen)