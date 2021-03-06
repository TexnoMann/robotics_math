cmake_minimum_required(VERSION 2.8.3)
project(robotics_math CXX)
add_compile_options(-std=c++11)
find_package(catkin QUIET)

if(catkin_FOUND)
    message([STATUS] "Catkin build")
else()
    message([STATUS] "Build without catkin")
endif()

find_package(Eigen3 REQUIRED Core)
find_package(PythonLibs 2.7)

if(catkin_FOUND)
    catkin_package(
            INCLUDE_DIRS src matplotlibcpp
            CATKIN_DEPENDS roscpp std_msgs
            LIBRARIES robotics_math)
endif()

add_library(robotics_math SHARED
        src/trj_planning/trj_planer.h
        src/trj_planning/trj_planer.cpp
        src/trj_planning/curves/besie_spline.h
        src/trj_planning/curves/besie_spline.cpp
        src/trj_planning/planners/lspb.h
        src/trj_planning/planners/lspb.cpp
        src/trj_planning/planners/cubic_spline.h
        src/trj_planning/planners/cubic_spline.cpp

        src/path_planning/graph_map.h
        src/path_planning/graph_map.cpp

        src/controllers/pid.h
        src/controllers/pid.cpp

        src/localization/monte_carlo.h
        src/localization/monte_carlo.cpp

        src/algorithms/pc_vec/pointcloud_vectorize.h
        src/algorithms/pc_vec/pointcloud_vectorize.cpp

#        visual tools
        matplotlibcpp/matplotlibcpp.h
        )

if(catkin_FOUND)
    include_directories(
            ${catkin_INCLUDE_DIRS}
            ${Eigen_INCLUDE_DIRS}
            ${PYTHON_INCLUDE_DIRS})
endif()

target_link_libraries(robotics_math Eigen3::Eigen ${PYTHON_LIBRARIES})


if(catkin_FOUND)
    install(TARGETS robotics_math
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

    install(DIRECTORY
            src/trj_planning
            src/path_planning
            src/controllers
            src/localization
            src/algorithms
            matplotlibcpp
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
endif()

## TESTS

add_executable(test_lspb_planer src/trj_planning/test/lspb_test.cpp)
target_link_libraries(test_lspb_planer robotics_math)

add_executable(test_graph_gen src/path_planning/test/graph_generation_test.cpp)
target_link_libraries(test_graph_gen robotics_math)

add_executable(test_besie_splines src/trj_planning/test/besie_spline_test.cpp)
target_link_libraries(test_besie_splines robotics_math)

add_executable(test_mcl src/localization/test/monte_carlo_localization_test.cpp)
target_link_libraries(test_mcl robotics_math)

add_executable(test_tls src/algorithms/pc_vec/test/tls_linear_fit_test.cpp)
target_link_libraries(test_tls robotics_math)

add_executable(test_pcv src/algorithms/pc_vec/test/point_cloud_vectorization_test.cpp)
target_link_libraries(test_pcv robotics_math)