cmake_minimum_required(VERSION 2.8.3)
project(traj_opt)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(osqp REQUIRED)
find_package(ompl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    visualization_msgs
    geometry_msgs
    sensor_msgs
    traj_utils
    # path_searching
    # plan_env
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES traj_opt
    CATKIN_DEPENDS plan_env traj_utils

    # DEPENDS osqp
)

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

add_library(traj_opt
    # src/poly_traj_optimizer.cpp
    src/mini_snap.cpp
    src/bezier_optimizer.cpp
)

target_link_libraries(traj_opt
    ${catkin_LIBRARIES}
    osqp::osqp
)

add_executable(vis_minisnap test/vis_minisnap.cpp)
target_link_libraries(vis_minisnap
    ${catkin_LIBRARIES}
    traj_opt
)

add_executable(vis_bezier_opt test/vis_bezier_opt.cpp)
target_link_libraries(vis_bezier_opt
    ${catkin_LIBRARIES}
    traj_opt
)

add_executable(minco_node src/minco_node.cpp)
target_link_libraries(minco_node
    ${catkin_LIBRARIES}
    ${OMPL_LIBRARIES}
    traj_opt
)

# #########
# # Test ##
# #########
if(CATKIN_ENABLE_TESTING)
    # find_package(rostest REQUIRED)
    catkin_add_gtest(test_bezier_opt test/test_bezier_opt.cpp)

    # add_rostest_gtest(test_bernstein test/test_bernstein.cpp)
    target_link_libraries(test_bezier_opt traj_opt ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
endif()
