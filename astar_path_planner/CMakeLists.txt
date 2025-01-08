cmake_minimum_required(VERSION 3.0.2)
project(astar_path_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 catkin 和必要的 ROS 包
find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    geometry_msgs
)

# 查找 Eigen3 和 OsqpEigen
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

# catkin_package 声明
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp visualization_msgs geometry_msgs
)

# 包含目录
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    /usr/local/include/osqp
)

# 添加可执行文件和链接库
add_executable(obstacle_generator src/obstacle_generator.cpp)
target_link_libraries(obstacle_generator
    ${catkin_LIBRARIES}
)

add_executable(astar_planner src/astar_planner.cpp)
target_link_libraries(astar_planner
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)

add_executable(trajectory src/Trajectory.cpp)
target_link_libraries(trajectory
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    OsqpEigen::OsqpEigen
    osqp
)

