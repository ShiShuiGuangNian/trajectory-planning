set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.0.2)
project(astar_path_planner)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        visualization_msgs
        geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${QPOASES_INCLUDE_DIRS}
)



add_executable(obstacle_generator src/obstacle_generator.cpp)
target_link_libraries(obstacle_generator ${catkin_LIBRARIES})


add_executable(astar_planner_1 src/astar_planner_1.cpp)
target_link_libraries(astar_planner_1 ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(astar_planner_2 src/astar_planner_2.cpp)
target_link_libraries(astar_planner_2 ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} )



