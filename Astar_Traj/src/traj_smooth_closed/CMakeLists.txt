set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 2.8.3)

project(traj_smooth_closed)

set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  message_generation
)

add_message_files(
  FILES
  TrajData_msgs.msg  
)

generate_messages(  # 直接放开该部分注释即可
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime geometry_msgs
)

add_executable(Traj_gen src/Traj_gen.cpp)


target_link_libraries(Traj_gen
  ${catkin_LIBRARIES}
)

