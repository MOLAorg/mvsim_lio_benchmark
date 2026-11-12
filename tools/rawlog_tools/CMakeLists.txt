cmake_minimum_required(VERSION 3.16)
project(rawlog_tools CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(mrpt_maps REQUIRED)
find_package(mrpt_obs REQUIRED)

add_executable(compare-rawlogs compare_rawlogs.cpp)
target_link_libraries(compare-rawlogs mrpt::mrpt_maps mrpt::mrpt_obs)

add_executable(set-imu-pose set_imu_pose.cpp)
target_link_libraries(set-imu-pose mrpt::mrpt_maps mrpt::mrpt_obs)
