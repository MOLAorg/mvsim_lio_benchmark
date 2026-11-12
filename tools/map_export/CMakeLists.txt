cmake_minimum_required(VERSION 3.16)
project(bench_map_export CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(mrpt_maps REQUIRED)
find_package(mrpt_obs REQUIRED)
find_package(mrpt_io REQUIRED)
find_package(mrpt_poses REQUIRED)

add_executable(bench-map-export bench_map_export.cpp)
target_link_libraries(bench-map-export mrpt::mrpt_maps mrpt::mrpt_obs mrpt::mrpt_io mrpt::mrpt_poses)
