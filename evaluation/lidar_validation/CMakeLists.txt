cmake_minimum_required(VERSION 3.16)
project(lidar_validation_tool CXX)

# MVSim source tree of the dataset generator that renders the benchmark (the commit of the
# release); its sources are compiled in so that the LiDAR model is the very same code.
set(MVSIM_SRC "" CACHE PATH "MVSim source tree")
if(NOT MVSIM_SRC)
  message(FATAL_ERROR "Set -DMVSIM_SRC=<MVSim source tree>")
endif()
set(GEN ${MVSIM_SRC}/mvsim-dataset-gen)

find_package(mvsim-simulator REQUIRED)
find_package(mvsim-raytracer REQUIRED)
find_package(TBB QUIET)

add_executable(lidar_tool lidar_tool.cpp ${GEN}/LidarSimulator.cpp ${GEN}/TrajectorySource.cpp)
target_include_directories(lidar_tool PRIVATE ${GEN})
target_compile_features(lidar_tool PRIVATE cxx_std_17)
target_link_libraries(lidar_tool mvsim::mvsim-simulator mvsim::mvsim-raytracer pthread)
if(TBB_FOUND)
	target_link_libraries(lidar_tool TBB::tbb)
	target_compile_definitions(lidar_tool PRIVATE MVSIM_HAS_TBB)
endif()
