cmake_minimum_required(VERSION 3.0.2)
project(DBSCAN_point_cloud)

option(BUILD_DBSCAN_CUDA "Build GPU-powered DBSCAN" ON)
set(BUILD_DBSCAN_CUDA ON)
if(BUILD_DBSCAN_CUDA)
  message("build fast_gicp with CUDA!!!")
  if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  else()
    add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
    set(CMAKE_C_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
    set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
  endif()

  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
  set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")
  add_definitions(-DUSE_VGICP_CUDA)
  cuda_add_library(dbscan_cuda SHARED
    src/cuda/findNeighbours.cu
  )
  target_include_directories(dbscan_cuda PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(dbscan_cuda
    ${catkin_LIBRARIES}
  )
  cuda_add_cublas_to_target(dbscan_cuda)
endif()

add_compile_options(-std=c++17)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")

find_package(PCL 1.8 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
src/main.cc)

add_executable(${PROJECT_NAME}_kitti_test_node 
src/testOnKitti.cc)

target_link_libraries(${PROJECT_NAME}_node
   ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_kitti_test_node 
   ${PCL_LIBRARIES}
)

if(BUILD_DBSCAN_CUDA)
  # add vgicp_cuda to libfast_gicp
  target_sources(${PROJECT_NAME}_node PRIVATE
  src/cuda/findNeighbours.cu
  )
  target_link_libraries(${PROJECT_NAME}_node
  dbscan_cuda
  )
  add_dependencies(${PROJECT_NAME}_node dbscan_cuda)
endif()
