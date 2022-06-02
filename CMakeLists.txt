cmake_minimum_required(VERSION 3.0.2)
project(DBSCAN_point_cloud)

add_compile_options(-std=c++17)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")

find_package(PCL 1.8 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
src/main.cc)

target_link_libraries(${PROJECT_NAME}_node
   ${PCL_LIBRARIES}
)
