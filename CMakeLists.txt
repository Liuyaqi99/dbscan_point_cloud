cmake_minimum_required(VERSION 3.0.2)
project(Ceres_for_Horizon)

add_compile_options(-std=c++14)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")

find_package(PCL 1.8 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_3D_node src/main3D.cpp)
add_executable(${PROJECT_NAME}_2D_node src/main2D.cpp)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

target_link_libraries(${PROJECT_NAME}_3D_node
   ${PCL_LIBRARIES}
   ${CERES_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_2D_node
   ${PCL_LIBRARIES}
   ${CERES_LIBRARIES}
)

