/*
 * @Author: Liuyaqi99 liuyaqi1212h@163.com
 * @Date: 2022-06-01 21:27:17
 * @LastEditTime: 2022-06-02 22:05:13
 * @FilePath: /dbscan_pcl/src/main.cc
 * @Description: main
 */
#include <chrono>
#include <iostream>
#include <memory>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "dbscanPcl.h"
#include "showPcl.h"

#define USE_VOXEL
// #define SHOW_RAW_POINT_CLOUD

int main(int argc, char **argv) {
  std::string file = "../data/kitti/000008.pcd";
  if (argc > 1)
    file = (argv[1]);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrc(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file.c_str(), *cloudSrc) == -1) {
    std::cout << "Error:can not open the file: " << file.c_str() << std::endl;
    return (-1);
  }
  int minpts = 20;
  double epis = 0.5;
  double voxel_leaf = 0.1;
  std::cout << "-------------- info --------------" << std::endl;
  std::cout << "Min Points:\t" << minpts << "\nEpsilon:\t" << epis
            << "\nFile Name:\t" << file.c_str() << "\nRaw Points Size:\t"
            << cloudSrc->size() << std::endl;

  std::cout << "-------------- test --------------" << std::endl;
  std::shared_ptr<ShowPointCloud<pcl::PointXYZ>> visualizer =
      std::make_shared<ShowPointCloud<pcl::PointXYZ>>();
// 原始点云可视化 Raw point cloud visualization
#ifdef SHOW_RAW_POINT_CLOUD
  visualizer->visualizePcd(cloudSrc);
#endif

  // 聚类 segmetation
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::shared_ptr<DBSCANPointCloud<pcl::PointXYZ>> cluster =
      std::make_shared<DBSCANPointCloud<pcl::PointXYZ>>();
  std::vector<pcl::PointIndices> clusterIdxList;
  pcl::PointIndices noiseIdx;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(
      new pcl::KdTreeFLANN<pcl::PointXYZ>);
  cluster->setClusterTolerance(epis);
  cluster->setCorePointMinPts(minpts);
  cloud = cloudSrc;
#ifdef USE_VOXEL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxeled(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloudSrc);
  sor.setLeafSize(voxel_leaf, voxel_leaf, voxel_leaf);
  sor.filter(*cloudVoxeled);
  std::cout << "Use Voxel, voxel leaf size:\t" << voxel_leaf
            << "\npoint size(after voxelized):\t" << cloudVoxeled->size()
            << std::endl;
  cloud = cloudVoxeled;
#endif
  kdtree->setInputCloud(cloud);
  cluster->setInputCloud(cloud);
  cluster->setSearchMethod(kdtree);
  cluster->extract(clusterIdxList, noiseIdx);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double, std::ratio<1, 1000>> time_span =
      std::chrono::duration_cast<
          std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
  std::cout << "cluster num:\t" << clusterIdxList.size()
            << "\nnoise point num:\t" << noiseIdx.indices.size() << std::endl;
  std::cout << "Extract by Time:\t" << time_span.count() << " ms." << std::endl;

  // 结果可视化 result visualization
  visualizer->visualizePcd(cloud, clusterIdxList, noiseIdx);
  return 0;
}