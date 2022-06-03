/*
 * @Author: Liuyaqi99 liuyaqi1212h@163.com
 * @Date: 2022-06-03 13:22:46
 * @LastEditTime: 2022-06-03 23:43:21
 * @FilePath: /dbscan_pcl/src/testOnKitti.cc
 */
#include <chrono>
#include <iostream>
#include <memory>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "dbscanPcl.h"
#include "showPcl.h"

// #define USE_VOXEL
// #define SHOW_RAW_POINT_CLOUD

void readLabel(const std::string &frame_id,
               std::vector<std::vector<float>> &bbox_list) {
  bbox_list.clear();
  std::ifstream read;
  std::stringstream filename;
  filename << "/home/lyq/Dataset/IVFC2021/debug/xjy_data/kitti/"
              "kitti_label_align_only_car/"
           << frame_id << ".txt";
  read.open(filename.str(), std::ios::in);
  std::string buf;
  std::vector<float> a;
  bool only_one_flag = false;
  while (getline(read, buf)) {
    std::vector<float> line;
    std::istringstream iss(buf);
    float temp;
    while (iss >> temp) {
      line.push_back(temp);
    }
    if (line.size() == 1) {
      a.insert(a.end(), line.begin(), line.end());
      line.clear();
      only_one_flag = true;
      continue;
    } else {
      if (line.size() != 16) {
        std::cout << "wrong size in box file!" << std::endl;
      }
      bbox_list.push_back(line);
      line.clear();
    }
  }
  if (only_one_flag) {
    if (a.size() != 16) {
      std::cout << "wrong size in box file!" << std::endl;
    }
    bbox_list.push_back(a);
  }
}

void test(const int &minpts, const double &epis, const double &voxel_leaf) {
  size_t all_obstacle_gt = 0;
  size_t all_obstacle_det = 0;
  double ave_rate = 0;
  double ave_time = 0;
  int correct_count = 0;
  std::string path =
      "../../../Dataset/IVFC2021/debug/xjy_data/kitti/kitti_obstacle";
  DIR *pDir;
  struct dirent *ptr;
  int count = 0;
  if (!(pDir = opendir(path.c_str()))) {
    std::cout << "Folder doesn't Exist!" << std::endl;
  }
  while ((ptr = readdir(pDir)) != 0) {
    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
      std::string filename = path + "/" + ptr->d_name;
      char frameid_c[10];
      strncpy(frameid_c, ptr->d_name, 6);
      std::string frameid = frameid_c;
      std::vector<std::vector<float>> list;
      readLabel(frameid, list); // 读取label

      std::stringstream file;
      file << path << "/" << frameid << ".pcd";
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrc(
          new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(file.str(), *cloudSrc) == -1) {
        std::cout << "Error:can not open the file: " << file.str() << std::endl;
      }
      // 聚类 segmetation
      std::shared_ptr<DBSCANPointCloud<pcl::PointXYZ>> cluster =
          std::make_shared<DBSCANPointCloud<pcl::PointXYZ>>();
      std::vector<pcl::PointIndices> clusterIdxList;
      pcl::PointIndices noiseIdx;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
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
      cloud = cloudVoxeled;
#endif
      kdtree->setInputCloud(cloud);
      cluster->setInputCloud(cloud);
      cluster->setSearchMethod(kdtree);
      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();
      cluster->extract(clusterIdxList, noiseIdx);
      std::chrono::steady_clock::time_point t2 =
          std::chrono::steady_clock::now();
      std::chrono::duration<double, std::ratio<1, 1000>> time_span =
          std::chrono::duration_cast<
              std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
      double rate = std::abs(static_cast<double>(list.size()) -
                             static_cast<double>(clusterIdxList.size())) /
                    std::max(list.size(), clusterIdxList.size());

      std::ofstream write;
      std::stringstream file_name;
      file_name << "../data/result/record_" << epis << "_" << minpts << ".txt";
      write.open(file_name.str(), std::ios::app);
      write << frameid << "\t" << clusterIdxList.size() << "\t" << list.size()
            << "\t" << time_span.count() << "\t" << rate << std::endl;
      all_obstacle_gt += list.size();
      all_obstacle_det += clusterIdxList.size();
      ave_rate += rate;
      ave_time += time_span.count();
      if (clusterIdxList.size() == list.size())
        correct_count++;
    }
  }
  closedir(pDir);
  ave_time /= 7474;
  ave_rate /= 7474;
  std::ofstream write_result;
  std::string result_name = "../data/result/result.txt";
  write_result.open(result_name.c_str(), std::ios::app);
  write_result << minpts << "\t" << epis << "\t" << all_obstacle_gt << "\t"
               << all_obstacle_det << "\t" << correct_count << "\t" << ave_time
               << "\t" << ave_rate << std::endl;
  std::cout << "-------------- final --------------" << std::endl;
  std::cout << "Min Points:\t" << minpts << "\nEpsilon:\t" << epis << std::endl;
  std::cout << "all_obstacle_gt:\t" << all_obstacle_gt
            << "\nall_obstacle_det:\t" << all_obstacle_det
            << "\ncorrect_count:\t" << correct_count << "\nave_time:\t"
            << ave_time << "\nave_rate:\t" << ave_rate << std::endl;
}

int main(int argc, char **argv) {
  int minpts = 4;
  double epis = 1.5;
  double voxel_leaf = 0.05;
  test(minpts, epis, voxel_leaf);
  return 0;
}