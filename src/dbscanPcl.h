/*
 * @Author: Liuyaqi99 liuyaqi1212h@163.com
 * @Date: 2022-06-01 21:13:47
 * @LastEditTime: 2022-06-03 20:03:51
 * @FilePath: /dbscan_pcl/src/dbscanPcl.h
 * @Description: DBSCAN algorithm for point cloud cluster
 */
#ifndef DBSCAN_PCL_H
#define DBSCAN_PCL_H

#define PCL_NO_PRECOMPILE
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#define UN_VISITED 0
#define VISITED 1

template <typename PointT> class DBSCANPointCloud {
public:
  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  typedef typename pcl::KdTreeFLANN<PointT>::Ptr KdTreePtr;
  DBSCANPointCloud() { input_cloud_ = boost::make_shared<PointCloud>(); }

  virtual void setInputCloud(PointCloudPtr cloud) { input_cloud_ = cloud; }

  void setSearchMethod(KdTreePtr tree) { search_method_ = tree; }

  void extract(std::vector<pcl::PointIndices> &cluster_indices,
               pcl::PointIndices &noise_indices) {
    int cluster_count = 0;
    std::vector<std::vector<int>> nnList;
    std::vector<int> pointIdxSearch;
    std::vector<float> pointSquareDistance;
    std::vector<int> coreListsIdx;
    std::vector<bool> is_noise(input_cloud_->points.size(), false);
    std::vector<int> types(input_cloud_->points.size(), UN_VISITED);

    // initial core list
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for (size_t i = 0; i < input_cloud_->size(); i++) {
      const PointT &pt = input_cloud_->points[i];
      const double dis_xy = sqrt(pt.x * pt.x + pt.y * pt.y);
      const double local_eps = std::max(
          dis_xy * 0.16 * M_PI / 180 * (1 / tan(4 * M_PI / 180)), eps_);
      if (radiusSearch(i, local_eps, pointIdxSearch, pointSquareDistance) >=
          minPts_)
        coreListsIdx.push_back(i);
      nnList.push_back(pointIdxSearch);
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::ratio<1, 1000>> time_span =
        std::chrono::duration_cast<
            std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
    // std::cout << "1 Extract by Time:\t" << time_span.count() << " ms."
    // << std::endl;

    t1 = std::chrono::steady_clock::now();
    // for each core point, find its neighbours
    while (!coreListsIdx.empty()) {
      const int &core = coreListsIdx[0];
      std::vector<int> QIdx{core};
      types[core] = VISITED;
      is_noise[core] = false;
      pcl::PointIndices clusterIdx;
      clusterIdx.indices.push_back(core);
      while (!QIdx.empty()) {
        int q = QIdx[0];
        QIdx.erase(QIdx.begin());
        if (nnList[q].size() >= minPts_) {
          for (size_t i = 0; i < nnList[q].size(); i++) {
            const int &idx = nnList[q][i];
            if (types[idx] == UN_VISITED) {
              QIdx.push_back(idx);
              types[idx] = VISITED;
              is_noise[idx] = false;
              clusterIdx.indices.push_back(idx);
            }
          }
        }
      }
      cluster_indices.push_back(clusterIdx);
      cluster_count++;
      std::sort(clusterIdx.indices.begin(), clusterIdx.indices.end());
      std::sort(coreListsIdx.begin(), coreListsIdx.end());
      std::vector<int> result;
      std::set_difference(coreListsIdx.begin(), coreListsIdx.end(),
                          clusterIdx.indices.begin(), clusterIdx.indices.end(),
                          std::back_inserter(result));
      std::swap(coreListsIdx, result);
    }
    t2 = std::chrono::steady_clock::now();
    time_span = std::chrono::duration_cast<
        std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
    // std::cout << "2 Extract by Time:\t" << time_span.count() << " ms."
    // << std::endl;

    t1 = std::chrono::steady_clock::now();
    for (size_t i = 0; i < is_noise.size(); i++) {
      if (is_noise[i])
        noise_indices.indices.push_back(i);
    }
    t2 = std::chrono::steady_clock::now();
    time_span = std::chrono::duration_cast<
        std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
    // std::cout << "3 Extract by Time:\t" << time_span.count() << " ms."
    // << std::endl;
  }

  void setClusterTolerance(double tolerance) { eps_ = tolerance; }

  void setCorePointMinPts(int core_point_min_pts) {
    minPts_ = core_point_min_pts;
  }

protected:
  PointCloudPtr input_cloud_; //输入点云 input point cloud
  KdTreePtr search_method_;   //搜索方法（kd-tree）search method
  double eps_ = 0.0;          //邻域半径 epsilon
  int minPts_ = 1;            //最少点数 minpts

  virtual int radiusSearch(int index, double radius,
                           std::vector<int> &k_indices,
                           std::vector<float> &k_sqr_distances) const {
    return this->search_method_->radiusSearch(index, radius, k_indices,
                                              k_sqr_distances);
  }
}; // class DBSCANPointCloud

#endif // DBSCAN_H