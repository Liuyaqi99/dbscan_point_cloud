#ifndef SHOW_PCL_H_
#define SHOW_PCL_H_

#define PCL_NO_PRECOMPILE
#include <ctime>
#include <pcl/visualization/pcl_visualizer.h>

template <typename PointT> class ShowPointCloud {
public:
  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

  void CopyPointCloud(const PointCloud &cloud_in,
                      const std::vector<int> &indices, PointCloud &cloud_out) {
    cloud_out.resize(indices.size());
    for (size_t i = 0; i < indices.size(); i++) {
      cloud_out.points[i] = cloud_in.points[indices[i]];
    }
  }

  void visualizePcd(const PointCloudPtr &pcd) {
    pcl::visualization::PCLVisualizer viewer("Raw");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src(pcd, 0, 0, 0);
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud(pcd, src, "source");

    while (!viewer.wasStopped()) {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }

  void visualizePcd(const PointCloudPtr &pcd,
                    const std::vector<pcl::PointIndices> &cluster_indices,
                    const pcl::PointIndices &noise_indices) {
    pcl::visualization::PCLVisualizer viewer("Result");
    viewer.setBackgroundColor(255, 255, 255);
    srand((int)(time(NULL)));
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      PointCloudPtr cluster(new PointCloud);
      CopyPointCloud(*pcd, cluster_indices[i].indices, *cluster);
      auto r = (rand() % 255);
      auto g = (rand() % 255);
      auto b = (rand() % 255);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> src(cluster, r,
                                                                   g, b);
      std::stringstream ss;
      ss << "cloud" << i;
      viewer.addPointCloud(cluster, src, ss.str());
    }
    PointCloudPtr noise(new PointCloud);
    CopyPointCloud(*pcd, noise_indices.indices, *noise);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src(noise, 0, 0,
                                                                 0);
    viewer.addPointCloud(noise, src, "noise");

    while (!viewer.wasStopped()) {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }
};

#endif // SHOW_PCL_H_