/*
 * @Author: Liuyaqi99 liuyaqi1212h@163.com
 * @Date: 2022-06-01 21:27:17
 * @LastEditors: Liuyaqi99 283402121@qq.com
 * @LastEditTime: 2022-06-02 13:48:51
 * @FilePath: /dbscan_pcl/src/main.cc
 */
#ifndef COMMON_POINT_TYPE_H_
#define COMMON_POINT_TYPE_H_

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointTypeCloud;
typedef PointTypeCloud::Ptr PointTypeCloudPtr;
typedef PointTypeCloud::ConstPtr PointTypeCloudConstPtr;

#endif // COMMON_POINT_TYPE_H_