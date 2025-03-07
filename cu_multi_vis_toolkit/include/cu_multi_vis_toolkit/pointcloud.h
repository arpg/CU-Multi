#ifndef CU_MULTI_VIS_TOOLKIT_POINTCLOUD_H_
#define CU_MULTI_VIS_TOOLKIT_POINTCLOUD_H_

#pragma once

#include <string>
#include <map>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cu_multi_vis_toolkit/pose.h>  // For the Pose struct

class PointCloud {
public:
    // Loads cloud timestamps and maps them to line indices for later retrieval
    bool loadTimestamps(const std::string &filePath,  std::map<double, int> &timestampIndexMap) const;
    
    // Utils
    void downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledCloud, float leafSize = 0.1f) const;
    void transform(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Pose& pose);

    // Setters
    bool setCurrentTimestamp(int fileIndex, const std::map<double, int> &cloudIndexTimestampMap, double &cloudTimestamp) const;
    bool setCurrentCloud(const std::string &filename, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) const;

    // // Getters
    // double getCurrentTimestamp();
    // double getCurrentCloud();

// private:
//     double currentTimestamp;
//     pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloud;
//     std::map<double, int> timestampMap;
};

#endif // CU_MULTI_VIS_TOOLKIT_POINTCLOUD_H_
