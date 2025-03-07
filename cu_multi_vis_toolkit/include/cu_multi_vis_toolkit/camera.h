#ifndef CU_MULTI_VIS_TOOLKIT_CAMERA_H_
#define CU_MULTI_VIS_TOOLKIT_CAMERA_H_

#pragma once

#include <string>

class Camera {
// public:
//     // Loads cloud timestamps and maps them to line indices for later retrieval
//     bool loadTimestamps(const std::string &filePath,  std::map<double, int> &timestampIndexMap) const;
    
//     // Utils
//     void downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float leafSize = 0.1f) const;
//     void transform(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Pose& pose);

//     // Setters
//     bool setRun(int runIndex);
//     bool setCurrentCloud(const std::string &filename, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) const;

//     // Getters
//     double getCurrentTimestamp();
//     double getCurrentCloud();

// private:
//     double currentTimestamp;
//     pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloud;
//     std::map<double, int> timestampMap;
};

#endif // CU_MULTI_VIS_TOOLKIT_CAMERA_H_