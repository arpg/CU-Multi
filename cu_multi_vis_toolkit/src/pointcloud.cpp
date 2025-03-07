#include <fstream>
#include <iostream>
#include <vector>
#include <pcl/filters/voxel_grid.h> // For the VoxelGrid filter

#include <cu_multi_vis_toolkit/pointcloud.h>
#include <cu_multi_vis_toolkit/pose.h>

/*
 *  Loads lidar timestamps into a map (associative array).
 *  Key: lidar timestamp
 *  Value: Corresponding index of lidar scan
*/
bool PointCloud::loadTimestamps(const std::string &filePath,  std::map<double, int> &timestampIndexMap) const {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file " << filePath << std::endl;
        return false;
    }

    std::string line;
    int lineIndex = 0;
    while (std::getline(file, line)) {
        std::istringstream lineStream(line);
        double timestamp;
        if (!(lineStream >> timestamp)) {
            std::cerr << "Error: Could not parse timestamp at line " << lineIndex << std::endl;
            return false;
        }

        timestampIndexMap[timestamp] = lineIndex; // Map timestamp to its line index
        ++lineIndex;
    }

    file.close();
    return true;
}

void PointCloud::downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledCloud, float leafSize) const {
    pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
    voxelGrid.filter(*downsampledCloud);
}

void PointCloud::transform(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Pose& pose) {
    // Create a transformation matrix from position and quaternion
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() << pose.x, pose.y, pose.z;
    Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);

    transform.linear() = q.toRotationMatrix();  // Set rotation matrix

    // Apply the tf to each point in PC
    for (auto& point : cloud->points) {
        Eigen::Vector3d pointVec(point.x, point.y, point.z);
        pointVec = transform * pointVec; // Apply transformation
        point.x = pointVec.x();
        point.y = pointVec.y();
        point.z = pointVec.z();
    }

    // Optionally update cloud to ensure the transformed points are stored
    cloud->is_dense = true;
    cloud->width = cloud->points.size();
    cloud->height = 1;
}

bool PointCloud::setCurrentCloud(const std::string &filename, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) const {
    std::ifstream inputFile(filename, std::ios::binary);
    if (!inputFile) {
        std::cerr << "Error in loadPointCloud: Could not open file " << filename << std::endl;
        return false;
    }

    inputFile.seekg(0, std::ios::end);
    std::streamsize fileSize = inputFile.tellg();
    inputFile.seekg(0, std::ios::beg);

    if (fileSize % (4 * sizeof(float)) != 0) {
        std::cerr << "Error in loadPointCloud: Invalid file size in " << filename << std::endl;
        return false;
    }

    size_t numPoints = fileSize / (4 * sizeof(float));
    std::vector<float> pointCloudData(numPoints * 4);
    if (!inputFile.read(reinterpret_cast<char*>(pointCloudData.data()), fileSize)) {
        std::cerr << "Error in loadPointCloud: Could not read the entire file " << filename << std::endl;
        return false;
    }
    inputFile.close();

    // Clear the existing point cloud and populate it with new data
    cloud->clear();
    for (size_t i = 0; i < pointCloudData.size(); i += 4) {
        pcl::PointXYZI point;
        point.x = pointCloudData[i];
        point.y = pointCloudData[i + 1];
        point.z = pointCloudData[i + 2];
        point.intensity = pointCloudData[i + 3];
        cloud->points.push_back(point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return true;
}

bool PointCloud::setCurrentTimestamp(int fileIndex, const std::map<double, int> &cloudIndexTimestampMap, double &cloudTimestamp) const {
    // Find the corresponding timestamp for the fileIndex
    auto it = std::find_if(cloudIndexTimestampMap.begin(), cloudIndexTimestampMap.end(),
                           [fileIndex](const std::pair<double, int> &entry) {
                               return entry.second == fileIndex;  // File index is 1-based, map is 0-based
                           });

    if (it == cloudIndexTimestampMap.end()) {
        std::cerr << "Error: Could not find a timestamp for file index " << fileIndex << std::endl;
        return false;
    }

    // Set the cloud timestamp
    cloudTimestamp = it->first;

    return true;
}