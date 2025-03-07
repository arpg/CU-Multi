#include <fstream>
#include <iostream>
#include <sstream>
#include <Eigen/Geometry>

#include <cu_multi_vis_toolkit/pose.h>


// Converts a txt pose file to a Pose
bool PoseHandler::getRelativePose(const std::string &posepath, Pose& pose) {
    std::ifstream relativePoseFile(posepath);

    if (!relativePoseFile.is_open()) {
        std::cerr << "Error in getRelativePose: Could not open relative pose file.\n";
        return false;
    }

    std::string poseLine;
    while (std::getline(relativePoseFile, poseLine)) {
        std::istringstream poseStream(poseLine);

        if (!(poseStream >> pose.x >> pose.y >> pose.z >> pose.qx >> pose.qy >> pose.qz >> pose.qw)) {
            std::cerr << "Error in getRelativePose: Could not parse relative pose.\n";
        }
    }

    return true;
}

// Converts a Eigen::Affine3f matrix to a Pose
Pose PoseHandler::affine3fToPose(const Eigen::Affine3f& transform) {
    // Extract translation
    Eigen::Vector3f translation = transform.translation();

    // Extract rotation
    Eigen::Quaternionf rotation(transform.rotation());

    // Construct and return the Pose
    Pose pose;
    pose.x = translation.x();
    pose.y = translation.y();
    pose.z = translation.z();
    pose.qw = rotation.w();
    pose.qx = rotation.x();
    pose.qy = rotation.y();
    pose.qz = rotation.z();

    return pose;
}

// Converts a Pose to an Eigen::Affine3f matrix
Eigen::Affine3f PoseHandler::poseToAffine3f(const Pose& pose) {
    Eigen::Vector3f translation(pose.x, pose.y, pose.z);
    Eigen::Quaternionf rotation(pose.qw, pose.qx, pose.qy, pose.qz);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translate(translation);
    transform.rotate(rotation);

    return transform;
}

// Loads poses from file
bool PoseHandler::loadPoses(const std::string &path, std::map<double, Pose> &poseMap) {
    std::ifstream posesFile(path + "groundtruth_odom.txt");
    std::ifstream timestampsFile(path + "odom_timestamps.txt");

    if (!posesFile.is_open() || !timestampsFile.is_open()) {
        std::cerr << "Error in loadPoses: Could not open pose or timestamp file.\n";
        return false;
    }

    std::string poseLine, timestampLine;
    while (std::getline(posesFile, poseLine) && std::getline(timestampsFile, timestampLine)) {
        Pose pose;
        std::istringstream poseStream(poseLine);
        std::istringstream timestampStream(timestampLine);
        double timestamp;

        if (!(poseStream >> pose.x >> pose.y >> pose.z >> pose.qx >> pose.qy >> pose.qz >> pose.qw) ||
            !(timestampStream >> timestamp)) {
            std::cerr << "Error in loadPoses: Could not parse pose or timestamp.\n";
            return false;
        }

        poseMap[timestamp] = pose;
    }

    return true;
}

// Gets the nearest pose based on timestamp
Pose PoseHandler::getNearestPose(double timestamp, const std::map<double, Pose>& poseMap) {
    auto lowerBound = poseMap.lower_bound(timestamp);

    if (lowerBound == poseMap.begin()) {
        return lowerBound->second;
    } else if (lowerBound == poseMap.end()) {
        return std::prev(lowerBound)->second;
    }

    auto prev = std::prev(lowerBound);
    return (timestamp - prev->first) <= (lowerBound->first - timestamp) ? prev->second : lowerBound->second;
}

// Interpolates a pose between two timestamps
Pose PoseHandler::interpolatePose(double timestamp, const std::map<double, Pose>& poseMap) {
    auto lowerBound = poseMap.lower_bound(timestamp);

    if (lowerBound == poseMap.begin()) {
        return lowerBound->second;
    } else if (lowerBound == poseMap.end()) {
        return std::prev(lowerBound)->second;
    }

    auto upper = lowerBound;
    auto lower = std::prev(lowerBound);

    double t1 = lower->first;
    double t2 = upper->first;
    double alpha = (timestamp - t1) / (t2 - t1);

    const Pose& pose1 = lower->second;
    const Pose& pose2 = upper->second;

    Pose interpolatedPose;
    interpolatedPose.x = pose1.x + alpha * (pose2.x - pose1.x);
    interpolatedPose.y = pose1.y + alpha * (pose2.y - pose1.y);
    interpolatedPose.z = pose1.z + alpha * (pose2.z - pose1.z);

    Eigen::Quaterniond q1(pose1.qw, pose1.qx, pose1.qy, pose1.qz);
    Eigen::Quaterniond q2(pose2.qw, pose2.qx, pose2.qy, pose2.qz);
    Eigen::Quaterniond qInterpolated = q1.slerp(alpha, q2);

    interpolatedPose.qw = qInterpolated.w();
    interpolatedPose.qx = qInterpolated.x();
    interpolatedPose.qy = qInterpolated.y();
    interpolatedPose.qz = qInterpolated.z();

    return interpolatedPose;
}
