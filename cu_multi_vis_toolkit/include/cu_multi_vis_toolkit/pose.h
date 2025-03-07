#ifndef CU_MULTI_VIS_TOOLKIT_POSE_H_
#define CU_MULTI_VIS_TOOLKIT_POSE_H_

#pragma once

#include <map>
#include <string>
#include <Eigen/Dense>

struct Pose {
    double x, y, z;
    double qx, qy, qz, qw;
};

class PoseHandler {
public:
    static bool loadPoses(const std::string &path, std::map<double, Pose> &poseMap);
    static Pose getNearestPose(double timestamp, const std::map<double, Pose>& poseMap);
    static Pose interpolatePose(double timestamp, const std::map<double, Pose>& poseMap);
    static bool getRelativePose(const std::string &posepath, Pose& pose);
    static Pose affine3fToPose(const Eigen::Affine3f& transform);
    static Eigen::Affine3f poseToAffine3f(const Pose& pose);
};

#endif // CU_MULTI_VIS_TOOLKIT_POSE_H_

