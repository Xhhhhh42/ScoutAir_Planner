#ifndef _PERCEPTION_UTILS_H_
#define _PERCEPTION_UTILS_H_

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <iostream>
#include <memory>
#include <vector>

using std::shared_ptr;
using std::unique_ptr;

namespace scoutair_planner {
class PerceptionUtils {
public:
  PerceptionUtils( ros::NodeHandle& nh );
  ~PerceptionUtils() {
  }
  // Set position and yaw
  void setPose(const Eigen::Vector3f& pos, const float& yaw);

  // Get info of current pose
  void getFOV(std::vector<Eigen::Vector3f>& list1, std::vector<Eigen::Vector3f>& list2);
  bool insideFOV(const Eigen::Vector3f& point);
  void getFOVBoundingBox(Eigen::Vector3f& bmin, Eigen::Vector3f& bmax);

private:
  // Data
  // Current camera pos and yaw
  Eigen::Vector3f pos_;
  float yaw_;
  // Camera plane's normals in world frame
  std::vector<Eigen::Vector3f> normals_;

  /* Params */
  // Sensing range of camera
  float left_angle_, right_angle_, top_angle_, max_dist_, vis_dist_;
  // Normal vectors of camera FOV planes in camera frame
  Eigen::Vector3f n_top_, n_bottom_, n_left_, n_right_;
  // Transform between camera and body
  Eigen::Matrix4f T_cb_, T_bc_;
  // FOV vertices in body frame
  std::vector<Eigen::Vector3f> cam_vertices1_, cam_vertices2_;
};

}  // namespace scoutair_planner
#endif