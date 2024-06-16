#include <scoutair_planner/perception_utils.h>

#include <pcl/filters/voxel_grid.h>

namespace scoutair_planner {

using namespace std;

PerceptionUtils::PerceptionUtils(ros::NodeHandle& nh) {
  nh.param<float>("perception_utils/top_angle", top_angle_, -1.0);
  nh.param<float>("perception_utils/left_angle", left_angle_, -1.0);
  nh.param<float>("perception_utils/right_angle", right_angle_, -1.0);
  nh.param<float>("perception_utils/max_dist", max_dist_, -1.0);
  nh.param<float>("perception_utils/vis_dist", vis_dist_, -1.0);

  n_top_ << 0.0, sin(M_PI_2 - top_angle_), cos(M_PI_2 - top_angle_);
  n_bottom_ << 0.0, -sin(M_PI_2 - top_angle_), cos(M_PI_2 - top_angle_);

  n_left_ << sin(M_PI_2 - left_angle_), 0.0, cos(M_PI_2 - left_angle_);
  n_right_ << -sin(M_PI_2 - right_angle_), 0.0, cos(M_PI_2 - right_angle_);
  T_cb_ << 0, -1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  T_bc_ = T_cb_.inverse();

  // FOV vertices in body frame, for FOV visualization
  float hor = vis_dist_ * tan(left_angle_);
  float vert = vis_dist_ * tan(top_angle_);
  Eigen::Vector3f origin(0, 0, 0);
  Eigen::Vector3f left_up(vis_dist_, hor, vert);
  Eigen::Vector3f left_down(vis_dist_, hor, -vert);
  Eigen::Vector3f right_up(vis_dist_, -hor, vert);
  Eigen::Vector3f right_down(vis_dist_, -hor, -vert);

  cam_vertices1_.push_back(origin);
  cam_vertices2_.push_back(left_up);
  cam_vertices1_.push_back(origin);
  cam_vertices2_.push_back(left_down);
  cam_vertices1_.push_back(origin);
  cam_vertices2_.push_back(right_up);
  cam_vertices1_.push_back(origin);
  cam_vertices2_.push_back(right_down);

  cam_vertices1_.push_back(left_up);
  cam_vertices2_.push_back(right_up);
  cam_vertices1_.push_back(right_up);
  cam_vertices2_.push_back(right_down);
  cam_vertices1_.push_back(right_down);
  cam_vertices2_.push_back(left_down);
  cam_vertices1_.push_back(left_down);
  cam_vertices2_.push_back(left_up);
}

void PerceptionUtils::setPose(const Eigen::Vector3f& pos, const float& yaw) {
  pos_ = pos;
  yaw_ = yaw;

  // Transform the normals of camera FOV
  Eigen::Matrix3f R_wb;
  R_wb << cos(yaw_), -sin(yaw_), 0.0, sin(yaw_), cos(yaw_), 0.0, 0.0, 0.0, 1.0;
  Eigen::Vector3f pc = pos_;

  Eigen::Matrix4f T_wb = Eigen::Matrix4f::Identity();
  T_wb.block<3, 3>(0, 0) = R_wb;
  T_wb.block<3, 1>(0, 3) = pc;
  Eigen::Matrix4f T_wc = T_wb * T_bc_;
  Eigen::Matrix3f R_wc = T_wc.block<3, 3>(0, 0);
  // Eigen::Vector3f t_wc = T_wc.block<3, 1>(0, 3);
  normals_ = { n_top_, n_bottom_, n_left_, n_right_ };
  for (auto& n : normals_)
    n = R_wc * n;
}

void PerceptionUtils::getFOV(std::vector<Eigen::Vector3f>& list1, std::vector<Eigen::Vector3f>& list2) {
  list1.clear();
  list2.clear();

  // Get info for visualizing FOV at (pos, yaw)
  Eigen::Matrix3f Rwb;
  Rwb << cos(yaw_), -sin(yaw_), 0, sin(yaw_), cos(yaw_), 0, 0, 0, 1;
  for (int i = 0; i < cam_vertices1_.size(); ++i) {
    auto p1 = Rwb * cam_vertices1_[i] + pos_;
    auto p2 = Rwb * cam_vertices2_[i] + pos_;
    list1.push_back(p1);
    list2.push_back(p2);
  }
}

bool PerceptionUtils::insideFOV(const Eigen::Vector3f& point) {
  Eigen::Vector3f dir = point - pos_;
  if (dir.norm() > max_dist_) return false;

  dir.normalize();
  for (auto n : normals_) {
    if (dir.dot(n) < 0.0) return false;
  }
  return true;
}

void PerceptionUtils::getFOVBoundingBox(Eigen::Vector3f& bmin, Eigen::Vector3f& bmax) {
  float left = yaw_ + left_angle_;
  float right = yaw_ - right_angle_;
  Eigen::Vector3f left_pt = pos_ + max_dist_ * Eigen::Vector3f(cos(left), sin(left), 0);
  Eigen::Vector3f right_pt = pos_ + max_dist_ * Eigen::Vector3f(cos(right), sin(right), 0);
  vector<Eigen::Vector3f> points = { left_pt, right_pt };
  if (left > 0 && right < 0)
    points.push_back(pos_ + max_dist_ * Eigen::Vector3f(1, 0, 0));
  else if (left > M_PI_2 && right < M_PI_2)
    points.push_back(pos_ + max_dist_ * Eigen::Vector3f(0, 1, 0));
  else if (left > -M_PI_2 && right < -M_PI_2)
    points.push_back(pos_ + max_dist_ * Eigen::Vector3f(0, -1, 0));
  else if ((left > M_PI && right < M_PI) || (left > -M_PI && right < -M_PI))
    points.push_back(pos_ + max_dist_ * Eigen::Vector3f(-1, 0, 0));

  bmax = bmin = pos_;
  for (auto p : points) {
    bmax = bmax.array().max(p.array());
    bmin = bmin.array().min(p.array());
  }
}

}  // namespace scoutair_planner