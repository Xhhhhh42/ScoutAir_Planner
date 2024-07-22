#include <scoutair_planner/graph_node.h>

// #include <path_searching/astar2.h>

namespace scoutair_planner {

// Static data
float ViewNode::vm_;
float ViewNode::am_;
float ViewNode::yd_;
float ViewNode::ydd_;
float ViewNode::w_dir_;
std::shared_ptr<Astar> ViewNode::astar_;
std::shared_ptr<RayCaster> ViewNode::caster_;
std::shared_ptr<boost::dynamic_bitset<>> ViewNode::free_bit_;
Box_boundaries ViewNode::boundaries_;

using namespace std;
using namespace Eigen;

// Graph node for viewpoints planning
ViewNode::ViewNode(const Eigen::Vector3f& pos, const float& yaw) 
{
  pos_ = pos;
  yaw_ = yaw;
  parent_ = nullptr;
  vel_.setZero();  // vel is zero by default, should be set explicitly
}


float ViewNode::costTo( const ViewNode::Ptr& node ) 
{
  vector<Eigen::Vector3f> path;
  float cost = ViewNode::computeCost(pos_, node->pos_, yaw_, node->yaw_, vel_, yaw_dot_, path);
  return cost;
}


void ViewNode::set( const std::shared_ptr<boost::dynamic_bitset<>>& freebit, const Box_boundaries &boundaries )
{
  free_bit_ = freebit;
  boundaries_ = boundaries;
}


bool ViewNode::check( const Eigen::Vector3i &idx ) 
{
  int linear_index = (idx[0] - boundaries_.box_min_[0]) * boundaries_.map_voxel_num_(1) * boundaries_.map_voxel_num_(2) + 
                      (idx[1] - boundaries_.box_min_[1]) * boundaries_.map_voxel_num_(2) + (idx[2] - boundaries_.box_min_[2]);
  if( !free_bit_->test(linear_index) ) {
    return false;
  }
  return true;
}


/// @brief 尝试使用直线连接这两个点，如果失败，则使用逐步降低分辨率的方法进行A*算法搜索
/// @param pos1 
/// @param pos2 
/// @param path 
/// @return 
float ViewNode::searchPath(const Eigen::Vector3f& pos1, const Eigen::Vector3f& pos2, vector<Eigen::Vector3f>& path) 
{
  // Try connect two points with straight line
  bool safe = true;
  Vector3i idx;
  if( caster_ == nullptr ) std::cout << "nullptr here in searchPath!" << std::endl;
  caster_->input(pos1, pos2);
  while (caster_->nextId(idx)) {
    if( !check(idx) ) {
      safe = false;
      break;
    }
  }
  if (safe) {
    path = { pos1, pos2 };
    return (pos1 - pos2).norm();
  }

  // Search a path using decreasing resolution
  vector<float> res = { 0.4 };
  for (int k = 0; k < res.size(); ++k) {
    astar_->reset();
    astar_->setResolution(res[k]);
    if (astar_->search(pos1, pos2) == Astar::REACH_END) {
      // std::cout << "Search a path using astar!" << std::endl;
      path = astar_->getPath();
      return astar_->pathLength(path);
    }
  }

  // Use Astar early termination cost as an estimate
  path = { pos1, pos2 };
  return 1000;
}


/// @brief 计算两个位置和方向之间的代价。这些位置和方向包括当前的位置和方向以及目标的位置和方向，同时还考虑了速度和方向变化的影响
/// @param pos1 
/// @param pos2 
/// @param yaw1 
/// @param yaw2 
/// @param v1 
/// @param yd1 
/// @param path 
/// @return 
float ViewNode::computeCost( const Eigen::Vector3f& pos1, const Eigen::Vector3f& pos2, const float& yaw1, const float& yaw2,
                             const Eigen::Vector3f& v1, const float& yd1, std::vector<Eigen::Vector3f>& path ) 
{
  // Cost of position change
  float pos_cost = ViewNode::searchPath(pos1, pos2, path) / vm_;

  // Consider velocity change
  // 如果 v1 很大,说明机器人正在运动。我们需要考虑当前速度方向和目标方向的偏差，以计算到达目标所需的额外代价。
  // 如果速度方向和目标方向完全一致，diff 为 0，说明没有额外代价;
  // 如果偏差很大，diff 也会很大，导致较高的额外代价。
  if (v1.norm() > 1e-3) {
    Eigen::Vector3f dir = (pos2 - pos1).normalized();
    Eigen::Vector3f vdir = v1.normalized();
    float diff = acos(vdir.dot(dir));
    pos_cost += w_dir_ * diff;
    // float vc = v1.dot(dir);
    // pos_cost += w_dir_ * pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
    // if (vc < 0)
    //   pos_cost += w_dir_ * 2 * fabs(vc) / am_;
  }

  // Cost of yaw change
  float diff = fabs(yaw2 - yaw1);
  diff = min(diff, static_cast<float>( 2 * M_PI - diff ));
  float yaw_cost = diff / yd_;
  return max(pos_cost, yaw_cost);
}

} // namespace scoutair_planner 
