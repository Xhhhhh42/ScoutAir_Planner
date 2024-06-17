#ifndef _GRAPH_NODE_H_
#define _GRAPH_NODE_H_
#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include <boost/dynamic_bitset.hpp>

#include <scoutair_planner/raycast.h>
// #include <scoutair_planner/voxblox_map.h>
#include <scoutair_planner/common.h>
#include <scoutair_planner/astar2.h>

namespace scoutair_planner {

// Basic noded type containing only general artributes required by graph search
class BaseNode {
public:
  typedef std::shared_ptr<BaseNode> Ptr;
  BaseNode() {
    g_value_ = 1000000;
    closed_ = false;
  }
  ~BaseNode() {
  }

  int id_;
  bool closed_;
  float g_value_;
};

// Node type for viewpoint refinement
class ViewNode : public BaseNode {
public:
  typedef std::shared_ptr<ViewNode> Ptr;

  ViewNode(const Eigen::Vector3f& pos, const float& yaw);

  ViewNode() {}
  ~ViewNode() {}

  static void set( const std::shared_ptr<boost::dynamic_bitset<>>& freebit, const Box_boundaries &boundaries );

  static bool check( const Eigen::Vector3i &idx );

  float costTo(const ViewNode::Ptr& node);

  static float computeCost( const Eigen::Vector3f& pos1, const Eigen::Vector3f& pos2, const float& yaw1, const float& yaw2,
                            const Eigen::Vector3f& v1, const float& yd1, std::vector<Eigen::Vector3f>& path);

  // Coarse to fine path searching
  static float searchPath(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, std::vector<Eigen::Vector3f>& path);

  // Data
  std::vector<ViewNode::Ptr> neighbors_;
  ViewNode::Ptr parent_;
  Eigen::Vector3f pos_, vel_;
  float yaw_, yaw_dot_;

  // Parameters shared among nodes
  static float vm_, am_, yd_, ydd_, w_dir_;
  static std::shared_ptr<Astar> astar_;
  static std::shared_ptr<RayCaster> caster_;
  // static std::shared_ptr<VoxbloxMap> map_;
  static std::shared_ptr<boost::dynamic_bitset<>> free_bit_;

  static Box_boundaries boundaries_;
};

} // namespace scoutair_planner

#endif  // _GRAPH_NODE_H_