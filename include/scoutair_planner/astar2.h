#ifndef _ASTAR2_H
#define _ASTAR2_H

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>

#include <scoutair_planner/common.h>
#include <scoutair_planner/frontier_map.h>

// #include "plan_env/edt_environment.h"

// #include <path_searching/matrix_hash.h>

namespace scoutair_planner {

class Node {
public:
  Eigen::Vector3i index;
  Eigen::Vector3f position;
  float g_score, f_score;
  Node* parent;

  /* -------------------- */
  Node() {
    parent = NULL;
  }
  ~Node(){};
};
typedef Node* NodePtr;

class NodeComparator0 {
public:
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};


class Astar 
{
public:

  Astar();
  ~Astar();
  enum { REACH_END = 1, NO_PATH = 2 };

//   void init(ros::NodeHandle& nh, const EDTEnvironment::Ptr& env);
  void init(ros::NodeHandle& nh, std::shared_ptr<FrontierMap> &frontiermap );
  void reset();
  int search(const Eigen::Vector3f& start_pt, const Eigen::Vector3f& end_pt);
  void setResolution(const float& res);
  static float pathLength(const std::vector<Eigen::Vector3f>& path);

  std::vector<Eigen::Vector3f> getPath();
  std::vector<Eigen::Vector3f> getVisited();
  float getEarlyTerminateCost();

  float lambda_heu_;
  float max_search_time_;

private:
  void backtrack(const NodePtr& end_node, const Eigen::Vector3f& end);
  void posToIndex(const Eigen::Vector3f& pt, Eigen::Vector3i& idx);
  float getDiagHeu(const Eigen::Vector3f& x1, const Eigen::Vector3f& x2);
  float getManhHeu(const Eigen::Vector3f& x1, const Eigen::Vector3f& x2);
  float getEuclHeu(const Eigen::Vector3f& x1, const Eigen::Vector3f& x2);

  // main data structure
  std::vector<NodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
  std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> open_set_map_;
  std::unordered_map<Eigen::Vector3i, int, matrix_hash<Eigen::Vector3i>> close_set_map_;
  std::vector<Eigen::Vector3f> path_nodes_;
  float early_terminate_cost_;

//   EDTEnvironment::Ptr edt_env_;
  std::shared_ptr<FrontierMap> frontiermap_;

  // parameter
  float margin_;
  int allocate_num_;
  float tie_breaker_;
  float resolution_, inv_resolution_;
  Eigen::Vector3f map_size_3d_, origin_;
};

}  // namespace scoutair_planner

#endif // _ASTAR2_H