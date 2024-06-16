#ifndef _EXPLORATION_FSM_H_
#define _EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

// using Eigen::Vector3d;
// using std::vector;
// using std::shared_ptr;
// using std::unique_ptr;
// using std::string;

namespace scoutair_planner {

class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH };

class ExplorationFSM {
private:
  /* planning utils */
  std::shared_ptr<FastPlannerManager> planner_manager_;
  std::shared_ptr<FastExplorationManager> expl_manager_;
  std::shared_ptr<PlanningVisualization> visualization_;

  std::shared_ptr<FSMParam> fp_;
  std::shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, std::string pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void visualize();
  void clearVisMarker();

public:
  ExplorationFSM(/* args */) {
  }
  ~ExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace scoutair_planner

#endif