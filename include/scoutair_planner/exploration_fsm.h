#ifndef _EXPLORATION_FSM_H_
#define _EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
// #include <thread>

#include <scoutair_planner/exploration_manager.h>

namespace scoutair_planner {

// class FastPlannerManager;
// class FastExplorationManager;
// class PlanningVisualization;
// struct FSMParam;
// struct FSMData;

enum EXPL_STATE { INIT, ROTATE, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH };

class ExplorationFSM 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  ExplorationFSM( const ros::NodeHandle& nh, const ros::NodeHandle& nh_private ) 
    : nh_(nh),
      nh_private_(nh_private)
  {}

  ~ExplorationFSM() {}

  void init();
  
private:
  /* planning utils */
  std::shared_ptr<ExplorationManager> exploration_manager_;
  
  // Visulization
  std::shared_ptr<FtrVisulization> fsm_visu_;
  
  // ROS Parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  EXPL_STATE state_;

  bool classic_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher controller_pub_;

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, std::string pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback( const geometry_msgs::PoseStamped::ConstPtr& msg );
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void visualize();

  void droneRotate();

  // FSM data
  bool trigger_, have_odom_, static_state_;
  std::vector<std::string> state_str_;

  Eigen::Vector3f odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaternionf odom_orient_;
  float odom_yaw_;
  Eigen::Vector3f start_pt_, start_vel_, start_acc_;
  Eigen::Vector3f start_yaw_;  // start state

  std::vector<Eigen::Vector3f> start_poss;
  // bspline::Bspline newest_traj_;

  double replan_thresh1_;
  double replan_thresh2_;
  double replan_thresh3_;
  double replan_time_;  // second
};

}  // namespace scoutair_planner

#endif