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

#include<scoutair_planner_msgs/StartPlanner.h>
#include<scoutair_planner_msgs/PausePlanner.h>

#include <scoutair_planner/exploration_manager.h>

namespace scoutair_planner {

enum EXPL_STATE { INIT, ROTATE, WAIT_TRIGGER, PLAN_TRAJ, OVERPASS, EXEC_TRAJ, FINISH, PAUSE };

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
  /* helper functions */
  int callExplorationPlanner();
  void transitState( EXPL_STATE new_state, std::string pos_call );

  /* ROS functions */
  void FSMCallback( const ros::TimerEvent& e );
  void frontierCallback( const ros::TimerEvent& e );
  void triggerCallback( const geometry_msgs::PoseStamped::ConstPtr& msg );
  void odometryCallback( const nav_msgs::OdometryConstPtr& msg );
  void visualize();

  void droneRotate();
  void droneRotate( float &yaw );
  void controllerPub( Eigen::Vector3f &pos, float &yaw );
  void controllerPub( Eigen::Vector3f &pos, Eigen::Quaternionf &q );

  bool startPlannerCallback( scoutair_planner_msgs::StartPlanner::Request &req,
                             scoutair_planner_msgs::StartPlanner::Response &res );
  bool pausePlannerCallback( scoutair_planner_msgs::PausePlanner::Request &req,
                             scoutair_planner_msgs::PausePlanner::Response &res );

  /* planning utils */
  std::shared_ptr<ExplorationManager> exploration_manager_;
  
  /* Visulization */ 
  std::shared_ptr<FtrVisulization> fsm_visu_;

  /* FSM data */
  EXPL_STATE state_;
  std::vector<std::string> state_str_;
  bool trigger_, have_odom_, static_state_;
  bool ready_to_fly_;
  bool paused_;
  
  Eigen::Vector3f odom_pos_, last_odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaternionf odom_orient_;
  float odom_yaw_;
  Eigen::Vector3f start_pt_, start_vel_, start_acc_;
  Eigen::Vector3f start_yaw_;  // start state

  std::vector<Eigen::Vector3f> next_pos_vec_;
  std::vector<float> next_yaw_vec_;
  Eigen::Vector3f next_pos_;
  float next_yaw_;

  /* ROS utils */
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher controller_pub_;
  ros::Publisher waypoint_pub_;
  ros::ServiceServer start_service_;
  ros::ServiceServer pause_service_;  
};

}  // namespace scoutair_planner

#endif