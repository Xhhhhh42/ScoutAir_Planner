#include <geometry_msgs/PoseStamped.h>

#include <scoutair_planner/exploration_fsm.h>

namespace scoutair_planner {

using namespace Eigen;
using namespace std;


void ExplorationFSM::init() {
  /* Initialize main modules */
  exploration_manager_.reset(new ExplorationManager(nh_, nh_private_));
  fsm_visu_ = exploration_manager_->manager_visu_;
  
  state_ = EXPL_STATE::INIT;
  state_str_ = { "INIT", "ROTATE", "WAIT_TRIGGER", "PLAN_TRAJ", "OVERPASS", "EXEC_TRAJ", "FINISH", "PAUSE" };
  have_odom_ = false;
  trigger_ = false; 
  ready_to_fly_ = false;
  paused_ = false;
  init_start_ = false;

  /* Ros sub, pub and timer */
  start_service_ = nh_private_.advertiseService("start_planner", &ExplorationFSM::startPlannerCallback, this);
  pause_service_ = nh_private_.advertiseService("pause_planner", &ExplorationFSM::pausePlannerCallback, this);

  exec_timer_ = nh_private_.createTimer(ros::Duration(0.3), &ExplorationFSM::FSMCallback, this);
  frontier_timer_ = nh_private_.createTimer(ros::Duration(0.3), &ExplorationFSM::frontierCallback, this);

  trigger_sub_ = nh_private_.subscribe("/move_base_simple/goal", 1, &ExplorationFSM::triggerCallback, this);
  odom_sub_ = nh_private_.subscribe("odometry", 1, &ExplorationFSM::odometryCallback, this);

  controller_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>( "command/pose", 1, false );
  waypoint_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>( "waypoint", 1, false );  

  std::cout << "ExplorationFSM inited" << std::endl;
}


void ExplorationFSM::FSMCallback( const ros::TimerEvent& e ) 
{
  ROS_INFO_STREAM_THROTTLE(5.0, "[FSM]: state: " << state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!have_odom_ ) {
        ROS_WARN_THROTTLE(5.0, "No odom.");
        return;
      }

      if ( odom_pos_(2) < 0.9 ) {
        ROS_WARN_THROTTLE(3.0, "Drone take off.");
        return;
      }
      
      if( !init_start_ ) {
        ROS_WARN_THROTTLE(3.0, "Wait for Init command.");
        break;
      }

      transitState(ROTATE, "FSM");
      break;
    }

    case ROTATE: {
      ROS_WARN_THROTTLE(2.0, "Drone rotates.");
      static int time = 0;
      if( time < 3 ) {
        droneRotate();
        ros::Duration(2.4).sleep();
        // std::cout<< "drone rotates "<< std::endl; 
        time++;
        return;
      }
      ros::Duration(0.3).sleep();
      exploration_manager_->frontiermap_->initFrontierMap();
      // Go to wait trigger when first update frontier map
      transitState(WAIT_TRIGGER, "FSM");
      ready_to_fly_ = true;
      break;
    }

    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      ROS_WARN_THROTTLE(5.0, "Wait for trigger.");
      break;
    }

    case FINISH: {
      ROS_INFO_THROTTLE(3.0, "Finish exploration.");
      break;
    }

    case PLAN_TRAJ: {
      start_pt_ = odom_pos_;
      start_yaw_(0) = odom_yaw_;
      start_yaw_(1) = start_yaw_(2) = 0.0;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        transitState(EXEC_TRAJ, "FSM");
      } else if (res == NO_FRONTIER) {
        transitState(FINISH, "FSM");
      }
      break;
    }

    case OVERPASS: {
      Eigen::Vector3f tmp_pos;
      tmp_pos.x() = odom_pos_.x();
      tmp_pos.y() = odom_pos_.y();
      tmp_pos.z() = 1.7;
      float tmp_yaw = (next_yaw_ + odom_yaw_)/2;
      controllerPub(tmp_pos, tmp_yaw);

      ROS_INFO("Drone meets obstacle.");
      ros::Duration(1.0).sleep();
      
      transitState(PLAN_TRAJ, "FSM");
      break;
    }

    case EXEC_TRAJ: {
      if((odom_pos_ - next_pos_).norm() > 3.0f) {
        exploration_manager_->frontiermap_->setOdom( odom_pos_, odom_yaw_ );
        exploration_manager_->frontiermap_->visualizeFrontiers();
      }
      if((odom_pos_ - next_pos_).norm() < 0.2f )
      {
        if(std::abs(odom_yaw_ - next_yaw_) < 15.0 * M_PI / 180.0) {
          ros::Duration(0.3).sleep();
        } else {
          ROS_INFO("Drone did't reached the target Yaw angle, republic pose to flight controller.");
          controllerPub(odom_pos_, next_yaw_);
          ros::Duration(1.0).sleep();
        }

        last_odom_pos_ = odom_pos_;
        transitState(PLAN_TRAJ, "FSM");
        break;
      }
        
      if((last_odom_pos_ - odom_pos_).norm() < 0.05f){
        static int time = 0;
        if( time++ > 3 ) {
          // static int replan = 0;
          ROS_INFO("Drone remains stationary for 1.0s, replanning.");
          // if( replan++ >= 2 ) {
          //   transitState(OVERPASS, "FSM");
          //   replan = 0;
          // }
          transitState(PLAN_TRAJ, "FSM");
          time = 0;
        }
        break;
      }

      last_odom_pos_ = odom_pos_;
      break;
    }

    case PAUSE: {
      ROS_INFO_THROTTLE(3.0, "Pause ScoutAir Planner FSM, waiting for restart.");
      if( !paused_ ) {
        transitState(PLAN_TRAJ, "FSM");
      }
    }
  }
}


int ExplorationFSM::callExplorationPlanner() 
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";

  next_pos_vec_.clear();
  next_yaw_vec_.clear();
  int res = exploration_manager_->planExploreMotion( start_pt_, start_vel_, start_acc_, start_yaw_, next_pos_vec_, next_yaw_vec_ );
  next_pos_ = next_pos_vec_[0];
  next_yaw_ = next_yaw_vec_[0];
  if( next_pos_vec_.size() <= 0 ) {
    ROS_ERROR("Find no next Viewpoint");
  } else if( next_pos_vec_.size() == 1 ) {
    ROS_INFO("Last Viewpoint.");
  } else {
    if((odom_pos_ - next_pos_).norm() < 0.5f) {
      droneRotate(next_yaw_);
      ros::Duration(0.5).sleep();
      next_pos_ = next_pos_vec_[1];
      next_yaw_ = next_yaw_vec_[1];
      
      if((odom_pos_ - next_pos_).norm() < 0.5f) {
        ROS_INFO("Next Viewpoint is nearby, skip and regenerate.");
        start_pt_ = next_pos_;
        start_vel_.setZero();
        start_acc_.setZero();
        start_yaw_(0) = next_yaw_;
        start_yaw_(1) = start_yaw_(2) = 0.0;
        res = exploration_manager_->planExploreMotion( start_pt_, start_vel_, start_acc_, start_yaw_, next_pos_vec_, next_yaw_vec_ );
        next_pos_ = next_pos_vec_[0];
        next_yaw_ = next_yaw_vec_[0];
      }
    }
  }

  // pub next point
  pose.pose.position.x = next_pos_.x(); 
  pose.pose.position.y = next_pos_.y();
  pose.pose.position.z = next_pos_.z();

  Eigen::AngleAxisf yawAngle(next_yaw_, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf q_yaw(yawAngle);
  pose.pose.orientation.x = q_yaw.x();
  pose.pose.orientation.y = q_yaw.y();
  pose.pose.orientation.z = q_yaw.z();
  pose.pose.orientation.w = q_yaw.w();
  waypoint_pub_.publish(pose);
  std::cout << "Next view: " << next_pos_.transpose() << ", " << next_yaw_ << std::endl;

  return res;
}


void ExplorationFSM::frontierCallback(const ros::TimerEvent& e) 
{
  if( !ready_to_fly_ || !have_odom_ ) 
    return;

  static int delay = 0;
  if( delay++ < 5 ) 
    return;

  exploration_manager_->frontiermap_->setOdom( odom_pos_, odom_yaw_ );
  exploration_manager_->frontiermap_->visualizeFrontiers();
}


void ExplorationFSM::triggerCallback( const geometry_msgs::PoseStamped::ConstPtr& msg ) 
{
  if (msg->pose.position.z < -0.1) return;
  if (state_ != WAIT_TRIGGER) return;
  trigger_ = true;

  Eigen::Vector3f tmp_pos;
  tmp_pos.x() = 1.0;
  tmp_pos.y() = odom_pos_.y();
  tmp_pos.z() = 1.2;

  // Set the new orientation
  Eigen::Quaternionf q;
  q.x() = 0;
  q.y() = 0;
  q.z() = 0;
  q.w() = 1;
  controllerPub(tmp_pos, q);
  ros::Duration(2.0).sleep();

  cout << "Triggered! Start Exploration." << endl;
  transitState(PLAN_TRAJ, "triggerCallback");
}


void ExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) 
{
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3f rot_x = odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  odom_yaw_ = atan2(rot_x(1), rot_x(0)); // radian

  // std::cout<<"current_yaw: " << odom_yaw_ << endl;
  have_odom_ = true;
}


void ExplorationFSM::transitState( EXPL_STATE new_state, string pos_call ) 
{
  int pre_s = int(state_);
  state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str_[pre_s] + " to " + state_str_[int(new_state)] << endl;
}


void ExplorationFSM::droneRotate()
{
  static bool init_move = false;
  Eigen::Vector3f tmp_pos;
  tmp_pos.x() = 1.0;
  tmp_pos.y() = odom_pos_.y();
  tmp_pos.z() = 1.4;

  if( !init_move ) {
    // Set the new orientation
    Eigen::Quaternionf q;
    q.x() = 0;
    q.y() = 0;
    q.z() = 0;
    q.w() = 1;
    controllerPub(tmp_pos, q);

    init_move = true;
    return;
  }

  // Rotate the pose by 120 degrees around the Z-axis
  static Eigen::Quaternionf q_last = odom_orient_;
  Eigen::AngleAxisf rotation(M_PI * 140.0 / 180.0, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf q_rot(rotation);
  Eigen::Quaternionf q_new = q_rot * q_last;
  q_last = q_new;
  q_new.normalize();
  controllerPub(tmp_pos, q_new);
}


void ExplorationFSM::droneRotate(float &yaw)
{
  controllerPub(odom_pos_, yaw);
}


void ExplorationFSM::controllerPub( Eigen::Vector3f &pos, float &yaw )
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = pos.x(); 
  pose.pose.position.y = pos.y();
  pose.pose.position.z = pos.z();

  Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf q_yaw(yawAngle);
  pose.pose.orientation.x = q_yaw.x();
  pose.pose.orientation.y = q_yaw.y();
  pose.pose.orientation.z = q_yaw.z();
  pose.pose.orientation.w = q_yaw.w();

  controller_pub_.publish(pose);
}


void ExplorationFSM::controllerPub( Eigen::Vector3f &pos, Eigen::Quaternionf &q )
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = pos.x(); 
  pose.pose.position.y = pos.y();
  pose.pose.position.z = pos.z();

  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  controller_pub_.publish(pose);
}


bool ExplorationFSM::startPlannerCallback( scoutair_planner_msgs::StartPlanner::Request &req,
                                           scoutair_planner_msgs::StartPlanner::Response &res )
{
  if( !init_start_ ) {
    ROS_INFO("Starting the planner.");
    init_start_ = true;
    res.success = true;
    return true;
  }
  
  ROS_INFO("Restarting the planner.");
  paused_ = false;
  res.success = true; 
  return true;
}


bool ExplorationFSM::pausePlannerCallback( scoutair_planner_msgs::PausePlanner::Request &req,
                                           scoutair_planner_msgs::PausePlanner::Response &res )
{
  ROS_INFO("Pausing the planner.");
  transitState(PAUSE, "FSM");
  paused_ = true;
  res.success = true; 
  return true;
}

}  // namespace scoutair_planner
