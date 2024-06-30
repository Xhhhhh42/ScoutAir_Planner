#include <geometry_msgs/PoseStamped.h>

#include <scoutair_planner/exploration_fsm.h>

namespace scoutair_planner {

using namespace Eigen;
using namespace std;


void ExplorationFSM::init() {
  /*  Fsm param  */
  nh_private_.param("fsm/thresh_replan1", replan_thresh1_, -1.0);
  nh_private_.param("fsm/thresh_replan2", replan_thresh2_, -1.0);
  nh_private_.param("fsm/thresh_replan3", replan_thresh3_, -1.0);
  nh_private_.param("fsm/replan_time", replan_time_, -1.0);

  /* Initialize main modules */
  exploration_manager_.reset(new ExplorationManager(nh_, nh_private_));
  fsm_visu_ = exploration_manager_->manager_visu_;
  
  state_ = EXPL_STATE::INIT;
  have_odom_ = false;
  state_str_ = { "INIT", "ROTATE", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH" };
  trigger_ = false; 

  /* Ros sub, pub and timer */
  exec_timer_ = nh_private_.createTimer(ros::Duration(0.3), &ExplorationFSM::FSMCallback, this);
  frontier_timer_ = nh_private_.createTimer(ros::Duration(0.3), &ExplorationFSM::frontierCallback, this);

  trigger_sub_ = nh_private_.subscribe("/move_base_simple/goal", 1, &ExplorationFSM::triggerCallback, this);
  odom_sub_ = nh_private_.subscribe("odometry", 1, &ExplorationFSM::odometryCallback, this);

  controller_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>( "command/pose", 1, false );
  waypoint_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>( "waypoint", 1, false );

  ready_to_fly_ = false;

  std::cout << "ExplorationFSM inited" << std::endl;
}


void ExplorationFSM::FSMCallback( const ros::TimerEvent& e ) 
{
  ROS_INFO_STREAM_THROTTLE(3.0, "[FSM]: state: " << state_str_[int(state_)]);

  switch (state_) {
    case INIT: {

      // Wait for odometry ready
      if (!have_odom_ ) {
        ROS_WARN_THROTTLE(3.0, "No odom.");
        return;
      }

      if ( odom_pos_(2) < 0.9 ) {
        ROS_WARN_THROTTLE(3.0, "Drone take off.");
        return;
      }

      transitState(ROTATE, "FSM");
      break;
    }

    case ROTATE: {
      ROS_WARN_THROTTLE(2.0, "Drone rotates.");
      static int time = 0;
      if( time < 4 ) {
        droneRotate();
        ros::Duration(4.0).sleep();
        // std::cout<< "drone rotates "<< std::endl; 
        time++;
        return;
      }
      exploration_manager_->frontiermap_->initFrontierMap();
      // Go to wait trigger when first update frontier map
      transitState(WAIT_TRIGGER, "FSM");
      ready_to_fly_ = true;
      break;
    }

    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      ROS_WARN_THROTTLE(3.0, "Wait for trigger.");
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
      // } else if (res == FAIL) {
      //   // Still in PLAN_TRAJ state, keep replanning
      //   ROS_WARN("plan fail");
      }
      break;
    }

    case PUB_TRAJ: {
      // thread vis_thread(&ExplorationFSM::visualize, this);
      // vis_thread.detach();
      transitState(EXEC_TRAJ, "FSM");
      break;
    }

    case EXEC_TRAJ: {
      if((odom_pos_ - next_pos_).norm() < 0.4f)
      {
        ros::Duration(0.3).sleep();
        transitState(PLAN_TRAJ, "FSM");
      }
        
      if((last_odom_pos_ - odom_pos_).norm() < 0.01f){
        static int time = 0;
        if( time++ > 5 ) {
          transitState(PLAN_TRAJ, "FSM");
        }
        break;
      }

      last_odom_pos_ = odom_pos_;
        
      // LocalTrajData* info = &planner_manager_->local_data_;
      // double t_cur = (ros::Time::now() - info->start_time_).toSec();

      // Replan if traj is almost fully executed
      // double time_to_end = info->duration_ - t_cur;
      // if (time_to_end < replan_thresh1_) {
      //   transitState(PLAN_TRAJ, "FSM");
      //   ROS_WARN("Replan: traj fully executed=================================");
      //   return;
      // }
      // // Replan if next frontier to be visited is covered
      // if (t_cur > replan_thresh2_ && exploration_manager_->frontiermap_->isFrontierCovered()) {
      //   transitState(PLAN_TRAJ, "FSM");
      //   ROS_WARN("Replan: cluster covered=====================================");
      //   return;
      // }
      // // Replan after some time
      // if (t_cur > replan_thresh3_ && !classic_) {
      //   transitState(PLAN_TRAJ, "FSM");
      //   ROS_WARN("Replan: periodic call=======================================");
      // }
      break;
    }
  }
}


int ExplorationFSM::callExplorationPlanner() 
{
  // ros::Time time_r = ros::Time::now() + ros::Duration(replan_time_);
  int res = exploration_manager_->planExploreMotion( start_pt_, start_vel_, start_acc_, start_yaw_, next_pos_, next_yaw_ );
  // pub next point
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";

  pose.pose.position.x = next_pos_.x(); 
  pose.pose.position.y = next_pos_.y();
  pose.pose.position.z = next_pos_.z();

  Eigen::AngleAxisf yawAngle(next_yaw_, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf q_yaw(yawAngle);
  pose.pose.orientation.x = q_yaw.x();
  pose.pose.orientation.y = q_yaw.y();
  pose.pose.orientation.z = q_yaw.z();
  pose.pose.orientation.w = q_yaw.w();

  std::cout << "Next view: " << next_pos_.transpose() << ", " << next_yaw_ << std::endl;

  waypoint_pub_.publish(pose);
  waypoint_pub_.publish(pose);

  // if (res == SUCCEED) {
  //   auto info = &planner_manager_->local_data_;
  //   info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

  //   bspline::Bspline bspline;
  //   bspline.order = planner_manager_->pp_.bspline_degree_;
  //   bspline.start_time = info->start_time_;
  //   bspline.traj_id = info->traj_id_;
  //   Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
  //   for (int i = 0; i < pos_pts.rows(); ++i) {
  //     geometry_msgs::Point pt;
  //     pt.x = pos_pts(i, 0);
  //     pt.y = pos_pts(i, 1);
  //     pt.z = pos_pts(i, 2);
  //     bspline.pos_pts.push_back(pt);
  //   }
  //   Eigen::VectorXd knots = info->position_traj_.getKnot();
  //   for (int i = 0; i < knots.rows(); ++i) {
  //     bspline.knots.push_back(knots(i));
  //   }
  //   Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
  //   for (int i = 0; i < yaw_pts.rows(); ++i) {
  //     double yaw = yaw_pts(i, 0);
  //     bspline.yaw_pts.push_back(yaw);
  //   }
  //   bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
  //   newest_traj_ = bspline;
  // }
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

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = 1.0; 
  pose.pose.position.y = odom_pos_.y();
  pose.pose.position.z = 1.2;
  // Set the new orientation
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;
  pose.pose.orientation.w = 1;

  controller_pub_.publish(pose);
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
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = 1.0; 
  pose.pose.position.y = odom_pos_.y();
  pose.pose.position.z = 1.4;

  if( !init_move ) {
    // Set the new orientation
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    controller_pub_.publish(pose);
    init_move = true;
    return;
  }

  // Rotate the pose by 120 degrees around the Z-axis
  static Eigen::Quaternionf q_last = odom_orient_;
  Eigen::AngleAxisf rotation(M_PI * 120.0 / 180.0, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf q_rot(rotation);
  Eigen::Quaternionf q_new = q_rot * q_last;
  q_last = q_new;
  q_new.normalize();

  // Set the new orientation
  pose.pose.orientation.x = q_new.x();
  pose.pose.orientation.y = q_new.y();
  pose.pose.orientation.z = q_new.z();
  pose.pose.orientation.w = q_new.w();

  controller_pub_.publish(pose);
}
}  // namespace scoutair_planner
