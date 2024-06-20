
// #include <plan_manage/planner_manager.h>
// #include <traj_utils/planning_visualization.h>

// #include <exploration_manager/fast_exploration_fsm.h>
// #include <exploration_manager/expl_data.h>

#include <scoutair_planner/exploration_fsm.h>


namespace scoutair_planner {

using namespace Eigen;
using namespace std;


void ExplorationFSM::init() {
  // fp_.reset(new FSMParam);
  // fd_.reset(new FSMData);

  /*  Fsm param  */
  nh_.param("fsm/thresh_replan1", replan_thresh1_, -1.0);
  nh_.param("fsm/thresh_replan2", replan_thresh2_, -1.0);
  nh_.param("fsm/thresh_replan3", replan_thresh3_, -1.0);
  nh_.param("fsm/replan_time", replan_time_, -1.0);

  /* Initialize main modules */
  exploration_manager_.reset(new ExplorationManager(nh_, nh_private_));
  exploration_manager_->initialize();
  // visualization_.reset(new PlanningVisualization(nh_));

  // planner_manager_ = exploration_manager_->planner_manager_;
  state_ = EXPL_STATE::INIT;
  have_odom_ = false;
  state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH" };
  static_state_ = true;
  trigger_ = false;

  /* Ros sub, pub and timer */
  exec_timer_ = nh_.createTimer(ros::Duration(0.01), &ExplorationFSM::FSMCallback, this);
  // safety_timer_ = nh_.createTimer(ros::Duration(0.05), &ExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh_.createTimer(ros::Duration(0.5), &ExplorationFSM::frontierCallback, this);

  trigger_sub_ = nh_.subscribe("/waypoint_generator/waypoints", 1, &ExplorationFSM::triggerCallback, this);
  odom_sub_ = nh_.subscribe("/odom_world", 1, &ExplorationFSM::odometryCallback, this);

  replan_pub_ = nh_.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh_.advertise<std_msgs::Empty>("/planning/new", 10);
  // bspline_pub_ = nh_.advertise<bspline::Bspline>("/planning/bspline", 10);
}


void ExplorationFSM::FSMCallback( const ros::TimerEvent& e ) 
{
  ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom.");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case FINISH: {
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case PLAN_TRAJ: {
      if (static_state_) {
        // Plan from static state (hover)
        start_pt_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();

        start_yaw_(0) = odom_yaw_;
        start_yaw_(1) = start_yaw_(2) = 0.0;
      } else {
        // Replan from non-static state, starting from 'replan_time' seconds later
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + replan_time_;

        start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }

      // Inform traj_server the replanning
      replan_pub_.publish(std_msgs::Empty());
      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        transitState(PUB_TRAJ, "FSM");
      } else if (res == NO_FRONTIER) {
        transitState(FINISH, "FSM");
        static_state_ = true;
        clearVisMarker();
      } else if (res == FAIL) {
        // Still in PLAN_TRAJ state, keep replanning
        ROS_WARN("plan fail");
        static_state_ = true;
      }
      break;
    }

    case PUB_TRAJ: {
      double dt = (ros::Time::now() - newest_traj_.start_time).toSec();
      if (dt > 0) {
        bspline_pub_.publish(newest_traj_);
        static_state_ = false;
        transitState(EXEC_TRAJ, "FSM");

        thread vis_thread(&ExplorationFSM::visualize, this);
        vis_thread.detach();
      }
      break;
    }

    case EXEC_TRAJ: {
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();

      // Replan if traj is almost fully executed
      double time_to_end = info->duration_ - t_cur;
      if (time_to_end < replan_thresh1_) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: traj fully executed=================================");
        return;
      }
      // Replan if next frontier to be visited is covered
      if (t_cur > replan_thresh2_ && exploration_manager_->frontiermap_->isFrontierCovered()) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: cluster covered=====================================");
        return;
      }
      // Replan after some time
      if (t_cur > replan_thresh3_ && !classic_) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: periodic call=======================================");
      }
      break;
    }
  }
}


int ExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(replan_time_);

  int res = exploration_manager_->planExploreMotion(start_pt_, start_vel_, start_acc_,
                                             start_yaw_);
  classic_ = false;

  // int res = exploration_manager_->classicFrontier(start_pt_, start_yaw_[0]);
  // classic_ = true;

  // int res = exploration_manager_->rapidFrontier(start_pt_, start_vel_, start_yaw_[0],
  // classic_);

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;
    info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    newest_traj_ = bspline;
  }
  return res;
}


// void ExplorationFSM::visualize() {
//   auto info = &planner_manager_->local_data_;
//   auto plan_data = &planner_manager_->plan_data_;
//   auto ed_ptr = exploration_manager_->ed_;


//   // Draw frontier
//   static int last_ftr_num = 0;
//   for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
//     visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
//                               visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
//                               "frontier", i, 4);
//     // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first, ed_ptr->frontier_boxes_[i].second,
//     //                         Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
//   }
//   for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
//     visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
//     // visualization_->drawBox(Vector3f(0, 0, 0), Vector3f(0, 0, 0), Vector4d(1, 0, 0, 0.3),
//     // "frontier_boxes", i, 4);
//   }
//   last_ftr_num = ed_ptr->frontiers_.size();
  
//   visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
//                               Vector4d(1, 1, 0, 1));
// }


// void ExplorationFSM::clearVisMarker() {

// }


void ExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  static int delay = 0;
  if (++delay < 5) return;

  if (state_ == WAIT_TRIGGER || state_ == FINISH) {
    auto ft = exploration_manager_->frontiermap_;
    // auto ed = exploration_manager_->ed_;
    ft->searchFrontiers();
    ft->computeFrontiersToVisit();
    ft->updateFrontierCostMatrix();

    // ft->getFrontiers(frontiers_);
    // ft->getFrontierBoxes(ed->frontier_boxes_);

    // Draw frontier and bounding box
    // for (int i = 0; i < ed->frontiers_.size(); ++i) {
    //   visualization_->drawCubes(ed->frontiers_[i], 0.1,
    //                             visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
    //                             "frontier", i, 4);
    // }
    // for (int i = ed->frontiers_.size(); i < 50; ++i) {
    //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // }
  }
}


void ExplorationFSM::triggerCallback( const nav_msgs::PathConstPtr& msg ) 
{
  if (msg->poses[0].pose.position.z < -0.1) return;
  if (state_ != WAIT_TRIGGER) return;
  trigger_ = true;
  cout << "Triggered!" << endl;
  transitState(PLAN_TRAJ, "triggerCallback");
}


// void ExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
//   if (state_ == EXPL_STATE::EXEC_TRAJ) {
//     // Check safety and trigger replan if necessary
//     double dist;
//     bool safe = planner_manager_->checkTrajCollision(dist);
//     if (!safe) {
//       ROS_WARN("Replan: collision detected==================================");
//       transitState(PLAN_TRAJ, "safetyCallback");
//     }
//   }
// }


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
  odom_yaw_ = atan2(rot_x(1), rot_x(0));

  have_odom_ = true;
}


void ExplorationFSM::transitState( EXPL_STATE new_state, string pos_call ) 
{
  int pre_s = int(state_);
  state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str_[pre_s] + " to " + state_str_[int(new_state)]
       << endl;
}
}  // namespace scoutair_planner
