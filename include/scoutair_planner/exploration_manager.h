#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>

#include <scoutair_planner/frontier_map.h>
#include <scoutair_planner/common.h>

using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace scoutair_planner {
// class EDTEnvironment;
// class SDFMap;
// class FastPlannerManager;
// class FrontierFinder;
// struct ExplorationParam;
// struct ExplorationData;

enum EXPL_RESULT { NO_FRONTIER, FAIL, SUCCEED };

class ExplorationManager 
{
public:

  ExplorationManager( const ros::NodeHandle& nh, const ros::NodeHandle& nh_private );
  
  ~ExplorationManager();

  void initialize();

  int planExploreMotion(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, const Eigen::Vector3f& acc,
                        const Eigen::Vector3f& yaw);

  std::shared_ptr<FrontierMap> frontiermap_;

private:
  void frontierCallback( const ros::TimerEvent& e );
  
  // Find optimal tour for coarse viewpoints of all frontiers
  void findGlobalTour(const Eigen::Vector3f& cur_pos, const Eigen::Vector3f& cur_vel, const Eigen::Vector3f cur_yaw,
                      std::vector<int>& indices);

  // Refine local tour for next few frontiers, using more diverse viewpoints
  void refineLocalTour(const Eigen::Vector3f& cur_pos, const Eigen::Vector3f& cur_vel, const Eigen::Vector3f& cur_yaw,
                       const std::vector<std::vector<Eigen::Vector3f>>& n_points, const std::vector<std::vector<float>>& n_yaws,
                       std::vector<Eigen::Vector3f>& refined_pts, std::vector<float>& refined_yaws);

  void shortenPath(std::vector<Eigen::Vector3f>& path);

  // ROS Parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Timer frontier_timer_;

  std::shared_ptr<ExplorationParam> ep_;
  std::shared_ptr<ExplorationData> ed_;

  std::vector<Eigen::Vector3f> global_tour_;

public:
  typedef shared_ptr<ExplorationManager> Ptr;

  // Visulization
  std::shared_ptr<FtrVisulization> manager_visu_;
};

}  // namespace scoutair_planner

#endif