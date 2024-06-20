#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <voxblox/core/common.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace scoutair_planner {
struct ExplorationParam;
struct ExplorationData;

enum EXPL_RESULT { NO_FRONTIER, FAIL, SUCCEED };

class ExplorationMapManager {
public:
  ExplorationMapManager( const ros::NodeHandle& nh, const ros::NodeHandle& nh_private ) {}
  ~ExplorationMapManager() {}

  void initialize(ros::NodeHandle& nh);

  int planExploreMotion(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc,
                        const Vector3d& yaw);

  // Benchmark method, classic frontier and rapid frontier
  int classicFrontier(const Vector3d& pos, const double& yaw);
  int rapidFrontier(const Vector3d& pos, const Vector3d& vel, const double& yaw, bool& classic);

  void getUpdatedBoxIndex( voxblox::GlobalIndex& index_min2, voxblox::GlobalIndex& index_max2 ) const;

  void getUpdatedBolocks( voxblox::BlockIndexList &updated_blocks ) const;

  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
//   shared_ptr<FastPlannerManager> planner_manager_;
  // shared_ptr<FrontierFinder> frontier_finder_;
  // unique_ptr<ViewFinder> view_finder_;

private:
//   shared_ptr<EDTEnvironment> edt_environment_;
//   shared_ptr<SDFMap> sdf_map_;

  // Find optimal tour for coarse viewpoints of all frontiers
  void findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
                      vector<int>& indices);

  // Refine local tour for next few frontiers, using more diverse viewpoints
  void refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
                       const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws,
                       vector<Vector3d>& refined_pts, vector<double>& refined_yaws);

  void shortenPath(vector<Vector3d>& path);

public:
  typedef shared_ptr<ExplorationMapManager> Ptr;
};

}  // namespace scoutair_planner

#endif