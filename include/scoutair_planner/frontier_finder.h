#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <utility>

#include <scoutair_planner/raycast.h>
#include <scoutair_planner/perception_utils.h>
#include <scoutair_planner/exploration_map_manager.h>
#include <voxblox/core/common.h>
#include <voxblox_ros/esdf_server.h>

// using Eigen::Vector3d;
// using std::shared_ptr;
// using std::unique_ptr;
// using std::vector;
// using std::list;
// using std::pair;

namespace scoutair_planner {

// // Viewpoint to cover a frontier cluster
// struct Viewpoint {
//   // Position and heading
//   Eigen::Vector3d pos_;
//   double yaw_;
//   // Fraction of the cluster that can be covered
//   // double fraction_;
//   int visib_num_;
// };

// // A frontier cluster, the viewpoints to cover it
// struct Frontier {
//   std::vector<voxblox::GlobalIndex> idx_cells;
//   voxblox::GlobalIndex idx_box_min_, idx_box_max_;
//   // Complete voxels belonging to the cluster
//   std::vector<Eigen::Vector3d> cells_;
//   // down-sampled voxels filtered by voxel grid filter
//   std::vector<Eigen::Vector3d> filtered_cells_;
//   // Average position of all voxels
//   Eigen::Vector3d average_;
//   // Idx of cluster
//   int id_;
//   // Viewpoints that can cover the cluster
//   std::vector<Viewpoint> viewpoints_;
//   // Bounding box of cluster, center & 1/2 side length
//   Eigen::Vector3d box_min_, box_max_;
//   // Path and cost from this cluster to other clusters
//   std::list<std::vector<Eigen::Vector3d>> paths_;
//   std::list<double> costs_;
// };

class FrontierFinder {
public:
  enum VoxelStatus { kUnknown = 0, kOccupied, kFree };

  FrontierFinder( ros::NodeHandle& nh, ros::NodeHandle& nh_private );
  ~FrontierFinder();

  void searchFrontiers();
  void computeFrontiersToVisit();

  void getFrontiers(std::vector<std::vector<Eigen::Vector3d>>& clusters);
  void getDormantFrontiers(std::vector<std::vector<Eigen::Vector3d>>& clusters);
  void getFrontierBoxes(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes);
  // Get viewpoint with highest coverage for each frontier
  void getTopViewpointsInfo(const Eigen::Vector3d& cur_pos, std::vector<Eigen::Vector3d>& points, std::vector<double>& yaws,
                            std::vector<Eigen::Vector3d>& averages);
  // Get several viewpoints for a subset of frontiers
  void getViewpointsInfo(const Eigen::Vector3d& cur_pos, const std::vector<int>& ids, const int& view_num,
                         const double& max_decay, std::vector<std::vector<Eigen::Vector3d>>& points,
                         std::vector<std::vector<double>>& yaws);
  void updateFrontierCostMatrix();
  void getFullCostMatrix(const Eigen::Vector3d& cur_pos, const Eigen::Vector3d& cur_vel, const Eigen::Vector3d cur_yaw,
                         Eigen::MatrixXd& mat);
  void getPathForTour(const Eigen::Vector3d& pos, const std::vector<int>& frontier_ids, std::vector<Eigen::Vector3d>& path);

  void setNextFrontier(const int& id);
  bool isFrontierCovered();
  void wrapYaw(double& yaw);

  shared_ptr<PerceptionUtils> percep_utils_;

private:
  void splitLargeFrontiers(std::list<Frontier>& frontiers);
  bool splitHorizontally(const Frontier& frontier, std::list<Frontier>& splits);
  void mergeFrontiers(Frontier& ftr1, const Frontier& ftr2);
  bool isFrontierChanged(const Frontier& ft);
  bool haveOverlap( const voxblox::GlobalIndex& index_min1, const voxblox::GlobalIndex& index_max1,
                    const voxblox::GlobalIndex& index_min2, const voxblox::GlobalIndex& index_max2 );
  void computeFrontierInfo(Frontier& frontier);
  void downsample(const std::vector<Eigen::Vector3d>& cluster_in, std::vector<Eigen::Vector3d>& cluster_out);
  void sampleViewpoints(Frontier& frontier);

  int countVisibleCells(const Eigen::Vector3d& pos, const double& yaw, const std::vector<Eigen::Vector3d>& cluster);
  bool isNearUnknown( const voxblox::GlobalIndex &global_voxel_index );
  std::vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  std::vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel);
  std::vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);
  bool isNeighborUnknown( const voxblox::GlobalIndex &global_voxel_index );
  void expandFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */);

  // Wrapper of sdf map
  int toadr(const Eigen::Vector3i& idx);
  bool knownfree( const voxblox::GlobalIndex &global_voxel_index );
  bool inmap(const Eigen::Vector3i& idx);
  bool validDataIdx( const voxblox::GlobalIndex &global_voxel_index );
  VoxelStatus checkVoxelStatus( const voxblox::EsdfVoxel* voxel ) const;
  bool inFrontierMap( const ) const;

  // Deprecated
  bool isInBoxes( const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes, const voxblox::GlobalIndex &global_voxel_index );
  bool canBeMerged(const Frontier& ftr1, const Frontier& ftr2);
  void findViewpoints(const Eigen::Vector3d& sample, const Eigen::Vector3d& ftr_avg, std::vector<Viewpoint>& vps);


  Eigen::Vector3i globalIndexToEigenVector3i( voxblox::GlobalIndex& global_index );
  voxblox::GlobalIndex eigenVector3iToGlobalIndex( Eigen::Vector3i& idx );

  // Data
  std::vector<char> frontier_flag_;
  std::list<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_;
  std::vector<int> removed_ids_;
  std::list<Frontier>::iterator first_new_ftr_;
  Frontier next_frontier_;

  // Params
  int cluster_min_;
  double cluster_size_xy_, cluster_size_z_;
  double candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_,
      min_candidate_clearance_;
  int down_sample_;
  double min_view_finish_fraction_, resolution_;
  int min_visib_num_, candidate_rnum_;

  // Utils
//   std::shared_ptr<EDTEnvironment> edt_env_;
  std::unique_ptr<RayCaster> raycaster_;


  // new add : Yuchen
  // BlockHashMap block_map_;
  voxblox::FloatingPoint voxel_size_; 
  voxblox::EsdfServer esdf_server_;
  voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer_;

  ExplorationMapManager map_manager_;

  voxblox::FloatingPoint esdf_max_distance_m_ = 0.2;
  // multiplier of a single voxel size to consider as a distance metric for
  // occupancy threshold
  float occupancy_distance_voxelsize_factor_;

  void resetMap() { esdf_server_.clear(); }

  voxblox::Layer<voxblox::EsdfVoxel>* getESDFLayer() {
    return esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr();
  }

};

}  // namespace scoutair_planner
#endif