#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <boost/dynamic_bitset.hpp>
#include <visualization_msgs/Marker.h>

#include <voxblox/core/common.h>
#include <voxblox_ros/esdf_server.h>
#include <esdf_idx_updater/blockidx_updater.h>
#include <esdf_idx_updater/common.h>

#include <scoutair_planner/perception_utils.h>
#include <scoutair_planner/raycast.h>
#include <scoutair_planner/common.h>
#include <scoutair_planner/voxblox_map.h>
#include <scoutair_planner/frontier_visulization.h>

namespace scoutair_planner {

class FrontierMap 
{
public:
  typedef Eigen::Matrix<voxblox::LongIndexElement, 2, 8> IndexOffsets;

  enum ScanAxis { x_pos = 0, x_neg, y_pos, y_neg };

  FrontierMap( const ros::NodeHandle& nh, const ros::NodeHandle& nh_private );
  ~FrontierMap();

  bool init();

  bool initFrontierMap();

  void updateFrontierMap();

  // Search frontiers and group them into clusters
  void searchFrontiers();

  void computeFrontiersToVisit();

  void getFrontiers( std::vector<std::vector<Eigen::Vector3f>>& clusters );
  void getDormantFrontiers( std::vector<std::vector<Eigen::Vector3f>>& clusters );

  void getTopViewpointsInfo( const Eigen::Vector3f& cur_pos, std::vector<Eigen::Vector3f>& points, 
                             std::vector<float>& yaws, std::vector<Eigen::Vector3f>& averages ); 
  
  void getViewpointsInfo( const Eigen::Vector3f& cur_pos, const std::vector<int>& ids, const int& view_num,
                          const float& max_decay, std::vector<std::vector<Eigen::Vector3f>>& points,
                          std::vector<std::vector<float>>& yaws );
  
  void frontierCallback( const ros::TimerEvent& e );

  // void visCallback( const ros::TimerEvent& e );
                          
  void updateFrontierCostMatrix();

  void getFullCostMatrix( const Eigen::Vector3f& cur_pos, const Eigen::Vector3f& cur_vel, 
                          const Eigen::Vector3f cur_yaw, Eigen::MatrixXf& mat );
                         
  void getPathForTour( const Eigen::Vector3f& pos, const std::vector<int>& frontier_ids, std::vector<Eigen::Vector3f>& path );

  void visualizeFrontiers();

  bool whitelist( const Eigen::Vector3i &idx );

  bool isInBox( const Eigen::Vector3f &pos ) const;

  void findScanZone( Eigen::Vector3f &odom_pos, float &odom_yaw, voxblox::BlockIndexList &updated_blocks );

  void setOdom( Eigen::Vector3f &odom_pos, float &odom_yaw );

  FrontierMap::ScanAxis getScanAxisFromNum(int num); 

  // Voxblox Map
  std::shared_ptr<VoxbloxMap> voxblox_map_;
  std::shared_ptr<voxblox::Layer<voxblox::EsdfVoxel>> esdf_layer_;
  
  std::shared_ptr<PerceptionUtils> percep_utils_;

  // Parameters for ESDF
  static voxblox::FloatingPoint voxel_size_;
  static int voxels_per_side_;
  static int num_voxels_per_block_;

  Eigen::Vector3f voxbloxmap_origin_;

  esdf_idx_updater::BlockIdxUpdater block_idx_updater_;
  // Visulization
  std::shared_ptr<FtrVisulization> ftr_visu_;

private:
  
  int global_to_linearidx_in_List( const voxblox::GlobalIndex &idx, const Eigen::Vector3i &box_min ) const;
  int global_to_linearidx_in_List( const int& x, const int& y, const int& z, const Eigen::Vector3i &map_voxel_num ) const;
  int blockandlocal_to_linearidx_in_List( const voxblox::BlockIndex& block_idx, const voxblox::VoxelIndex& local_idx );

  bool haveOverlap( const voxblox::GlobalIndex& index_min1, const voxblox::GlobalIndex& index_max1,
                    const voxblox::GlobalIndex& index_min2, const voxblox::GlobalIndex& index_max2 );

  bool isFrontierChanged(const Frontier& ft);

  void expandFrontier( const voxblox::GlobalIndex &global_voxel_index, const voxblox::BlockIndex block_index,
                       const int &linear_index /* , const int& depth, const int& parent_id */);

  bool isInBox( const voxblox::GlobalIndex &global_voxel_index ) const;
  bool isInBox( const voxblox::GlobalIndex &global_voxel_index, const Eigen::Vector3i &box_min, const Eigen::Vector3i &box_max ) const;
  bool isInBox( const Eigen::Vector3f &pos, const Eigen::Vector3f &box_minf, const Eigen::Vector3f &box_maxf ) const;
  bool isInBox( const voxblox::BlockIndex &block_index, const Eigen::Vector3i &box_min_i, const Eigen::Vector3i &box_max_i ) const;

  void splitLargeFrontiers(std::list<Frontier>& frontiers);
  bool splitHorizontally(const Frontier& frontier, std::list<Frontier>& splits);

  void computeFrontierInfo(Frontier& frontier);

  void downsample(const std::vector<Eigen::Vector3f>& cluster_in, std::vector<Eigen::Vector3f>& cluster_out);

  void sampleViewpoints( Frontier& frontier ); 
  void sampleViewpoints( Frontier& frontier, double &candidate_max, double &candidate_min, double &candidate_diff, 
                         voxblox::FloatingPoint &voxel_size, int &min_visib_num );

  void wrapYaw(float& yaw);

  bool ifBlockinFOV( Eigen::Vector3f &odom_pos, float &odom_yaw, voxblox::BlockIndex &goal_idx );

  bool ifPointinFOV( Eigen::Vector3f &odom_pos, float &start_theta, float &end_theta, const voxblox::Point &point );

  void filterBlocksWithZEqualToZero( voxblox::BlockIndexList &updated_blocks );

  int countVisibleCells(const Eigen::Vector3f& pos, const float& yaw, const std::vector<Eigen::Vector3f>& cluster);

  Eigen::Vector3i globalIndexToEigenVector3i( voxblox::GlobalIndex& global_index );
  voxblox::GlobalIndex eigenVector3iToGlobalIndex( Eigen::Vector3i& idx );

  // ROS Parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ros::Timer frontier_timer_, visulization_timer_;

  ros::Publisher marker_pub;

  // Frontier Map
  // static Eigen::Vector3i box_min_, box_max_, map_voxel_num_;
  // Eigen::Vector3f box_minf_, box_maxf_;
  // static Eigen::Vector3i block_min_i_, block_max_i_;
  // Eigen::Vector3i map_voxel_num_;
  static Box_boundaries boundaries_;

  // Data
  std::list<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_;
  std::vector<int> removed_ids_;
  std::list<Frontier>::iterator first_new_ftr_;
  Frontier next_frontier_;

  // Params
  int cluster_min_;
  double cluster_size_xy_, cluster_size_z_;
  double candidate_max_, candidate_min_, candidate_dphi_, min_candidate_dist_,
      min_candidate_clearance_;
  int down_sample_;
  double min_view_finish_fraction_;
  int min_visib_num_;
  double candidate_diff_;

  bool inited = false;
  bool visu_flag_ = false;
  bool cost_update_ = false;
  // Utils
  std::unique_ptr<RayCaster> raycaster_;

  bool tryone = false;

  std::unordered_set<voxblox::BlockIndex> edited_block_idx_;
  std::unordered_set<int> searched_;
  // boost::dynamic_bitset<> ganzfree_bit_; 
  std::shared_ptr<boost::dynamic_bitset<>> ganzfree_bit_;
  std::unordered_set<int> frontier_flag_;

  // 更新的Blocks的索引列表
  voxblox::BlockIndexList updated_blocks_;

  Eigen::Vector3f odom_pos_;  // odometry state
  Eigen::Quaternionf odom_orient_;
  float odom_yaw_;
  voxblox::BlockIndex odom_block_index_;
  float half_theta_;
};


inline int FrontierMap::global_to_linearidx_in_List( const voxblox::GlobalIndex &idx, const Eigen::Vector3i &box_min ) const
{
  return global_to_linearidx_in_List( idx[0] - box_min[0], idx[1] - box_min[1], idx[2] - box_min[2], boundaries_.map_voxel_num_ );
}

inline int FrontierMap::global_to_linearidx_in_List( const int& x, const int& y, const int& z, const Eigen::Vector3i &map_voxel_num ) const
{
  return x * map_voxel_num(1) * map_voxel_num(2) + y * map_voxel_num(2) + z;
}

inline int FrontierMap::blockandlocal_to_linearidx_in_List( const voxblox::BlockIndex& block_idx, const voxblox::VoxelIndex& local_idx )
{
  voxblox::GlobalIndex idx = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex( block_idx, local_idx, voxels_per_side_ );
  return  global_to_linearidx_in_List( idx, boundaries_.box_min_ );
}

// 判断传入Voxel索引是否在预设的Box范围里
inline bool FrontierMap::isInBox( const voxblox::GlobalIndex &global_voxel_index ) const
{
    for (int i = 0; i < 3; ++i) {
        if (global_voxel_index[i] < boundaries_.box_min_[i] || global_voxel_index[i] >= boundaries_.box_max_[i]) {
        return false;
        }
    }
    return true;
}

inline bool FrontierMap::isInBox( const voxblox::GlobalIndex &global_voxel_index, const Eigen::Vector3i &box_min, const Eigen::Vector3i &box_max ) const
{
    for (int i = 0; i < 3; ++i) {
        if (global_voxel_index[i] < box_min[i] || global_voxel_index[i] >= box_max[i]) {
        return false;
        }
    }
    return true;
}

inline bool FrontierMap::isInBox( const Eigen::Vector3f &pos ) const
{
  return isInBox( pos, boundaries_.box_minf_, boundaries_.box_maxf_ );
}

inline bool FrontierMap::isInBox( const Eigen::Vector3f &pos, const Eigen::Vector3f &box_minf, const Eigen::Vector3f &box_maxf ) const
{
    for (int i = 0; i < 3; ++i) {
        if (pos[i] < box_minf[i] || pos[i] >= box_maxf[i])
            { return false; }
        }
    return true;
}

inline bool FrontierMap::isInBox( const voxblox::BlockIndex &block_index, const Eigen::Vector3i &box_min_i, const Eigen::Vector3i &box_max_i ) const
{
    for (int i = 0; i < 3; ++i) {
        if (block_index[i] < box_min_i[i] || block_index[i] > box_max_i[i]) 
            { return false; }
    }
    return true;
}

inline FrontierMap::ScanAxis FrontierMap::getScanAxisFromNum(int num) 
{
  switch(num) {
    case 0:
      return FrontierMap::ScanAxis::x_pos;
    case 1:
      return FrontierMap::ScanAxis::x_neg;
    case 2:
      return FrontierMap::ScanAxis::y_pos;
    case 3:
      return FrontierMap::ScanAxis::y_neg;
    default:
      return FrontierMap::ScanAxis::x_pos; // or handle the error appropriately
  }
}


}  // namespace scoutair_planner
#endif