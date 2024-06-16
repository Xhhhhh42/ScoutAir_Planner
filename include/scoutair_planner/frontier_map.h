#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <boost/dynamic_bitset.hpp>

#include <voxblox/core/common.h>
#include <voxblox_ros/esdf_server.h>
#include <esdf_idx_updater/blockidx_updater.h>
#include <esdf_idx_updater/common.h>

#include <scoutair_planner/perception_utils.h>
#include <scoutair_planner/raycast.h>
#include <scoutair_planner/common.h>
#include <scoutair_planner/voxblox_map.h>

namespace scoutair_planner {

// Viewpoint to cover a frontier cluster
struct Viewpoint 
{
  // Position and heading
  Eigen::Vector3f pos_;
  float yaw_;
  // Fraction of the cluster that can be covered
  // double fraction_;
  int visib_num_;
};

// A frontier cluster, the viewpoints to cover it
struct Frontier {
  std::vector<voxblox::GlobalIndex> idx_;
  std::unordered_set<voxblox::BlockIndex> block_indices_;
  voxblox::BlockIndex main_block_index;

  // Complete voxels belonging to the cluster
  std::vector<voxblox::Point> cells_;
  // down-sampled voxels filtered by voxel grid filter
  std::vector<voxblox::Point> filtered_cells_;
  // Average position of all voxels
  Eigen::Vector3f average_;
  // Idx of cluster
  int id_;
  // Viewpoints that can cover the cluster
  std::vector<Viewpoint> viewpoints_;
  // // Bounding box of cluster, center & 1/2 side length
  // Eigen::Vector3f box_min_, box_max_;

  // Path and cost from this cluster to other clusters
  std::list<std::vector<Eigen::Vector3f>> paths_;
  std::list<float> costs_;
};


class FrontierMap 
{
public:
  typedef Eigen::Matrix<voxblox::LongIndexElement, 2, 8> IndexOffsets;

  // enum VoxelStatus { kUnknown = 0, kOccupied, kFree };

  FrontierMap( const ros::NodeHandle& nh, const ros::NodeHandle& nh_private );
  ~FrontierMap();

  bool init();

  // Search frontiers and group them into clusters
  void searchFrontiers( voxblox::BlockIndexList &updated_blocks );

  void computeFrontiersToVisit();

  void getFrontiers( std::vector<std::vector<Eigen::Vector3f>>& clusters );
  void getDormantFrontiers( std::vector<std::vector<Eigen::Vector3f>>& clusters );
  void getFrontierBoxes( std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& boxes );

  void getTopViewpointsInfo( const Eigen::Vector3f& cur_pos, std::vector<Eigen::Vector3f>& points, 
                             std::vector<float>& yaws, std::vector<Eigen::Vector3f>& averages ); 
  
  void getViewpointsInfo( const Eigen::Vector3f& cur_pos, const std::vector<int>& ids, const int& view_num,
                          const float& max_decay, std::vector<std::vector<Eigen::Vector3f>>& points,
                          std::vector<std::vector<float>>& yaws );
  
  void frontierCallback( const ros::TimerEvent& e );

  void visCallback( const ros::TimerEvent& e );
                          
  void drawFrontier( const std::vector<std::vector<Eigen::Vector3f>>& frontiers );

  void updateFrontierCostMatrix();

  bool whitelist( const Eigen::Vector3i idx );

  shared_ptr<PerceptionUtils> percep_utils_;

private:
  // voxblox::Layer<voxblox::EsdfVoxel>* getESDFLayer() 
  //   { return esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr(); }
  
  int global_to_linearidx_in_List( const voxblox::GlobalIndex &idx, const Eigen::Vector3i &box_min ) const;
  int global_to_linearidx_in_List( const int& x, const int& y, const int& z, const Eigen::Vector3i &map_voxel_num ) const;
  int blockandlocal_to_linearidx_in_List( const voxblox::BlockIndex& block_idx, const voxblox::VoxelIndex& local_idx );

  bool haveOverlap( const voxblox::GlobalIndex& index_min1, const voxblox::GlobalIndex& index_max1,
                    const voxblox::GlobalIndex& index_min2, const voxblox::GlobalIndex& index_max2 );

  bool isFrontierChanged(const Frontier& ft);

  // bool knownfree( const voxblox::GlobalIndex &global_voxel_index );
  // bool knownfree( const voxblox::EsdfVoxel* voxel );

  // bool checkVoxelStatus( const voxblox::EsdfVoxel* voxel, FrontierMap::VoxelStatus &goal_status ) const;
  // FrontierMap::VoxelStatus checkVoxelStatus( const voxblox::EsdfVoxel* voxel, const voxblox::FloatingPoint &esdf_max_distance_m, 
  //                                            const float &distance_thres ) const;

  // FrontierMap::VoxelStatus checkVoxelDistance( const voxblox::EsdfVoxel* voxel, const float &distance_thres ) const;
  // FrontierMap::VoxelStatus checkVoxelDistance( const Eigen::Vector3d& point ) const;


  // bool isNeighborUnknown( const voxblox::GlobalIndex &global_voxel_index );

  void expandFrontier( const voxblox::GlobalIndex &global_voxel_index, const voxblox::BlockIndex block_index,
                       const int &linear_index /* , const int& depth, const int& parent_id */);

  bool isInBox( const voxblox::GlobalIndex &global_voxel_index ) const;
  bool isInBox( const Eigen::Vector3f &pos, const Eigen::Vector3f &box_minf, const Eigen::Vector3f &box_maxf ) const;
  bool isInBox( const voxblox::GlobalIndex &global_voxel_index, const Eigen::Vector3i &box_min, const Eigen::Vector3i &box_max ) const;
  bool isInBox( const voxblox::BlockIndex &block_index, const Eigen::Vector3i &box_min_i, const Eigen::Vector3i &box_max_i ) const;

  void splitLargeFrontiers(std::list<Frontier>& frontiers);
  bool splitHorizontally(const Frontier& frontier, std::list<Frontier>& splits);

  void computeFrontierInfo(Frontier& frontier);

  void downsample(const std::vector<Eigen::Vector3f>& cluster_in, std::vector<Eigen::Vector3f>& cluster_out);

  void sampleViewpoints( Frontier& frontier ); 
  void sampleViewpoints( Frontier& frontier, double &candidate_max, double &candidate_min, double &candidate_diff, 
                         voxblox::FloatingPoint &voxel_size, int &min_visib_num );

  // bool isNearUnknown( const voxblox::GlobalIndex &global_voxel_index, const int &vox_num_nearUnknown ) const;
  // bool isNearUnknown( const Eigen::Vector3d& pos, const int &vox_num_nearUnknown ); 

  void wrapYaw(float& yaw);

  int countVisibleCells(const Eigen::Vector3f& pos, const float& yaw, const std::vector<Eigen::Vector3f>& cluster);

  Eigen::Vector3i globalIndexToEigenVector3i( voxblox::GlobalIndex& global_index );
  voxblox::GlobalIndex eigenVector3iToGlobalIndex( Eigen::Vector3i& idx );

  void drawCubes( const std::vector<Eigen::Vector3f>& list, const float& scale, const Eigen::Vector4f& color,
                  const std::string& ns, const int& id /* const int& pub_id */ );
  
  void fillBasicInfo( visualization_msgs::Marker& mk, const Eigen::Vector3f& scale,
                      const Eigen::Vector4f& color, const std::string& ns, const int& id, const int& shape );
  
  void fillGeometryInfo( visualization_msgs::Marker& mk, const std::vector<Eigen::Vector3f>& list );
  void fillGeometryInfo( visualization_msgs::Marker& mk, const std::vector<Eigen::Vector3f>& list1,
                         const std::vector<Eigen::Vector3f>& list2);
  
  Eigen::Vector4f getColor(const float& h, float alpha = 1.0);

  void publishBlock( const std::vector<voxblox::BlockIndex>& block_index_list );

  void initVisualizationMarker( visualization_msgs::Marker &mk, const Eigen::Vector3f& scale, const Eigen::Vector4f& color );

  // ROS Parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer frontier_timer_, visulization_timer_;
  ros::Publisher frontier_pub_;
  ros::Publisher uccu_pub_, free_pub_, unkno_pub_;

  // Voxblox Map
  std::unique_ptr<VoxbloxMap> voxblox_map_;
  std::shared_ptr<voxblox::Layer<voxblox::EsdfVoxel>> esdf_layer_;

  // Parameters for ESDF
  static voxblox::FloatingPoint voxel_size_;
  // double voxel_size_d_;
  // float voxel_size_inv_;
  static int voxels_per_side_;
  // voxblox::FloatingPoint voxels_per_side_inv_;
  static int num_voxels_per_block_;
  
  // voxblox::FloatingPoint esdf_max_distance_m_ = 2.0;
  // // multiplier of a single voxel size to consider as a distance metric for
  // // occupancy threshold
  // float occupancy_distance_voxelsize_factor_;
  // float distance_thres_;
  
  // // voxblox::EsdfServer esdf_server_;
  // std::unique_ptr<voxblox::EsdfServer> esdf_server_;
  // std::unique_ptr<voxblox::Layer<voxblox::EsdfVoxel>> esdf_layer_;

  // test
  esdf_idx_updater::BlockIdxUpdater block_idx_updater_;


  // Frontier Map
  static Eigen::Vector3i box_min_, box_max_, map_voxel_num_;
  Eigen::Vector3f box_minf_, box_maxf_;
  static Eigen::Vector3i block_min_i_, block_max_i_;
  // Eigen::Vector3i map_voxel_num_;

  // Data
  // std::vector<char> frontier_flag_;
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

  // int vox_num_nearUnknown_;

  bool inited = false;
  bool visu_flag_ = false;
  // Utils
  std::unique_ptr<RayCaster> raycaster_;

  bool tryone = false;

  std::unordered_set<voxblox::BlockIndex> edited_block_idx_;
  std::unordered_set<int> searched_;
//   std::bitset<100> searched_;
//   boost::dynamic_bitset<> searched_bit_;
//   std::unordered_set<voxblox::GlobalIndex> ganz_free_;
  boost::dynamic_bitset<> ganzfree_bit_;  
  std::unordered_set<int> frontier_flag_;
  std::vector<unsigned char> flag_;
};


inline int FrontierMap::global_to_linearidx_in_List( const voxblox::GlobalIndex &idx, const Eigen::Vector3i &box_min ) const
{
  return global_to_linearidx_in_List( idx[0] - box_min[0], idx[1] - box_min[1], idx[2] - box_min[2], map_voxel_num_ );
}

inline int FrontierMap::global_to_linearidx_in_List( const int& x, const int& y, const int& z, const Eigen::Vector3i &map_voxel_num ) const
{
  return x * map_voxel_num(1) * map_voxel_num(2) + y * map_voxel_num(2) + z;
}

inline int FrontierMap::blockandlocal_to_linearidx_in_List( const voxblox::BlockIndex& block_idx, const voxblox::VoxelIndex& local_idx )
{
  voxblox::GlobalIndex idx = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex( block_idx, local_idx, voxels_per_side_ );
  return  global_to_linearidx_in_List( idx, box_min_ );
}

// 判断传入Voxel索引是否在预设的Box范围里
inline bool FrontierMap::isInBox( const voxblox::GlobalIndex &global_voxel_index ) const
{
    for (int i = 0; i < 3; ++i) {
        if (global_voxel_index[i] < box_min_[i] || global_voxel_index[i] >= box_max_[i]) {
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

}  // namespace scoutair_planner
#endif