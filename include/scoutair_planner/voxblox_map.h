#ifndef _VOXBLOX_MAP_H_
#define _VOXBLOX_MAP_H_

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <voxblox/core/common.h>
#include <voxblox_ros/esdf_server.h>


namespace scoutair_planner {

class VoxbloxMap 
{
public:
  typedef Eigen::Matrix<voxblox::LongIndexElement, 2, 8> IndexOffsets;

  enum VoxelStatus { kUnknown = 0, kOccupied, kFree };

  VoxbloxMap( const ros::NodeHandle& nh, const ros::NodeHandle& nh_private );
  ~VoxbloxMap();

  std::shared_ptr<voxblox::Layer<voxblox::EsdfVoxel>> getESDFLayerSharedPtr() 
    { return esdf_layer_; }

  void getVoxelResolution( double &resolution );
  void getVoxelResolution( float &resolution );
  void getMapOrigin( Eigen::Vector3f &map_origin );
  void getVoxelPerSide( int &voxels_per_side );
  void getNumVoxelsPerBlock( int &num_voxels_per_block );

  bool knownfree( const voxblox::GlobalIndex &global_voxel_index );
  bool knownfree( const voxblox::EsdfVoxel* voxel );

  bool checkVoxelStatus( const voxblox::EsdfVoxel* voxel, VoxbloxMap::VoxelStatus &goal_status ) const;
  VoxbloxMap::VoxelStatus checkVoxelStatus( const voxblox::EsdfVoxel* voxel ) const;

  VoxbloxMap::VoxelStatus checkVoxelDistance( const voxblox::EsdfVoxel* voxel ) const;
  VoxbloxMap::VoxelStatus checkVoxelDistance( const Eigen::Vector3d& point ) const;


  bool isNeighborUnknown( const voxblox::GlobalIndex &global_voxel_index );

  bool isNearUnknown( const voxblox::GlobalIndex &global_voxel_index ) const;
  bool isNearUnknown( const Eigen::Vector3d& pos ); 

  voxblox::GlobalIndex getGridIndexFromPoint( const voxblox::Point &point );

  voxblox::BlockIndex getBlockIndexFromPoint( Eigen::Vector3f &point );

  voxblox::BlockIndex getBlockIndexFromGlobalVoxelIndex( const voxblox::GlobalIndex &global_voxel_idx );

  std::shared_ptr<voxblox::Block<voxblox::EsdfVoxel>> getBlockPtrByIndex( const voxblox::BlockIndex &index );

private:
  voxblox::Layer<voxblox::EsdfVoxel>* getESDFLayer() 
    { return esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr(); }

  // bool knownfree( const voxblox::GlobalIndex &global_voxel_index );
  // bool knownfree( const voxblox::EsdfVoxel* voxel );

  // bool checkVoxelStatus( const voxblox::EsdfVoxel* voxel, VoxbloxMap::VoxelStatus &goal_status ) const;
  VoxbloxMap::VoxelStatus checkVoxelStatus( const voxblox::EsdfVoxel* voxel, const voxblox::FloatingPoint &esdf_max_distance_m, 
                                             const float &distance_thres ) const;

  VoxbloxMap::VoxelStatus checkVoxelDistance( const voxblox::EsdfVoxel* voxel, const float &distance_thres ) const;
  // VoxbloxMap::VoxelStatus checkVoxelDistance( const Eigen::Vector3d& point ) const;


  // bool isNeighborUnknown( const voxblox::GlobalIndex &global_voxel_index );

  bool isNearUnknown( const voxblox::GlobalIndex &global_voxel_index, const int &vox_num_nearUnknown ) const;
  bool isNearUnknown( const Eigen::Vector3d& pos, const int &vox_num_nearUnknown ); 

  Eigen::Vector3i globalIndexToEigenVector3i( voxblox::GlobalIndex& global_index );
  voxblox::GlobalIndex eigenVector3iToGlobalIndex( Eigen::Vector3i& idx );

  // ROS Parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

//   ros::Publisher uccu_pub_, free_pub_, unkno_pub_;

  // Parameters for ESDF
  voxblox::FloatingPoint voxel_size_;
  double voxel_size_d_;
  float voxel_size_inv_;

  int voxels_per_side_ = 2;
  voxblox::FloatingPoint voxels_per_side_inv_;

  voxblox::FloatingPoint esdf_max_distance_m_ = 2.0;
  static size_t num_voxels_per_block_;
  // multiplier of a single voxel size to consider as a distance metric for
  // occupancy threshold
  float occupancy_distance_voxelsize_factor_;
  float distance_thres_;
//   double min_candidate_clearance_;
  int vox_num_nearUnknown_;
  Eigen::Vector3f map_origin_;
  
  // voxblox::EsdfServer esdf_server_;
  std::unique_ptr<voxblox::EsdfServer> esdf_server_;
  std::shared_ptr<voxblox::Layer<voxblox::EsdfVoxel>> esdf_layer_;
};

inline void VoxbloxMap::getVoxelResolution( double &resolution )
{
  resolution = voxel_size_d_;
}

inline void VoxbloxMap::getVoxelResolution( float &resolution )
{
  resolution = voxel_size_;
}

inline void VoxbloxMap::getMapOrigin( Eigen::Vector3f &map_origin )
{
  map_origin = map_origin_;
}

inline void VoxbloxMap::getVoxelPerSide( int &voxels_per_side )
{
  voxels_per_side = voxels_per_side_;
}

inline void VoxbloxMap::getNumVoxelsPerBlock( int &num_voxels_per_block )
{
  num_voxels_per_block = num_voxels_per_block_;
}

}  // namespace scoutair_planner

#endif // _VOXBLOX_MAP_H_