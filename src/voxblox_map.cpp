#include <scoutair_planner/voxblox_map.h>

namespace scoutair_planner {

using namespace Eigen;
using namespace std;
using namespace voxblox;

size_t VoxbloxMap::num_voxels_per_block_;

VoxbloxMap::VoxbloxMap( const ros::NodeHandle& nh, const ros::NodeHandle& nh_private ) 
  : nh_(nh),
    nh_private_(nh_private)
{
  esdf_server_.reset(new voxblox::EsdfServer(nh, nh_private));
  esdf_layer_ = std::shared_ptr<voxblox::Layer<voxblox::EsdfVoxel>>(getESDFLayer());
  CHECK_NOTNULL(esdf_layer_.get());

  constexpr double kVoxelSizeEpsilon = 1e-5;
  nh_private_.param("Voxblox_Map/voxel_size", voxel_size_d_, voxel_size_d_);
  nh_private_.param("Voxblox_Map/voxels_per_side", voxels_per_side_, voxels_per_side_);
  nh_private_.param("Voxblox_Map/esdf_max_distance_m", esdf_max_distance_m_, esdf_max_distance_m_ );
  voxel_size_ = esdf_layer_->voxel_size();
  int voxels_per_side = esdf_layer_->voxels_per_side();

  if( std::abs(voxel_size_d_ - static_cast<double>(voxel_size_)) > kVoxelSizeEpsilon ) {
    ROS_ERROR( "Unmatched Parameter: Voxel size--%f, Resolution of Frontier Map--%f. Check Frontier Map or ESDF Map voxel size.", voxel_size_, voxel_size_d_ );
  } 
  if( voxels_per_side != voxels_per_side_ ) {
    ROS_ERROR( "Unmatched Parameter: Voxels_per_side in ESDF Map--%d, but in Frontier Map--%d.", voxels_per_side, voxels_per_side_ );
  } 

  // 调试信息：打印参数值
  ROS_INFO("Voxblox Map: voxel size: %f", voxel_size_d_);
  ROS_INFO("Voxblox Map: voxels_per_side: %d", voxels_per_side_);

  voxel_size_inv_ = 1.0 / voxel_size_;
  voxels_per_side_inv_ = 1.0f / static_cast<FloatingPoint>(voxels_per_side_);
  num_voxels_per_block_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;

  double min_candidate_clearance;
  float occupancy_distance_voxelsize_factor;
  nh_private_.param("Voxblox_Map/min_candidate_clearance", min_candidate_clearance, -1.0);
  nh_private_.param("Voxblox_Map/occupancy_distance_voxelsize_factor", occupancy_distance_voxelsize_factor, 1.0F );
  distance_thres_ = occupancy_distance_voxelsize_factor * voxel_size_ + 1e-6;
  vox_num_nearUnknown_ = floor( min_candidate_clearance / voxel_size_d_ );

  voxblox::GlobalIndex global_origin_index( 0, 0, 0 );
  map_origin_ = voxblox::getCenterPointFromGridIndex( global_origin_index, voxel_size_ );
  // map_origin_ = Vector3f( 0, 0, 0 );
}


VoxbloxMap::~VoxbloxMap() {
}


/// @brief 判断传入索引对应的Voxel的状态是否是kFree
/// @param global_voxel_index 
/// @return 
bool VoxbloxMap::knownfree( const voxblox::GlobalIndex &global_voxel_index ) 
{
  voxblox::EsdfVoxel* voxel = esdf_layer_->getVoxelPtrByGlobalIndex( global_voxel_index );
  return knownfree(voxel);
}


bool VoxbloxMap::knownfree( const voxblox::EsdfVoxel* voxel ) 
{
  VoxelStatus goal = kFree;
  return checkVoxelStatus(voxel, goal);
}


/// @brief 检查给定Voxel的状态是否与目标状态一致
/// @param voxel 
/// @param goal_status 
/// @return 
bool VoxbloxMap::checkVoxelStatus( const voxblox::EsdfVoxel* voxel, VoxbloxMap::VoxelStatus &goal_status ) const
{
  if (voxel == nullptr) 
    { return goal_status == VoxelStatus::kUnknown; }

  VoxbloxMap::VoxelStatus curr_status = checkVoxelStatus( voxel, esdf_max_distance_m_, distance_thres_ );
  return curr_status == goal_status;
}


VoxbloxMap::VoxelStatus VoxbloxMap::checkVoxelStatus( const voxblox::EsdfVoxel* voxel ) const
{
  return checkVoxelStatus( voxel, esdf_max_distance_m_, distance_thres_ );
}


/// @brief 检查给定Voxel的状态
/// @param voxel 
/// @return 
VoxbloxMap::VoxelStatus VoxbloxMap::checkVoxelStatus( const voxblox::EsdfVoxel* voxel, 
                                                        const voxblox::FloatingPoint &esdf_max_distance_m, 
                                                        const float &distance_thres ) const
{
  // std::cout<<"checkVoxelStatus"<<std::endl;
  if( voxel == nullptr || !voxel->observed || 
      voxel->distance + 3 * distance_thres >= esdf_max_distance_m || 
      voxel->distance <= -esdf_max_distance_m )
    { return VoxelStatus::kUnknown; }

  if( voxel->distance >= 2 * distance_thres )
    { return VoxelStatus::kFree; }

  if (voxel->fixed) 
    { return VoxelStatus::kOccupied; }

  return VoxelStatus::kUnknown;
}


VoxbloxMap::VoxelStatus VoxbloxMap::checkVoxelDistance( const voxblox::EsdfVoxel* voxel ) const
{
  return checkVoxelDistance( voxel, distance_thres_ );
}


VoxbloxMap::VoxelStatus VoxbloxMap::checkVoxelDistance( const voxblox::EsdfVoxel* voxel, const float &distance_thres ) const
{
  if( voxel != nullptr && voxel->observed ) {
    if ( voxel->distance < distance_thres ) 
      { return VoxelStatus::kOccupied; } 
    else 
      { return VoxelStatus::kFree; }
  } else 
    { return VoxelStatus::kUnknown; }
}


VoxbloxMap::VoxelStatus VoxbloxMap::checkVoxelDistance( const Eigen::Vector3d& point ) const
{
  double distance = 0.0;
  if (esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(point, &distance)) {
    // This means the voxel is observed
    if (distance < voxel_size_d_ ) {
      return VoxelStatus::kOccupied;
    } else {
      return VoxelStatus::kFree;
    }
  } else {
    return VoxelStatus::kUnknown;
  }
}


/// @brief 检查给定的 global_voxel_index 是否有未知的邻居体素
/// @param global_voxel_index 
/// @return 如果至少一个邻居是未知体素，则返回 true；否则返回 false
bool VoxbloxMap::isNeighborUnknown( const voxblox::GlobalIndex &global_voxel_index ) 
{
  // At least one neighbor is unknown
  // voxblox::EsdfVoxel* voxel = esdf_layer_->getVoxelPtrByGlobalIndex( global_voxel_index );
  // CHECK_NOTNULL(voxel);
  // 获取 6 连接邻域的索引
  voxblox::Neighborhood<Connectivity::kSix>::IndexMatrix neighbor_indices;
  voxblox::Neighborhood<Connectivity::kSix>::getFromGlobalIndex(global_voxel_index, &neighbor_indices);

  for (unsigned int idx = 0u; idx < neighbor_indices.cols(); ++idx) {
    const voxblox::GlobalIndex& neighbor_index = neighbor_indices.col(idx);
    voxblox::EsdfVoxel* neighbor_voxel = esdf_layer_->getVoxelPtrByGlobalIndex(neighbor_index);
    if( checkVoxelStatus( neighbor_voxel, esdf_max_distance_m_, distance_thres_ ) == VoxelStatus::kUnknown ) 
      return true ;
  }
  return false;
}


bool VoxbloxMap::isNearUnknown( const voxblox::GlobalIndex &global_voxel_index ) const
{
  return isNearUnknown( global_voxel_index, vox_num_nearUnknown_ );
}


/// @brief 检查给定的 global_voxel_index 是否在一定范围内接近未知体素
/// @param global_voxel_index 
/// @return 
bool VoxbloxMap::isNearUnknown( const voxblox::GlobalIndex &global_voxel_index, const int &vox_num_nearUnknown ) const
{
  // for (int x = -vox_num_nearUnknown; x <= vox_num_nearUnknown; ++x)
  //   for (int y = -vox_num_nearUnknown; y <= vox_num_nearUnknown; ++y)
  //     for (int z = -1; z <= 1; ++z) {
  //       if( x == 0 && y == 0 && z == 0 ) continue;
  //       voxblox::GlobalIndex new_index;
  //       new_index << global_voxel_index[0] + x, global_voxel_index[1] + y, global_voxel_index[2] + z;
  //       voxblox::EsdfVoxel* voxel = esdf_layer_->getVoxelPtrByGlobalIndex( new_index );
  //       if( checkVoxelStatus( voxel, esdf_max_distance_m_, distance_thres_ ) == VoxelStatus::kUnknown ) return true;
  //     }
  // return false;

  const int x_start = global_voxel_index[0] - vox_num_nearUnknown;
  const int y_start = global_voxel_index[1] - vox_num_nearUnknown;
  const int z_start = global_voxel_index[2] - 1;
  const int x_end = global_voxel_index[0] + vox_num_nearUnknown;
  const int y_end = global_voxel_index[1] + vox_num_nearUnknown;
  const int z_end = global_voxel_index[2] + 1;

  for (int x = x_start; x <= x_end; ++x) {
    for (int y = y_start; y <= y_end; ++y) {
      for (int z = z_start; z <= z_end; ++z) {
        if (x == global_voxel_index[0] && y == global_voxel_index[1] && z == global_voxel_index[2]) {
          continue; // 跳过中心点
        }

        voxblox::GlobalIndex new_index(x, y, z); // 直接使用构造函数创建新索引
        voxblox::EsdfVoxel* voxel = esdf_layer_->getVoxelPtrByGlobalIndex(new_index);

        // 预先保存需要检查的状态
        if( checkVoxelStatus(voxel, esdf_max_distance_m_, distance_thres_) != VoxelStatus::kFree ) 
          { return true; }
      }
    }
  }
  return false;
}


bool VoxbloxMap::isNearUnknown( const Eigen::Vector3d& pos )
{
  return isNearUnknown( pos, vox_num_nearUnknown_ );
}


bool VoxbloxMap::isNearUnknown( const Eigen::Vector3d& pos, const int &vox_num_nearUnknown ) 
{
  // const int vox_num = floor( vox_num_nearUnknown / voxel_size_d_ );
  for (int x = -vox_num_nearUnknown; x <= vox_num_nearUnknown; ++x)
    for (int y = -vox_num_nearUnknown; y <= vox_num_nearUnknown; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0) 
          { continue; }
        Eigen::Vector3d vox;
        vox << pos[0] + x * voxel_size_d_, pos[1] + y * voxel_size_d_, pos[2] + z * voxel_size_d_;
        if ( checkVoxelDistance(vox) != VoxelStatus::kFree ) return true;
      }
  return false;
}



voxblox::BlockIndex VoxbloxMap::getBlockIndexFromPoint( Eigen::Vector3f &point )
{
  voxblox::GlobalIndex global_voxel_idx = voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>( point, voxel_size_inv_ );
  voxblox::BlockIndex block_index = voxblox::getBlockIndexFromGlobalVoxelIndex( global_voxel_idx, voxels_per_side_inv_ );
  return block_index;
}


voxblox::GlobalIndex VoxbloxMap::getGridIndexFromPoint(const voxblox::Point &point )
{
  return voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>( point, voxel_size_inv_ );
}


voxblox::BlockIndex VoxbloxMap::getBlockIndexFromGlobalVoxelIndex( const voxblox::GlobalIndex &global_voxel_idx )
{
  return voxblox::getBlockIndexFromGlobalVoxelIndex( global_voxel_idx, voxels_per_side_inv_ );
}


std::shared_ptr<voxblox::Block<voxblox::EsdfVoxel>> VoxbloxMap::getBlockPtrByIndex( const voxblox::BlockIndex &index )
{ 
  return esdf_layer_->getBlockPtrByIndex(index);
} 


/// @brief 
/// @param global_index 
/// @return 
Eigen::Vector3i VoxbloxMap::globalIndexToEigenVector3i( voxblox::GlobalIndex& global_index ) 
{
  Eigen::Vector3i idx;
  idx[0] = static_cast<int>(global_index[0]);
  idx[1] = static_cast<int>(global_index[1]);
  idx[2] = static_cast<int>(global_index[2]);
  return idx;
}


/// @brief 
/// @param idx 
/// @return 
voxblox::GlobalIndex VoxbloxMap::eigenVector3iToGlobalIndex( Eigen::Vector3i& idx ) 
{
  voxblox::GlobalIndex global_index;
  global_index[0] = static_cast<voxblox::LongIndexElement>(idx[0]);
  global_index[1] = static_cast<voxblox::LongIndexElement>(idx[1]);
  global_index[2] = static_cast<voxblox::LongIndexElement>(idx[2]);
  return global_index;
}


}  // namespace scoutair_planner