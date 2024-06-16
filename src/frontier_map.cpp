#include <scoutair_planner/frontier_map.h>
#include <scoutair_planner/graph_node.h>

#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/Marker.h>

namespace scoutair_planner {

using namespace Eigen;
using namespace std;
using namespace voxblox;

// 初始化静态成员变量
Eigen::Vector3i FrontierMap::box_min_ = (Eigen::Vector3i() << 0, 0, 0).finished();
Eigen::Vector3i FrontierMap::box_max_ = (Eigen::Vector3i() << 0, 0, 0).finished();
Eigen::Vector3i FrontierMap::map_voxel_num_;

Eigen::Vector3i FrontierMap::block_min_i_;
Eigen::Vector3i FrontierMap::block_max_i_;
voxblox::FloatingPoint FrontierMap::voxel_size_;
int FrontierMap::voxels_per_side_;
int FrontierMap::num_voxels_per_block_;

FrontierMap::FrontierMap( const ros::NodeHandle& nh, const ros::NodeHandle& nh_private ) 
  : nh_(nh),
    nh_private_(nh_private),
    block_idx_updater_( nh, nh_private )
{
  // esdf_server_.reset(new voxblox::EsdfServer(nh, nh_private));
  // esdf_layer_ = std::unique_ptr<voxblox::Layer<voxblox::EsdfVoxel>>(getESDFLayer());
  voxblox_map_.reset(new VoxbloxMap(nh, nh_private));
  esdf_layer_ = voxblox_map_->getESDFLayerSharedPtr();
  // CHECK_NOTNULL(esdf_layer_.get());

  // constexpr double kVoxelSizeEpsilon = 1e-5;
  // nh_private_.param("Voxblox_Map/voxel_size", voxel_size_d_, voxel_size_d_);
  // nh_private_.param("Voxblox_Map/voxels_per_side", voxels_per_side_, voxels_per_side_);
  // nh_private_.param("Voxblox_Map/esdf_max_distance_m", esdf_max_distance_m_, esdf_max_distance_m_ );
  // voxel_size_ = esdf_layer_->voxel_size();
  // int voxels_per_side = esdf_layer_->voxels_per_side();

  // if( std::abs(voxel_size_d_ - static_cast<double>(voxel_size_)) > kVoxelSizeEpsilon ) {
  //   ROS_ERROR( "Unmatched Parameter: Voxel size--%f, Resolution of Frontier Map--%f. Check Frontier Map or ESDF Map voxel size.", voxel_size_, voxel_size_d_ );
  // } 
  // if( voxels_per_side != voxels_per_side_ ) {
  //   ROS_ERROR( "Unmatched Parameter: Voxels_per_side in ESDF Map--%d, but in Frontier Map--%d.", voxels_per_side, voxels_per_side_ );
  // } 
  // // 调试信息：打印参数值
  // ROS_INFO("Voxblox Map: voxel size: %f", voxel_size_d_);
  // ROS_INFO("Voxblox Map: voxels_per_side: %d", voxels_per_side_);

  // voxel_size_inv_ = 1.0 / voxel_size_;
  // voxels_per_side_inv_ = 1.0f / static_cast<FloatingPoint>(voxels_per_side_);
  // num_voxels_per_block_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;

  voxblox_map_->getVoxelResolution( voxel_size_ );
  voxblox_map_->getVoxelPerSide( voxels_per_side_ );
  voxblox_map_->getNumVoxelsPerBlock( num_voxels_per_block_ );
  

  // Try retriving bounding box of map, set box to map size if not specified
  Vector3d map_min_boundary( -10, -15, 0 );
  Vector3d map_max_boundary( 10, 15, 3 );
  vector<string> axis = { "x", "y", "z" };
  Eigen::Vector3d box_mind, box_maxd;
  for (int i = 0; i < 3; ++i) {
    nh_private_.param("FrontierMap/box_min_" + axis[i], box_mind[i], map_min_boundary[i]);
    nh_private_.param("FrontierMap/box_max_" + axis[i], box_maxd[i], map_max_boundary[i]);
  }
  box_minf_ = box_mind.cast<float>();
  box_maxf_ = box_maxd.cast<float>();

  Vector3f map_size = Vector3f( box_maxf_[0] - box_minf_[0], box_maxf_[1] - box_minf_[1], box_maxf_[2] - box_minf_[2] );
  for (int i = 0; i < 3; ++i) 
  { 
    map_voxel_num_(i) = ceil( map_size(i) / voxel_size_ ); 
    box_min_(i) = ceil(box_minf_(i) / voxel_size_);
    box_max_(i) = ceil(box_maxf_(i) / voxel_size_);
  }
  // 计算 block_index 的最小值和最大值
  block_min_i_ = box_min_.array() / voxels_per_side_;
  block_max_i_ = box_max_.array() / voxels_per_side_;

  int voxel_num = map_voxel_num_[0] * map_voxel_num_[1] * map_voxel_num_[2];
  ganzfree_bit_.reserve(voxel_num);
  ganzfree_bit_.resize(voxel_num);
  ganzfree_bit_.reset();

  nh_private_.param("frontier/cluster_min", cluster_min_, -1);
  nh_private_.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
  // nh_private_.param("frontier/cluster_size_z", cluster_size_z_, -1.0);
  nh_private_.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
  // nh_private_.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
  nh_private_.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
  nh_private_.param("frontier/candidate_max", candidate_max_, -1.0);
  nh_private_.param("frontier/candidate_min", candidate_min_, -1.0);
  nh_private_.param("frontier/candidate_diff", candidate_diff_, -1.0);
  nh_private_.param("frontier/down_sample", down_sample_, -1);
  nh_private_.param("frontier/min_visib_num", min_visib_num_, -1);
  nh_private_.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);

  // double min_candidate_clearance;
  // float occupancy_distance_voxelsize_factor;
  // nh_private_.param("Voxblox_Map/min_candidate_clearance", min_candidate_clearance, -1.0);
  // nh_private_.param("Voxblox_Map/occupancy_distance_voxelsize_factor", occupancy_distance_voxelsize_factor, 1.0F );
  // distance_thres_ = occupancy_distance_voxelsize_factor_ * voxel_size_ + 1e-6;
  // vox_num_nearUnknown_ = floor( min_candidate_clearance_ / voxel_size_d_ );

  Vector3f map_origin = Vector3f( 0, 0, 0 );
  raycaster_.reset(new RayCaster);
  raycaster_->setParams( voxel_size_, map_origin );

  percep_utils_.reset(new PerceptionUtils(nh_private_));

  frontier_timer_ = nh_private_.createTimer(ros::Duration(0.2), &FrontierMap::frontierCallback, this);
  visulization_timer_ = nh_private_.createTimer(ros::Duration(0.5), &FrontierMap::visCallback, this);
  frontier_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/planning_vis/frontier", 10000);

  uccu_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/planning_vis/kOccupied", 10000);
  free_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/planning_vis/kFree", 10000);
  unkno_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/planning_vis/kUnknown", 10000);

  std::cout << "Frontier Map inited" << std::endl;
}


FrontierMap::~FrontierMap() {
}


// bool FrontierMap::init()
// {
//   voxblox::FloatingPoint voxel_size_ = esdf_layer_->voxel_size();
//   int voxels_per_side = esdf_layer_->voxels_per_side();

//   if( std::abs(voxel_size_d_ - static_cast<double>(voxel_size_)) > kVoxelSizeEpsilon ) {
//     ROS_ERROR( "Unmatched Parameter: Voxel size--%f, Resolution of Frontier Map--%f. Check Frontier Map or ESDF Map voxel size.", voxel_size_, voxel_size_d_ );
//   } 
//   if( voxels_per_side != voxels_per_side_ ) {
//     ROS_ERROR( "Unmatched Parameter: Voxels_per_side in ESDF Map--%d, but in Frontier Map--%d.", voxels_per_side, voxels_per_side_ );
//   } 
//   // 调试信息：打印参数值
//   ROS_INFO("Voxblox Map: voxel size: %f", voxel_size_d_);
//   ROS_INFO("Voxblox Map: voxels_per_side: %d", voxels_per_side_);
// }


void FrontierMap::frontierCallback( const ros::TimerEvent& e ) 
{
    static int delay = 0;
    if (delay++ < 2) return;

    // 更新的Blocks的索引列表
    BlockIndexList updated_blocks;

    // 获取更新的区块
    if (!block_idx_updater_.getUpdatedBolocks(updated_blocks)) {
        ROS_WARN_THROTTLE( 2.0, "Wait until ESDF map from voxblox node..." );
        return; // 如果未初始化，提前返回
    }

    //  if( !tryone ) {
    //   std::vector<voxblox::BlockIndex> vectorList;
    //   // vectorList.push_back(updated_blocks.front());
    //   vectorList.assign(updated_blocks.begin(), updated_blocks.end());
    //   publishBlock( vectorList );
    //   std::cout<< "Finish Publish" << std::endl;
    //   tryone = true;
    // }

    ros::Time t4 = ros::Time::now();
    searchFrontiers( updated_blocks );
    ROS_WARN( "searchFrontiers: %lf", (ros::Time::now() - t4).toSec());

    ros::Time t5 = ros::Time::now();
    computeFrontiersToVisit();
    ROS_WARN( "computeFrontiersToVisit: %lf", (ros::Time::now() - t5).toSec());
    // updateFrontierCostMatrix();
    visu_flag_ = true;
}


void FrontierMap::visCallback( const ros::TimerEvent& e ) 
{
    std::vector<std::vector<Eigen::Vector3f>> clusters; 
    getFrontiers( clusters );
    if( !clusters.empty() ) {
      for (int i = 0; i < clusters.size(); ++i ) {
        drawCubes( clusters[i], 0.1, getColor(float(i) / clusters.size(), 0.4), "frontier", i );
      }
    }
    visu_flag_ = false;
}


void FrontierMap::searchFrontiers( voxblox::BlockIndexList &updated_blocks ) 
{
  ros::Time t1 = ros::Time::now();
  tmp_frontiers_.clear();

  // Removed changed frontiers in updated blocks
  auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& frontiers ) 
  {
    for (const auto& element : iter->idx_) {
      int lin_idx = global_to_linearidx_in_List(element, box_min_);
      frontier_flag_.erase(lin_idx);
    }
    edited_block_idx_.insert( iter->main_block_index );
    iter = frontiers.erase(iter);
  };

  removed_ids_.clear();
  int rmv_idx = 0;

  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) 
  {
    if( isFrontierChanged(*iter) ) 
    {
      resetFlag(iter, frontiers_);
      removed_ids_.push_back(rmv_idx);
    } else {
      ++rmv_idx;
      ++iter;
    }
  }

  for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();) 
  {
    if( isFrontierChanged(*iter) ) 
      { resetFlag(iter, dormant_frontiers_); } 
    else 
      { ++iter; }
  }

  // 加上Frontier状态有改变的BlockIndex
  for (const auto& block_idx : edited_block_idx_) {
    updated_blocks.push_back(block_idx);
  }
  size_t size = updated_blocks.size();
  if( size <= 0 ) return;

  edited_block_idx_.clear();
  searched_.clear();
  std::cout << "updated + edited block size: " << updated_blocks.size() << std::endl;


  // Search new frontier within box slightly inflated from updated box
  for (const BlockIndex& block_index : updated_blocks) {

    voxblox::Block<EsdfVoxel>::Ptr esdf_block = esdf_layer_->getBlockPtrByIndex(block_index);
    if (!esdf_block) {
      ROS_WARN("Getting wrong block index");
      continue;
    }

    for (ssize_t lin_index = num_voxels_per_block_ - 1; lin_index >= 0u; lin_index-- ) {
      VoxelIndex local_idx = esdf_block->computeVoxelIndexFromLinearIndex( lin_index );
      GlobalIndex global_idx = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex( block_index, local_idx, voxels_per_side_ );

      int linear_index = global_to_linearidx_in_List( global_idx, box_min_ );

      if( searched_.find(linear_index) != searched_.end() ) 
        { continue; }
      
      if( !isInBox(global_idx, box_min_, box_max_) 
          || ganzfree_bit_.test(linear_index) 
          || frontier_flag_.find(linear_index) != frontier_flag_.end() ) { 
        searched_.insert( linear_index );
        continue; 
      }
      
      // if( knownfree(global_idx) )  {
      //   if(isNeighborUnknown(global_idx) ) {
      if( voxblox_map_->knownfree(global_idx) )  {
        if( voxblox_map_->isNeighborUnknown(global_idx) ) {            
          // Expand from the seed cell to find a complete frontier cluster
          expandFrontier( global_idx, block_index, linear_index );
        } else {
          ganzfree_bit_.set(linear_index);
          searched_.insert( linear_index );
        }
      }
    }
  }

  splitLargeFrontiers(tmp_frontiers_);
}



void FrontierMap::computeFrontiersToVisit() 
{
  first_new_ftr_ = frontiers_.end();
  int new_num = 0;
  // Try find viewpoints for each cluster and categorize them according to viewpoint number
  for (auto& tmp_ftr : tmp_frontiers_) {
    // Search viewpoints around frontier
    sampleViewpoints( tmp_ftr, candidate_max_, candidate_min_, candidate_diff_, voxel_size_, min_visib_num_ );
    if (!tmp_ftr.viewpoints_.empty()) {
      ++new_num;
      list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), tmp_ftr);
      // Sort the viewpoints by coverage fraction, best view in front
      sort(
          inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
          [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
      if (first_new_ftr_ == frontiers_.end()) first_new_ftr_ = inserted;
    } else {
      // Find no viewpoint, move cluster to dormant list
      dormant_frontiers_.push_back(tmp_ftr);
    }
  }

  frontiers_.splice(frontiers_.end(), tmp_frontiers_);

  // Reset indices of frontiers
  int idx = 0;
  for (auto& ft : frontiers_) {
    ft.id_ = idx++;
  }
}


void FrontierMap::getFrontiers( std::vector<std::vector<Eigen::Vector3f>>& clusters ) 
{
  clusters.clear();
  for (auto frontier : frontiers_)
    clusters.push_back(frontier.cells_);
}


/// @brief 用于获取视点信息. 遍历每个 frontier，根据当前位置 cur_pos 和视点的覆盖范围选择合适的视点，最后将这些视点存储在 points、yaws 和 averages 中
/// @param cur_pos 
/// @param points 
/// @param yaws 
/// @param averages 
void FrontierMap::getTopViewpointsInfo( const Eigen::Vector3f& cur_pos, std::vector<Eigen::Vector3f>& points, 
                                        std::vector<float>& yaws, std::vector<Eigen::Vector3f>& averages ) 
{
  points.clear();
  yaws.clear();
  averages.clear();
  for (auto frontier : frontiers_) {
    bool no_view = true;
    for (auto view : frontier.viewpoints_) {
      // Retrieve the first viewpoint that is far enough and has highest coverage
      if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
      no_view = false;
      break;
    }
    if (no_view) {
      // All viewpoints are very close, just use the first one (with highest coverage).
      auto view = frontier.viewpoints_.front();
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
    }
  }
}


void FrontierMap::getViewpointsInfo( const Eigen::Vector3f& cur_pos, const std::vector<int>& ids, const int& view_num,
                                     const float& max_decay, std::vector<std::vector<Eigen::Vector3f>>& points,
                                     std::vector<std::vector<float>>& yaws ) 
{
  points.clear();
  yaws.clear();
  for (auto id : ids) {
    // Scan all frontiers to find one with the same id
    for (auto frontier : frontiers_) {
      if (frontier.id_ == id) {
        // Get several top viewpoints that are far enough
        std::vector<Eigen::Vector3f> pts;
        std::vector<float> ys;
        int visib_thresh = frontier.viewpoints_.front().visib_num_ * max_decay;
        for (auto view : frontier.viewpoints_) {
          if (pts.size() >= view_num || view.visib_num_ <= visib_thresh) break;
          if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
          pts.push_back(view.pos_);
          ys.push_back(view.yaw_);
        }
        if (pts.empty()) {
          // All viewpoints are very close, ignore the distance limit
          for (auto view : frontier.viewpoints_) {
            if (pts.size() >= view_num || view.visib_num_ <= visib_thresh) break;
            pts.push_back(view.pos_);
            ys.push_back(view.yaw_);
          }
        }
        points.push_back(pts);
        yaws.push_back(ys);
      }
    }
  }
}


void FrontierMap::drawFrontier( const std::vector<std::vector<Eigen::Vector3f>>& frontiers ) 
{
  for (int i = 0; i < frontiers.size(); ++i) {
    // displayCubeList(frontiers[i], 0.1, getColor(double(i) / frontiers.size(),
    // 0.4), i, 4);
    drawCubes(frontiers[i], 0.1, getColor(float(i) / frontiers.size(), 0.8), "frontier", i );
  }

  // vector<Eigen::Vector3f> frontier;
  // for (int i = frontiers.size(); i < last_frontier_num_; ++i) {
  //   // displayCubeList(frontier, 0.1, getColor(1), i, 4);
  //   drawCubes(frontier, 0.1, getColor(1), "frontier", i);
  // }

  // last_frontier_num_ = frontiers.size();
}


bool FrontierMap::whitelist( const Eigen::Vector3i idx )
{
  int linear_index = (idx[0] - box_min_[0]) * map_voxel_num_(1) * map_voxel_num_(2) + 
                      (idx[1] - box_min_[1]) * map_voxel_num_(2) + (idx[2] - box_min_[2]);
  if( !ganzfree_bit_.test(linear_index) ) {
    return false;
  }
  return true;
}


// /// @brief 更新前沿（frontier）之间的成本矩阵和路径矩阵
// void FrontierMap::updateFrontierCostMatrix() 
// {
//   // 删除路径和成本
//   if (!removed_ids_.empty()) {
//     // Delete path and cost for removed clusters
//     for (auto it = frontiers_.begin(); it != first_new_ftr_; ++it) {
//       auto cost_iter = it->costs_.begin();
//       auto path_iter = it->paths_.begin();
//       int iter_idx = 0;
//       for (int i = 0; i < removed_ids_.size(); ++i) {
//         // Step iterator to the item to be removed
//         while (iter_idx < removed_ids_[i]) {
//           ++cost_iter;
//           ++path_iter;
//           ++iter_idx;
//         }
//         cost_iter = it->costs_.erase(cost_iter);
//         path_iter = it->paths_.erase(path_iter);
//       }
//       // std::cout << "(" << it->costs_.size() << "," << it->paths_.size() << "), ";
//     }
//     removed_ids_.clear();
//   }

//   auto updateCost = [](const list<Frontier>::iterator& it1, const list<Frontier>::iterator& it2) 
//   {
//     Viewpoint& vui = it1->viewpoints_.front();
//     Viewpoint& vuj = it2->viewpoints_.front();
//     vector<Vector3f> path_ij;
//     float cost_ij = ViewNode::computeCost(
//         vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3f(0, 0, 0), 0, path_ij);
//     // Insert item for both old and new clusters
//     it1->costs_.push_back(cost_ij);
//     it1->paths_.push_back(path_ij);
//     reverse(path_ij.begin(), path_ij.end());
//     it2->costs_.push_back(cost_ij);
//     it2->paths_.push_back(path_ij);
//   };

//   // Compute path and cost between old and new clusters
//   for (auto it1 = frontiers_.begin(); it1 != first_new_ftr_; ++it1)
//     for (auto it2 = first_new_ftr_; it2 != frontiers_.end(); ++it2)
//       updateCost(it1, it2);

//   // Compute path and cost between new clusters
//   for (auto it1 = first_new_ftr_; it1 != frontiers_.end(); ++it1)
//     for (auto it2 = it1; it2 != frontiers_.end(); ++it2) {
//       if (it1 == it2) {
//         it1->costs_.push_back(0);
//         it1->paths_.push_back({});
//       } else
//         updateCost(it1, it2);
//     }
// }


/// @brief 
/// @param index_min1 
/// @param index_max1 
/// @param index_min2 
/// @param index_max2 
/// @return 
bool FrontierMap::haveOverlap( const voxblox::GlobalIndex& index_min1, const voxblox::GlobalIndex& index_max1,
                               const voxblox::GlobalIndex& index_min2, const voxblox::GlobalIndex& index_max2 ) 
{
  // Check if two box have overlap part
  voxblox::GlobalIndex bmin, bmax;
  for (int i = 0; i < 3; ++i) {
    bmin[i] = std::max(index_min1[i], index_min2[i]);
    bmax[i] = std::min(index_max1[i], index_max2[i]);
    if (bmin[i] > bmax[i] + 1e-3) return false;
  }
  return true;
}


/// @brief 
/// @param ft 
/// @return 
bool FrontierMap::isFrontierChanged( const Frontier& ft ) 
{
  for (const auto& element : ft.idx_) {
    if ( !voxblox_map_->isNeighborUnknown(element) ) 
      return true;
  }
  return false;
}


// /// @brief 判断传入索引对应的Voxel的状态是否是kFree
// /// @param global_voxel_index 
// /// @return 
// bool FrontierMap::knownfree( const voxblox::GlobalIndex &global_voxel_index ) 
// {
//   voxblox::EsdfVoxel* voxel = esdf_layer_->getVoxelPtrByGlobalIndex( global_voxel_index );
//   return knownfree(voxel);
// }

// bool FrontierMap::knownfree( const voxblox::EsdfVoxel* voxel ) 
// {
//   VoxelStatus goal = kFree;
//   return checkVoxelStatus(voxel, goal);
// }


// /// @brief 检查给定Voxel的状态是否与目标状态一致
// /// @param voxel 
// /// @param goal_status 
// /// @return 
// bool FrontierMap::checkVoxelStatus( const voxblox::EsdfVoxel* voxel, FrontierMap::VoxelStatus &goal_status ) const
// {
//   if (voxel == nullptr) 
//     { return goal_status == VoxelStatus::kUnknown; }

//   FrontierMap::VoxelStatus curr_status = checkVoxelStatus( voxel, esdf_max_distance_m_, distance_thres_ );
//   return curr_status == goal_status;
// }


// /// @brief 检查给定Voxel的状态
// /// @param voxel 
// /// @return 
// FrontierMap::VoxelStatus FrontierMap::checkVoxelStatus( const voxblox::EsdfVoxel* voxel, 
//                                                         const voxblox::FloatingPoint &esdf_max_distance_m, 
//                                                         const float &distance_thres ) const
// {
//   if( voxel == nullptr || !voxel->observed || 
//       voxel->distance + 4 * distance_thres >= esdf_max_distance_m || 
//       voxel->distance <= -esdf_max_distance_m )
//     { return VoxelStatus::kUnknown; }

//   if( voxel->distance >= 2 * distance_thres )
//     { return VoxelStatus::kFree; }

//   if (voxel->fixed) 
//     { return VoxelStatus::kOccupied; }

//   return VoxelStatus::kUnknown;
// }


// FrontierMap::VoxelStatus FrontierMap::checkVoxelDistance( const voxblox::EsdfVoxel* voxel, const float &distance_thres ) const
// {
//   if( voxel != nullptr && voxel->observed ) {
//     if ( voxel->distance < distance_thres ) 
//       { return VoxelStatus::kOccupied; } 
//     else 
//       { return VoxelStatus::kFree; }
//   } else 
//     { return VoxelStatus::kUnknown; }
// }


// FrontierMap::VoxelStatus FrontierMap::checkVoxelDistance( const Eigen::Vector3d& point ) const
// {
//   double distance = 0.0;
//   if (esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(point, &distance)) {
//     // This means the voxel is observed
//     if (distance < voxel_size_d_ ) {
//       return VoxelStatus::kOccupied;
//     } else {
//       return VoxelStatus::kFree;
//     }
//   } else {
//     return VoxelStatus::kUnknown;
//   }
// }


// /// @brief 检查给定的 global_voxel_index 是否有未知的邻居体素
// /// @param global_voxel_index 
// /// @return 如果至少一个邻居是未知体素，则返回 true；否则返回 false
// bool FrontierMap::isNeighborUnknown( const voxblox::GlobalIndex &global_voxel_index ) 
// {
//   // At least one neighbor is unknown
//   // voxblox::EsdfVoxel* voxel = esdf_layer_->getVoxelPtrByGlobalIndex( global_voxel_index );
//   // CHECK_NOTNULL(voxel);
//   // 获取 6 连接邻域的索引
//   voxblox::Neighborhood<Connectivity::kSix>::IndexMatrix neighbor_indices;
//   voxblox::Neighborhood<Connectivity::kSix>::getFromGlobalIndex(global_voxel_index, &neighbor_indices);

//   for (unsigned int idx = 0u; idx < neighbor_indices.cols(); ++idx) {
//     const voxblox::GlobalIndex& neighbor_index = neighbor_indices.col(idx);
//     voxblox::EsdfVoxel* neighbor_voxel = esdf_layer_->getVoxelPtrByGlobalIndex(neighbor_index);
//     if( checkVoxelStatus( neighbor_voxel, esdf_max_distance_m_, distance_thres_ ) == VoxelStatus::kUnknown ) 
//       return true ;
//   }
//   return false;
// }


void FrontierMap::expandFrontier( const voxblox::GlobalIndex &global_voxel_index, const voxblox::BlockIndex block_index,
                                  const int &linear_index /* , const int& depth, const int& parent_id */ ) 
{
  // auto t1 = ros::Time::now();

  // Data for clustering
  queue<voxblox::GlobalIndex> cell_queue;
  std::vector<voxblox::Point> expanded;
  std::vector<voxblox::GlobalIndex> frontier_idx;
  voxblox::Point point_tmp = voxblox::getCenterPointFromGridIndex( global_voxel_index, voxel_size_ );

  // 添加新的Frontier
  frontier_idx.push_back(global_voxel_index);
  expanded.push_back(point_tmp);
  frontier_flag_.insert(linear_index);
  cell_queue.push(global_voxel_index);

  searched_.insert( linear_index );

  // Search frontier cluster based on region growing (distance clustering)
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    voxblox::Neighborhood<>::IndexMatrix neighbor_indices;
    voxblox::Neighborhood<>::getFromGlobalIndex(cur, &neighbor_indices);
    for (ssize_t idx = neighbor_indices.cols() - 1; idx >= 0u; idx--) {
      // Qualified cell should be inside bounding box and frontier cell not clustered
      const voxblox::GlobalIndex& neighbor_index = neighbor_indices.col(idx);
      int neighbor_lin_idx = global_to_linearidx_in_List(neighbor_index, box_min_);

      if( searched_.find(neighbor_lin_idx) != searched_.end() ) {
        { continue; }
      } else if ( frontier_flag_.find(neighbor_lin_idx) != frontier_flag_.end() 
                  || ganzfree_bit_.test(neighbor_lin_idx)
                  || !voxblox_map_->knownfree(neighbor_index) 
                  || !isInBox(neighbor_index, box_min_, box_max_) ) 
        { searched_.insert( neighbor_lin_idx ); 
          continue; }         
      else if ( !voxblox_map_->isNeighborUnknown(neighbor_index) ) {
        ganzfree_bit_.set(neighbor_lin_idx);
        searched_.insert( neighbor_lin_idx ); 
        continue;
      }
      //   !(knownfree(neighbor_index) && isNeighborUnknown(neighbor_index)) 
      //             || !isInBox(neighbor_index, box_min_, box_max_) ) { 
      //   // ganzfree_bit_.set(neighbor_lin_idx);
      //   searched_.insert( neighbor_lin_idx );
      //   continue; 
      // }

      searched_.insert( neighbor_lin_idx );
      point_tmp = voxblox::getCenterPointFromGridIndex( neighbor_index, voxel_size_ );
      if ( point_tmp[2] < 0.2 ) continue;  // Remove noise close to ground

    //   searched_bit_.set(neighbor_lin_idx);
      frontier_idx.push_back(neighbor_index);
      expanded.push_back(point_tmp);
      // frontier_flag_[neighbor_lin_idx] = 1;
      frontier_flag_.insert(neighbor_lin_idx);
      cell_queue.push(neighbor_index);
    }
  }

  if (expanded.size() >= cluster_min_) {
    // Compute detailed info
    Frontier frontier;
    frontier.idx_ = frontier_idx;
    frontier.cells_ = expanded;
    // frontier.main_block_index = block_index;
    // frontier.block_indices_ =
    computeFrontierInfo(frontier);
    tmp_frontiers_.push_back(frontier);
  }
  // ROS_WARN( "iteration_expandFrontier: %lf", (ros::Time::now() - t1).toSec());
}



/// @brief 
/// @param frontiers 
void FrontierMap::splitLargeFrontiers( list<Frontier>& frontiers ) 
{
  list<Frontier> splits, tmps;
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) 
  {
    // Check if each frontier needs to be split horizontally
    if (splitHorizontally(*it, splits)) 
    {
      tmps.insert(tmps.end(), splits.begin(), splits.end());
      splits.clear();
    } 
    else { tmps.push_back(*it); }
  }
  frontiers = tmps;
}


/// @brief 将一个大的前沿（frontier）分割成较小的部分。如果前沿的大小超出一定阈值，
///        它会被沿着其主成分（principal component）方向分割
/// @param frontier 
/// @param splits 
/// @return 
bool FrontierMap::splitHorizontally( const Frontier& frontier, list<Frontier>& splits ) 
{
  // Split a frontier into small piece if it is too large
  auto mean = frontier.average_.head<2>();
  bool need_split = false;
  for (auto cell : frontier.filtered_cells_) {
    if ((cell.head<2>() - mean).norm() > cluster_size_xy_) {
      need_split = true;
      break;
    }
  }
  if (!need_split) return false;

  // Compute principal component
  // Covariance matrix of cells
  Eigen::Matrix2f cov;
  cov.setZero();
  for (auto cell : frontier.filtered_cells_) {
    Eigen::Vector2f diff = cell.head<2>() - mean;
    cov += diff * diff.transpose();
  }
  cov /= float(frontier.filtered_cells_.size());

  // Find eigenvector corresponds to maximal eigenvector
  Eigen::EigenSolver<Eigen::Matrix2f> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  float max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector2f first_pc = vectors.col(max_idx);

  // Split the frontier into two groups along the first PC
  Frontier ftr1, ftr2;
  for ( size_t i = 0; i < frontier.idx_.size(); i++ ) {
    auto element = frontier.idx_[i];
    if ((frontier.cells_[i].head<2>() - mean).dot(first_pc) >= 0) {
      ftr1.idx_.push_back(element);
      ftr1.cells_.push_back(frontier.cells_[i]);
    } else {
      ftr2.idx_.push_back(element);
      ftr2.cells_.push_back(frontier.cells_[i]);
    }
  }
  computeFrontierInfo(ftr1);
  computeFrontierInfo(ftr2);

  // Recursive call to split frontier that is still too large
  list<Frontier> splits2;
  if (splitHorizontally(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  } else
    splits.push_back(ftr1);

  if (splitHorizontally(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}


/// @brief 
/// @param ftr 
void FrontierMap::computeFrontierInfo( Frontier& ftr ) 
{
  // Compute average position and bounding box of cluster
  ftr.average_.setZero();

  for (auto cell : ftr.cells_) {
    ftr.average_ += cell;
  }
  ftr.average_ /= float(ftr.cells_.size());

  GlobalIndex global_voxel_idx = voxblox_map_->getGridIndexFromPoint( ftr.average_ );
  BlockIndex block_index = voxblox_map_->getBlockIndexFromGlobalVoxelIndex( global_voxel_idx );

  ftr.main_block_index = block_index;

  // Compute downsampled cluster
  downsample( ftr.cells_, ftr.filtered_cells_ );
}


/// @brief 
/// @param cluster_in 
/// @param cluster_out 
void FrontierMap::downsample( const std::vector<Eigen::Vector3f>& cluster_in, std::vector<Eigen::Vector3f>& cluster_out ) 
{
  // downsamping cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto cell : cluster_in)
    cloud->points.emplace_back(cell[0], cell[1], cell[2]);

  const float leaf_size = voxel_size_ * down_sample_;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudf);

  cluster_out.clear();
  for (auto pt : cloudf->points)
    cluster_out.emplace_back(pt.x, pt.y, pt.z);
}


void FrontierMap::sampleViewpoints( Frontier& frontier ) 
{
  sampleViewpoints( frontier, candidate_max_, candidate_min_, candidate_diff_, voxel_size_, min_visib_num_ );
} 


// Sample viewpoints around frontier's average position, check coverage to the frontier cells
void FrontierMap::sampleViewpoints( Frontier& frontier, double &candidate_max, double &candidate_min, double &candidate_diff, 
                                    voxblox::FloatingPoint &voxel_size, int &min_visib_num ) 
{   
  // Evaluate sample viewpoints on circles, find ones that cover most cells
  const auto& cells = frontier.filtered_cells_;
  int num_cells = cells.size();
  const Eigen::Vector3d frontier_avg = frontier.average_.cast<double>();

  for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    for ( double rc = candidate_min; rc <= candidate_max + 1e-3; rc += candidate_diff )
    {   
      const Vector3d sample_pos = frontier_avg + rc * Vector3d(cos_phi, sin_phi, 0);

      if( voxblox_map_->checkVoxelDistance(sample_pos) != VoxbloxMap::VoxelStatus::kFree 
          || voxblox_map_->isNearUnknown(sample_pos) )
        { break; }

      // Compute average yaw
      const Vector3f sample_pos_f = sample_pos.cast<float>();
      Vector3f ref_dir = (cells.front() - sample_pos_f).normalized();
      float avg_yaw = 0.0;

      for (int i = 1; i < num_cells; ++i) 
      {
        Vector3f dir = (cells[i] - sample_pos_f).normalized();
        float yaw = acos(dir.dot(ref_dir));
        if (ref_dir.cross(dir)[2] < 0) yaw = -yaw;
        avg_yaw += yaw;
      }
      // avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
      avg_yaw = avg_yaw / (num_cells - 1) + atan2(ref_dir[1], ref_dir[0]);
      wrapYaw(avg_yaw);
    
      // Compute the fraction of covered and visible cells
      int visib_num = countVisibleCells(sample_pos_f, avg_yaw, cells);
      if (visib_num >= min_visib_num) {
        Viewpoint vp = { sample_pos_f, avg_yaw, visib_num };
        frontier.viewpoints_.push_back(vp);
      }
    }
  }
}


// /// @brief 检查给定的 global_voxel_index 是否在一定范围内接近未知体素
// /// @param global_voxel_index 
// /// @return 
// bool FrontierMap::isNearUnknown( const voxblox::GlobalIndex &global_voxel_index, const int &vox_num_nearUnknown ) const
// {
//   // for (int x = -vox_num_nearUnknown; x <= vox_num_nearUnknown; ++x)
//   //   for (int y = -vox_num_nearUnknown; y <= vox_num_nearUnknown; ++y)
//   //     for (int z = -1; z <= 1; ++z) {
//   //       if( x == 0 && y == 0 && z == 0 ) continue;
//   //       voxblox::GlobalIndex new_index;
//   //       new_index << global_voxel_index[0] + x, global_voxel_index[1] + y, global_voxel_index[2] + z;
//   //       voxblox::EsdfVoxel* voxel = esdf_layer_->getVoxelPtrByGlobalIndex( new_index );
//   //       if( checkVoxelStatus( voxel, esdf_max_distance_m_, distance_thres_ ) == VoxelStatus::kUnknown ) return true;
//   //     }
//   // return false;

//   const int x_start = global_voxel_index[0] - vox_num_nearUnknown;
//   const int y_start = global_voxel_index[1] - vox_num_nearUnknown;
//   const int z_start = global_voxel_index[2] - 1;
//   const int x_end = global_voxel_index[0] + vox_num_nearUnknown;
//   const int y_end = global_voxel_index[1] + vox_num_nearUnknown;
//   const int z_end = global_voxel_index[2] + 1;

//   for (int x = x_start; x <= x_end; ++x) {
//     for (int y = y_start; y <= y_end; ++y) {
//       for (int z = z_start; z <= z_end; ++z) {
//         if (x == global_voxel_index[0] && y == global_voxel_index[1] && z == global_voxel_index[2]) {
//           continue; // 跳过中心点
//         }

//         voxblox::GlobalIndex new_index(x, y, z); // 直接使用构造函数创建新索引
//         voxblox::EsdfVoxel* voxel = esdf_layer_->getVoxelPtrByGlobalIndex(new_index);

//         // 预先保存需要检查的状态
//         if( checkVoxelStatus(voxel, esdf_max_distance_m_, distance_thres_) == VoxelStatus::kUnknown) {
//           return true;
//         }
//       }
//     }
//   }
//   return false;
// }


// bool FrontierMap::isNearUnknown( const Eigen::Vector3d& pos, const int &vox_num_nearUnknown ) 
// {
//   // const int vox_num = floor( vox_num_nearUnknown / voxel_size_d_ );
//   for (int x = -vox_num_nearUnknown; x <= vox_num_nearUnknown; ++x)
//     for (int y = -vox_num_nearUnknown; y <= vox_num_nearUnknown; ++y)
//       for (int z = -1; z <= 1; ++z) {
//         if (x == 0 && y == 0 && z == 0) 
//           { continue; }
//         Eigen::Vector3d vox;
//         vox << pos[0] + x * voxel_size_d_, pos[1] + y * voxel_size_d_, pos[2] + z * voxel_size_d_;
//         if ( checkVoxelDistance(vox) == VoxelStatus::kUnknown ) return true;
//       }
//   return false;
// }


/// @brief 
/// @param yaw 
void FrontierMap::wrapYaw(float& yaw) {
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  while (yaw > M_PI)
    yaw -= 2 * M_PI;
}


/// @brief 
/// @param pos 
/// @param yaw 
/// @param cluster 
/// @return 
int FrontierMap::countVisibleCells( const Eigen::Vector3f& pos, const float& yaw, const std::vector<Eigen::Vector3f>& cluster ) 
{
  percep_utils_->setPose(pos, yaw);
  int visib_num = 0;
  Eigen::Vector3i idx;
  // voxblox::FloatingPoint esdf_max_distance_tmp = esdf_max_distance_m_;
  // float distance_thres_tmp = distance_thres_;

  for (auto cell : cluster) {
    // Check if frontier cell is inside FOV
    if (!percep_utils_->insideFOV(cell)) continue;

    // Check if frontier cell is visible (not occulded by obstacles)
    raycaster_->input(cell, pos);
    bool visib = true;
    

    while (raycaster_->nextId(idx)) {
      if( !whitelist(idx) ) {
        visib = false;
        break;
      }
    }
    if (visib) visib_num += 1;
  }
  return visib_num;
}


/// @brief 
/// @param global_index 
/// @return 
Eigen::Vector3i FrontierMap::globalIndexToEigenVector3i( voxblox::GlobalIndex& global_index ) 
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
voxblox::GlobalIndex FrontierMap::eigenVector3iToGlobalIndex( Eigen::Vector3i& idx ) 
{
  voxblox::GlobalIndex global_index;
  global_index[0] = static_cast<voxblox::LongIndexElement>(idx[0]);
  global_index[1] = static_cast<voxblox::LongIndexElement>(idx[1]);
  global_index[2] = static_cast<voxblox::LongIndexElement>(idx[2]);
  return global_index;
}


void FrontierMap::drawCubes( const std::vector<Eigen::Vector3f>& list, const float& scale, const Eigen::Vector4f& color,
                             const std::string& ns, const int& id /* const int& pub_id */ ) 
{
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3f(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::CUBE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  frontier_pub_.publish(mk);

  // pub new marker
  fillGeometryInfo(mk, list);
  mk.action = visualization_msgs::Marker::ADD;
  frontier_pub_.publish(mk);
  ros::Duration(0.000001).sleep();
}


void FrontierMap::fillBasicInfo( visualization_msgs::Marker& mk, const Eigen::Vector3f& scale,
                                 const Eigen::Vector4f& color, const std::string& ns, const int& id, const int& shape ) 
{
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = id;
  mk.ns = ns;
  mk.type = shape;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = scale[0];
  mk.scale.y = scale[1];
  mk.scale.z = scale[2];
}


void FrontierMap::fillGeometryInfo( visualization_msgs::Marker& mk,
                                    const std::vector<Eigen::Vector3f>& list ) 
{
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
}


void FrontierMap::fillGeometryInfo( visualization_msgs::Marker& mk,
                                    const std::vector<Eigen::Vector3f>& list1,
                                    const std::vector<Eigen::Vector3f>& list2 ) 
{
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
}


Eigen::Vector4f FrontierMap::getColor( const float& h, float alpha ) 
{
  float h1 = h;
  if (h1 < 0.0 || h1 > 1.0) {
    std::cout << "h out of range" << std::endl;
    h1 = 0.0;
  }

  float lambda;
  Eigen::Vector4f color1, color2;
  if (h1 >= -1e-4 && h1 < 1.0 / 6) {
    lambda = (h1 - 0.0) * 6;
    color1 = Eigen::Vector4f(1, 0, 0, 1);
    color2 = Eigen::Vector4f(1, 0, 1, 1);
  } else if (h1 >= 1.0 / 6 && h1 < 2.0 / 6) {
    lambda = (h1 - 1.0 / 6) * 6;
    color1 = Eigen::Vector4f(1, 0, 1, 1);
    color2 = Eigen::Vector4f(0, 0, 1, 1);
  } else if (h1 >= 2.0 / 6 && h1 < 3.0 / 6) {
    lambda = (h1 - 2.0 / 6) * 6;
    color1 = Eigen::Vector4f(0, 0, 1, 1);
    color2 = Eigen::Vector4f(0, 1, 1, 1);
  } else if (h1 >= 3.0 / 6 && h1 < 4.0 / 6) {
    lambda = (h1 - 3.0 / 6) * 6;
    color1 = Eigen::Vector4f(0, 1, 1, 1);
    color2 = Eigen::Vector4f(0, 1, 0, 1);
  } else if (h1 >= 4.0 / 6 && h1 < 5.0 / 6) {
    lambda = (h1 - 4.0 / 6) * 6;
    color1 = Eigen::Vector4f(0, 1, 0, 1);
    color2 = Eigen::Vector4f(1, 1, 0, 1);
  } else if (h1 >= 5.0 / 6 && h1 <= 1.0 + 1e-4) {
    lambda = (h1 - 5.0 / 6) * 6;
    color1 = Eigen::Vector4f(1, 1, 0, 1);
    color2 = Eigen::Vector4f(1, 0, 0, 1);
  }

  Eigen::Vector4f fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;

  return fcolor;
}


void FrontierMap::publishBlock( const std::vector<voxblox::BlockIndex>& block_index_list )
{
  if( block_index_list.empty() ) 
    { return; }

  visualization_msgs::Marker mk_occu, mk_free, mk_unk;
  Eigen::Vector3f scale( 0.05, 0.05, 0.05 );
  initVisualizationMarker( mk_occu, scale, Eigen::Vector4f(1.0, 0.0, 0.0, 1.0 ) );
  initVisualizationMarker( mk_free, scale, Eigen::Vector4f(1.0, 1.0, 1.0, 1.0 ) );
  initVisualizationMarker( mk_unk, scale, Eigen::Vector4f(0.0, 0.0, 1.0, 1.0 ) );
  ros::Duration lifetime(200); // 200s
  mk_occu.lifetime = lifetime;
  mk_free.lifetime = lifetime;
  mk_unk.lifetime = lifetime;

  for(const auto block_index: block_index_list ) {
    voxblox::Block<EsdfVoxel>::Ptr esdf_block = esdf_layer_->getBlockPtrByIndex(block_index);
    if (!esdf_block) {
      std::cout << "Getting empty block index, publishBlock interrupt" << std::endl;
      return;
    }
    const size_t num_voxels_per_block = esdf_block->num_voxels();
    
    for (size_t id = 0u; id < num_voxels_per_block; ++id) {
      VoxelIndex local_idx = esdf_block->computeVoxelIndexFromLinearIndex( id );
      GlobalIndex global_idx = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex( block_index, local_idx, voxels_per_side_ );
      voxblox::EsdfVoxel* esdf_voxel = esdf_layer_->getVoxelPtrByGlobalIndex(global_idx);
      VoxbloxMap::VoxelStatus status = voxblox_map_->checkVoxelStatus( esdf_voxel );
      voxblox::Point point_tmp = voxblox::getCenterPointFromGridIndex( global_idx, voxel_size_ );

      // pub new marker
      geometry_msgs::Point pt;
      pt.x = point_tmp[0];
      pt.y = point_tmp[1];
      pt.z = point_tmp[2];

      switch (status) {
        case VoxbloxMap::VoxelStatus::kUnknown:
          mk_unk.points.push_back(pt);
          break;
        case VoxbloxMap::VoxelStatus::kFree:
          mk_free.points.push_back(pt);
          break;
        case VoxbloxMap::VoxelStatus::kOccupied:
          mk_occu.points.push_back(pt);
          break;
      }           
      ros::Duration(0.00001).sleep();
    }
  }
  unkno_pub_.publish(mk_unk);
  free_pub_.publish(mk_free);
  uccu_pub_.publish(mk_occu);
}


void FrontierMap::initVisualizationMarker( visualization_msgs::Marker &mk, const Eigen::Vector3f& scale, const Eigen::Vector4f& color )
{
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = scale[0];
  mk.scale.y = scale[1];
  mk.scale.z = scale[2];
  mk.action = visualization_msgs::Marker::ADD;
}

}  // namespace scoutair_planner