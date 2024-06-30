#include <thread>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/Marker.h>

#include <scoutair_planner/frontier_map.h>
#include <scoutair_planner/graph_node.h>
#include <scoutair_planner/graph_node.h>


namespace scoutair_planner {

using namespace Eigen;
using namespace std;
using namespace voxblox;

// 初始化静态成员变量
// Eigen::Vector3i FrontierMap::box_min_ = (Eigen::Vector3i() << 0, 0, 0).finished();
// Eigen::Vector3i FrontierMap::box_max_ = (Eigen::Vector3i() << 0, 0, 0).finished();
// Eigen::Vector3i FrontierMap::map_voxel_num_;

// Eigen::Vector3i FrontierMap::block_min_i_;
// Eigen::Vector3i FrontierMap::block_max_i_;
Box_boundaries FrontierMap::boundaries_;
voxblox::FloatingPoint FrontierMap::voxel_size_;
int FrontierMap::voxels_per_side_;
int FrontierMap::num_voxels_per_block_;

FrontierMap::FrontierMap( const ros::NodeHandle& nh, const ros::NodeHandle& nh_private ) 
  : nh_(nh),
    nh_private_(nh_private)
    // block_idx_updater_(nh, nh_private),
    // voxblox_map_(std::make_shared<VoxbloxMap>(nh, nh_private)),
    // esdf_layer_(voxblox_map_->getESDFLayerSharedPtr()),
    // ftr_visu_(std::make_shared<FtrVisulization>(nh, nh_private, voxblox_map_))
{
  voxblox_map_.reset(new VoxbloxMap(nh_, nh_private_));
  esdf_layer_ = voxblox_map_->getESDFLayerSharedPtr();
  ftr_visu_ = std::make_shared<FtrVisulization>(nh, nh_private, voxblox_map_);
}


FrontierMap::~FrontierMap() {}


bool FrontierMap::init()
{
  voxblox_map_->getVoxelResolution( voxel_size_ );
  voxblox_map_->getVoxelPerSide( voxels_per_side_ );
  voxblox_map_->getNumVoxelsPerBlock( num_voxels_per_block_ );
  voxblox_map_->getMapOrigin( voxbloxmap_origin_ );

  // Try retriving bounding box of map, set box to map size if not specified
  Vector3d map_min_boundary( -10, -15, 0 );
  Vector3d map_max_boundary( 10, 15, 3 );
  vector<string> axis = { "x", "y", "z" };
  Eigen::Vector3d box_mind, box_maxd;
  for (int i = 0; i < 3; ++i) {
    nh_private_.param("FrontierMap/box_min_" + axis[i], box_mind[i], map_min_boundary[i]);
    nh_private_.param("FrontierMap/box_max_" + axis[i], box_maxd[i], map_max_boundary[i]);
  }
  boundaries_.box_minf_ = box_mind.cast<float>();
  boundaries_.box_maxf_ = box_maxd.cast<float>();

  Eigen::Vector3f map_size = boundaries_.box_maxf_ - boundaries_.box_minf_;

  for (int i = 0; i < 3; ++i) 
  { 
    boundaries_.map_voxel_num_(i) = ceil( map_size(i) / voxel_size_ ); 
    boundaries_.box_min_(i) = ceil(boundaries_.box_minf_(i) / voxel_size_);
    boundaries_.box_max_(i) = ceil(boundaries_.box_maxf_(i) / voxel_size_);
  }
  // 计算 block_index 的最小值和最大值
  boundaries_.block_min_i_ = boundaries_.box_min_.array() / voxels_per_side_;
  boundaries_.block_max_i_ = boundaries_.box_max_.array() / voxels_per_side_;

  int voxel_num = boundaries_.map_voxel_num_[0] * boundaries_.map_voxel_num_[1] * boundaries_.map_voxel_num_[2];
  ganzfree_bit_ = std::make_shared<boost::dynamic_bitset<>>(voxel_num);
  // ganzfree_bit_->reserve(voxel_num);
  // ganzfree_bit_->resize(voxel_num);
  ganzfree_bit_->reset();

  ViewNode::set( ganzfree_bit_, boundaries_ );

  nh_private_.param("frontier/cluster_min", cluster_min_, -1);
  nh_private_.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
  nh_private_.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
  nh_private_.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
  nh_private_.param("frontier/candidate_max", candidate_max_, -1.0);
  nh_private_.param("frontier/candidate_min", candidate_min_, -1.0);
  nh_private_.param("frontier/candidate_diff", candidate_diff_, -1.0);
  nh_private_.param("frontier/down_sample", down_sample_, -1);
  nh_private_.param("frontier/min_visib_num", min_visib_num_, -1);
  nh_private_.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);

  Vector3f map_origin = Vector3f( 0, 0, 0 );
  raycaster_.reset(new RayCaster);
  raycaster_->setParams( voxel_size_, map_origin );

  percep_utils_.reset(new PerceptionUtils(nh_private_));

  // frontier_timer_ = nh_private_.createTimer(ros::Duration(0.1), &FrontierMap::frontierCallback, this);
  // visulization_timer_ = nh_private_.createTimer(ros::Duration(0.3), &FrontierMap::visCallback, this);

  marker_pub = nh_private_.advertise<visualization_msgs::MarkerArray>("/planning_vis/sample_points", 10000);

  half_theta_ = std::atan2(4.0f, 3.0f);

  std::cout << "Frontier Map inited" << std::endl;
  return true;
}


void FrontierMap::frontierCallback( const ros::TimerEvent& e ) 
{
  static int delay = 0;
  if (delay++ < 2) return;

  // // 获取更新的区块
  // if (!block_idx_updater_.getUpdatedBolocks(updated_blocks_)) {
  //     ROS_WARN_THROTTLE( 2.0, "Wait until ESDF map from voxblox node..." );
  //     return; // 如果未初始化，提前返回
  // }

  // ros::Time t4 = ros::Time::now();
  // searchFrontiers();
  // ROS_WARN( "searchFrontiers: %lf", (ros::Time::now() - t4).toSec());

  // computeFrontiersToVisit();
  // updateFrontierCostMatrix();
  // visu_flag_ = true;
}


// void FrontierMap::visCallback( const ros::TimerEvent& e ) 
// {
//   std::vector<std::vector<Eigen::Vector3f>> clusters; 
//   getFrontiers( clusters );
//   if( !clusters.empty() ) {
//     for (int i = 0; i < clusters.size(); ++i ) {
//       ftr_visu_->drawCubes( clusters[i], 0.1, float(i) / clusters.size(), 0.4, "frontier", i );
//       // drawCubes( clusters[i], 0.1, getColor(float(i) / clusters.size(), 0.4), "frontier", i );
//     }
//   }
//   visu_flag_ = false;
// }


bool FrontierMap::initFrontierMap()
{
  // // 获取更新的区块
  // if (!block_idx_updater_.getUpdatedBolocks(updated_blocks_)) {
  //   ROS_WARN_THROTTLE( 2.0, "Wait until ESDF map from voxblox node..." );
  //   return false; // 如果未初始化，提前返回
  // }

  updated_blocks_.clear();
  esdf_layer_->getAllAllocatedBlocks(&updated_blocks_);
  if( updated_blocks_.empty() ) {
    ROS_WARN_THROTTLE( 2.0, "Wait until ESDF map from voxblox node..." );
    return false; // 如果未初始化，提前返回
  }

  // filterBlocksWithZEqualToZero(updated_blocks_);

  ros::Time t = ros::Time::now();
  searchFrontiers();
  ROS_WARN( "searchFrontiers: %lf", (ros::Time::now() - t).toSec());

  t = ros::Time::now();
  computeFrontiersToVisit();
  ROS_WARN( "computeFrontiersToVisit: %lf", (ros::Time::now() - t).toSec());
  visualizeFrontiers();
  t = ros::Time::now();
  updateFrontierCostMatrix();
  ROS_WARN( "updateFrontierCostMatrix: %lf", (ros::Time::now() - t).toSec());

  ROS_WARN_ONCE( "Frontier Map inited from ESDF map... ");
  return true;
}


void FrontierMap::updateFrontierMap()
{
  // esdf_layer_->getAllAllocatedBlocks(&updated_blocks_);
  // if( updated_blocks_.empty() ) {
  //   ROS_WARN_THROTTLE( 2.0, "Wait until ESDF map from voxblox node..." );
  //   return; // 如果未初始化，提前返回
  // }
  
  // ros::Time t3 = ros::Time::now();
  findScanZone( odom_pos_, odom_yaw_, updated_blocks_ );
  // ROS_WARN( "current time: %lf", ros::Time::now().toSec());
  // ROS_WARN( "findZone: %lf", (ros::Time::now() - t3).toSec());

  checkFrontiers();

  // ros::Time t4 = ros::Time::now();
  searchFrontiers();
  // ROS_WARN( "current time: %lf", ros::Time::now().toSec());
  // ROS_WARN( "searchFrontiers: %lf", (ros::Time::now() - t4).toSec());

  // ros::Time t5 = ros::Time::now();
  computeFrontiersToVisit();
  // ROS_WARN( "computeFrontiersToVisit: %lf", (ros::Time::now() - t5).toSec());
  // ros::Time t6 = ros::Time::now();
  updateFrontierCostMatrix();
  // ROS_WARN( "updateFrontierCostMatrix: %lf", (ros::Time::now() - t6).toSec());
}


void FrontierMap::findScanZone( Eigen::Vector3f &odom_pos, float odom_yaw, voxblox::BlockIndexList &updated_blocks )
{
  voxblox::BlockIndex odom_block_index = voxblox_map_->getBlockIndexFromPoint( odom_pos );
  if( odom_block_index != odom_block_index_ ) 
    { odom_block_index_ = odom_block_index; }
  
  updated_blocks.clear();
  updated_blocks.push_back(odom_block_index_);
  normalizeYaw( odom_yaw );

  for (int dx = -2; dx <= 2; ++dx) {
    for (int dy = -2; dy <= 2; ++dy) {
      if (dx == 0 && dy == 0) continue;

      voxblox::BlockIndex goal_idx = odom_block_index_;
      goal_idx.x() += dx;
      goal_idx.y() += dy;

      if( ifBlockinFOV( odom_pos, odom_yaw, goal_idx ) ) {
        updated_blocks.push_back(goal_idx);
      }      
    }
  }

  // int num = 0;
  // while( theta1 >= M_PI / 2 ) {
  //   theta1 -= M_PI / 2;
  //   num++;
  // }
  // ScanAxis theta1_axis = getScanAxisFromNum(num);
  // num = 0;
  // while( theta2 >= M_PI / 2 ) {
  //   theta2 -= M_PI / 2;
  //   num++;
  // }
  // ScanAxis theta2_axis = getScanAxisFromNum(num);

  // voxblox::BlockIndex tmp;
  // float angle1 = std::asin(1.6 / 3.0) * 180.0 / M_PI;;
  // switch (theta1_axis)
  // {
  //   case  x_pos:
  //     tmp = odom_block_index;
  //     tmp.x() += 1;
  //     updated_blocks_.push_back(tmp);
  //     tmp.x() += 1;
  //     updated_blocks_.push_back(tmp);
  //     if( theta1 > angle1 ) { 
  //       tmp.y() += 1;
  //       updated_blocks_.push_back(tmp);
  //     }
  //     if( theta1 > M_PI / 4 ) {
  //       tmp.x() -= 1;
  //       updated_blocks_.push_back(tmp);
  //     }
  //     break;

  //   case  x_neg:
  //     tmp = odom_block_index;
  //     tmp.x() -= 1;
  //     updated_blocks_.push_back(tmp);
  //     tmp.x() -= 1;
  //     updated_blocks_.push_back(tmp);
  //     if( theta1 > angle1 + M_PI ) { 
  //       tmp.y() -= 1;
  //       updated_blocks_.push_back(tmp);
  //     }
  //     if( theta1 > M_PI / 4 + M_PI ) {
  //       tmp.x() += 1;
  //       updated_blocks_.push_back(tmp);
  //     }
  //     break;

  //   case  y_pos:
  //     tmp = odom_block_index;
  //     tmp.y() += 1;
  //     updated_blocks_.push_back(tmp);
  //     tmp.y() += 1;
  //     updated_blocks_.push_back(tmp);
  //     if( theta1 > angle1 + M_PI / 2 ) { 
  //       tmp.x() -= 1;
  //       updated_blocks_.push_back(tmp);
  //     }
  //     if( theta1 > M_PI / 4 + M_PI / 2 ) {
  //       tmp.y() -= 1;
  //       updated_blocks_.push_back(tmp);
  //     }
  //     break;

  //   case  y_neg:
  //     tmp = odom_block_index;
  //     tmp.y() -= 1;
  //     updated_blocks_.push_back(tmp);
  //     tmp.y() -= 1;
  //     updated_blocks_.push_back(tmp);
  //     if( theta1 > angle1 + 3 * M_PI / 2 ) { 
  //       tmp.x() += 1;
  //       updated_blocks_.push_back(tmp);
  //     }
  //     if( theta1 > M_PI / 4 + 3 * M_PI / 2 ) {
  //       tmp.y() += 1;
  //       updated_blocks_.push_back(tmp);
  //     }
  //     break;
  // } 
}


void FrontierMap::setOdom( Eigen::Vector3f &odom_pos, float &odom_yaw )
{
  if( odom_pos.z() <= 0.7 ) {
    ROS_WARN( "Insufficient current altitude of drone");
    return;
  }
  if( !have_odom_ ) {
    odom_pos_ = odom_pos;
    odom_yaw_ = odom_yaw;
    have_odom_ = true;
    return;
  }

  // Define a threshold for significant change
  float pos_threshold = 0.25f; 
  float yaw_threshold = 5.0 * M_PI / 180.0; // Yaw threshold in radians (~5 degree)

  // Check if there is a significant difference in position or yaw
  bool pos_diff = (odom_pos_ - odom_pos).norm() > pos_threshold;
  bool yaw_diff = std::abs(odom_yaw_ - odom_yaw) > yaw_threshold;

  // if( pos_diff ) {
  //   ROS_WARN( "Motion detected");
  //   std::cout << "current pos: " << odom_pos_ << std::endl;
  //   std::cout << "new pos: " << odom_pos << std::endl;
  // }
  // if( yaw_diff ) {
  //   ROS_WARN( "Rotation detected");
  //   std::cout << "current yaw: " << odom_yaw_ << std::endl;
  //   std::cout << "new yaw: " << odom_yaw << std::endl;
  // }

  if (pos_diff || yaw_diff) {
    odom_pos_ = odom_pos;
    odom_yaw_ = odom_yaw;
    // ROS_WARN( "updateFrontierMap start: %lf", ros::Time::now().toSec());
    updateFrontierMap();
    // ROS_WARN( "updateFrontierMap end: %lf", ros::Time::now().toSec());
  }
}


void FrontierMap::checkFrontiers()
{
  edited_block_idx_.clear();

  // Removed changed frontiers in updated blocks
  auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& frontiers ) 
  {
    for (const auto& element : iter->idx_) {
      int lin_idx = global_to_linearidx_in_List(element, boundaries_.box_min_);
      frontier_flag_.erase(lin_idx);
    }
    for (const auto& block_id : iter->block_idx_) {
      edited_block_idx_.insert( block_id );
    }
    iter = frontiers.erase(iter);
  };

  std::cout << "Before remove: " << frontiers_.size() << std::endl;
  removed_ids_.clear();
  int rmv_idx = 0;

  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) 
  {
    bool found = false;
    for( const auto& block_id : iter->block_idx_ ) {
      if( std::find(updated_blocks_.begin(), updated_blocks_.end(), block_id) != updated_blocks_.end() ) {
        if( isFrontierChanged(*iter) ) 
          {
            resetFlag(iter, frontiers_);
            removed_ids_.push_back(rmv_idx);
            found = true;
            break;
          }
      }
    }
    if( found ) continue;
    ++rmv_idx;
    ++iter;
    // if( isFrontierChanged(*iter) ) 
    // {
    //   resetFlag(iter, frontiers_);
    //   removed_ids_.push_back(rmv_idx);
    // } else {
    //   ++rmv_idx;
    //   ++iter;
    // }
  }

  std::cout << "After remove: " << frontiers_.size() << std::endl;
  for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();) 
  {
    // if( isFrontierChanged(*iter) ) 
    //   { resetFlag(iter, dormant_frontiers_); } 
    // else 
    //   { ++iter; }
    bool found = false;
    for( const auto& block_id : iter->block_idx_ ) {
      if( std::find(updated_blocks_.begin(), updated_blocks_.end(), block_id) != updated_blocks_.end() ) {
        if( isFrontierChanged(*iter) ) 
          {
            resetFlag(iter, dormant_frontiers_);
            // removed_ids_.push_back(rmv_idx);
            found = true;
            break;
          }
      }
    }
    if( found ) continue;
    // ++rmv_idx;
    ++iter;
  }

  std::cout << "checkFrontiers end: " << frontiers_.size() << std::endl;
}


void FrontierMap::searchFrontiers() 
{
  std::cout << "Start search: " << frontiers_.size() << std::endl;
  tmp_frontiers_.clear();

  voxblox::BlockIndexList final_list;
  // 加上Frontier状态有改变的BlockIndex
  for (const auto& block_idx : updated_blocks_) {
    if( block_idx.z() == 0 ) 
      { final_list.push_back(block_idx); }
  }
  for (const auto& block_idx : edited_block_idx_) {
    if( block_idx.z() == 0 ) 
      { final_list.push_back(block_idx); }
  }
  size_t size = final_list.size();
  if( size <= 0 ) return;
  
  searched_.clear();
  std::cout << "updated + edited block size: " << final_list.size() << std::endl;
  std::cout << "After search3: " << frontiers_.size() << std::endl;

  // Search new frontier within box slightly inflated from updated box
  for (const BlockIndex& block_index : final_list) {

    voxblox::Block<EsdfVoxel>::Ptr esdf_block = esdf_layer_->getBlockPtrByIndex(block_index);
    if (!esdf_block) {
      ROS_WARN("Getting wrong block index");
      continue;
    }

    for (ssize_t lin_index = num_voxels_per_block_ - 1; lin_index >= 0u; lin_index-- ) {
      VoxelIndex local_idx = esdf_block->computeVoxelIndexFromLinearIndex( lin_index );
      GlobalIndex global_idx = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex( block_index, local_idx, voxels_per_side_ );

      int linear_index = global_to_linearidx_in_List( global_idx, boundaries_.box_min_ );

      if( searched_.find(linear_index) != searched_.end() ) 
        { continue; }
      
      if( !isInBox(global_idx, boundaries_.box_min_, boundaries_.box_max_) 
          || ganzfree_bit_->test(linear_index) 
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
          ganzfree_bit_->set(linear_index);
          searched_.insert( linear_index );
        }
      }
    }
  }

  std::cout << "After search2: " << frontiers_.size() << std::endl;

  splitLargeFrontiers(tmp_frontiers_);
  visu_flag_ = true;

  std::cout << "After search1: " << frontiers_.size() << std::endl;
}


void FrontierMap::computeFrontiersToVisit() 
{
  first_new_ftr_ = frontiers_.end();
  std::cout << "tmp_frontiers_ size: " << tmp_frontiers_.size() << std::endl;
  if( tmp_frontiers_.size() <= 0 ) return;
  
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

  // frontiers_.splice(frontiers_.end(), tmp_frontiers_);

  // Reset indices of frontiers
  int idx = 0;
  for (auto& ft : frontiers_) {
    ft.id_ = idx++;
  }

  // cost_update_ = true;
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


void FrontierMap::updateFrontierCostMatrix() 
{
  // if( !cost_update_ ) return;

  std::cout << "frontiers_ size:" << frontiers_.size() << std::endl;;
  std::cout << "cost mat size before remove: " << std::endl;
  for (auto ftr : frontiers_)
    std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
  std::cout << "" << std::endl;

  std::cout << "removed_ids_:" << removed_ids_.size() << std::endl;

  std::cout << "cost mat size remove: " << std::endl;
  if (!removed_ids_.empty()) {
    // Delete path and cost for removed clusters
    for (auto it = frontiers_.begin(); it != first_new_ftr_; ++it) {
      auto cost_iter = it->costs_.begin();
      auto path_iter = it->paths_.begin();
      int iter_idx = 0;
      for (int i = 0; i < removed_ids_.size(); ++i) {
        // Step iterator to the item to be removed
        while (iter_idx < removed_ids_[i]) {
          ++cost_iter;
          ++path_iter;
          ++iter_idx;
        }
        cost_iter = it->costs_.erase(cost_iter);
        path_iter = it->paths_.erase(path_iter);
      }
      std::cout << "(" << it->costs_.size() << "," << it->paths_.size() << "), ";
    }
    removed_ids_.clear();
  }
  std::cout << "" << std::endl;

  auto updateCost = [](const list<Frontier>::iterator& it1, const list<Frontier>::iterator& it2) {
    std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
    // Search path from old cluster's top viewpoint to new cluster'
    Viewpoint& vui = it1->viewpoints_.front();
    Viewpoint& vuj = it2->viewpoints_.front();
    vector<Vector3f> path_ij;
    float cost_ij = ViewNode::computeCost( vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3f(0, 0, 0), 0, path_ij );
    // float cost_ij = 1.0;
    // Insert item for both old and new clusters
    it1->costs_.push_back(cost_ij);
    it1->paths_.push_back(path_ij);
    reverse(path_ij.begin(), path_ij.end());
    it2->costs_.push_back(cost_ij);
    it2->paths_.push_back(path_ij);
  };

  std::cout << "cost mat add: " << std::endl;
  if (first_new_ftr_ == frontiers_.end()) {
    return;
  }
  // Compute path and cost between old and new clusters
  for (auto it1 = frontiers_.begin(); it1 != first_new_ftr_; ++it1)
    for (auto it2 = first_new_ftr_; it2 != frontiers_.end(); ++it2)
      updateCost(it1, it2);

  // Compute path and cost between new clusters
  for (auto it1 = first_new_ftr_; it1 != frontiers_.end(); ++it1)
    for (auto it2 = it1; it2 != frontiers_.end(); ++it2) 
    {
      if (it1 == it2) {
        it1->costs_.push_back(0);
        it1->paths_.push_back({});
      } else
        updateCost(it1, it2);
    }
  std::cout << "" << std::endl;
  std::cout << "cost mat size final: " << std::endl;
  for (auto ftr : frontiers_)
    std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
  std::cout << "" << std::endl;

  // cost_update_ = false;
}


void FrontierMap::getFullCostMatrix( const Eigen::Vector3f& cur_pos, const Eigen::Vector3f& cur_vel, 
                                     const Eigen::Vector3f cur_yaw, Eigen::MatrixXf& mat ) 
{
  // Use Asymmetric TSP
  int dimen = frontiers_.size();
  mat.resize(dimen + 1, dimen + 1);

  // Fill block for clusters
  int i = 1, j = 1;
  for (auto ftr : frontiers_) {
    
    for (auto cs : ftr.costs_) {
      if (i < mat.rows() && j < mat.cols()) {
        mat(i, j++) = cs;
      } else {
        // 错误处理：索引超出范围
        std::cerr << "Error: Index out of bounds while filling cluster block." << std::endl;
        return;
      }
    }
    ++i;
    j = 1; // 重置列索引
  }

  // Fill block from current state to clusters
  
  if (mat.cols() > 1) {
    mat.col(0).setZero(); // 初始化第一列为零
  }

  j = 1;  
  for (auto ftr : frontiers_) {
    if (j >= mat.cols()) {
      std::cerr << "Error: Index out of bounds while filling current state to clusters block." << std::endl;
      return;
    }
    Viewpoint vj = ftr.viewpoints_.front();
    vector<Vector3f> path;
    mat(0, j++) = ViewNode::computeCost( cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path );
  }
}


/// @brief 循环遍历frontier_ids，将每个frontier的路径段添加到最终路径中
/// @param pos 
/// @param frontier_ids 
/// @param path 
void FrontierMap::getPathForTour( const Eigen::Vector3f& pos, const std::vector<int>& frontier_ids, std::vector<Eigen::Vector3f>& path ) 
{
  // Make an frontier_indexer to access the frontier list easier
  vector<list<Frontier>::iterator> frontier_indexer;
  for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it)
    frontier_indexer.push_back(it);

  // Compute the path from current pos to the first frontier
  vector<Vector3f> segment;
  ViewNode::searchPath(pos, frontier_indexer[frontier_ids[0]]->viewpoints_.front().pos_, segment);
  path.insert(path.end(), segment.begin(), segment.end());

  // Get paths of tour passing all clusters
  for (int i = 0; i < frontier_ids.size() - 1; ++i) {
    // Move to path to next cluster
    auto path_iter = frontier_indexer[frontier_ids[i]]->paths_.begin();
    int next_idx = frontier_ids[i + 1];
    for (int j = 0; j < next_idx; ++j)
      ++path_iter;
    path.insert(path.end(), path_iter->begin(), path_iter->end());
  }
}


void FrontierMap::visualizeFrontiers()
{
  if( visu_flag_ ) {
    std::vector<std::vector<Eigen::Vector3f>> clusters; 
    getFrontiers( clusters );
    if( !clusters.empty() ) {
      for (int i = 0; i < clusters.size(); ++i ) {
        ftr_visu_->drawCubes( clusters[i], 0.1, float(i) / clusters.size(), 0.4, "frontier", i );
      }
    }
    visu_flag_ = false;
  }
}


bool FrontierMap::whitelist( const Eigen::Vector3i &idx )
{
  int linear_index = (idx[0] - boundaries_.box_min_[0]) * boundaries_.map_voxel_num_(1) * boundaries_.map_voxel_num_(2) + 
                      (idx[1] - boundaries_.box_min_[1]) * boundaries_.map_voxel_num_(2) + (idx[2] - boundaries_.box_min_[2]);
  if( !ganzfree_bit_->test(linear_index) ) {
    return false;
  }
  return true;
}


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
      int neighbor_lin_idx = global_to_linearidx_in_List(neighbor_index, boundaries_.box_min_);

      if( searched_.find(neighbor_lin_idx) != searched_.end() ) {
        { continue; }
      } else if ( frontier_flag_.find(neighbor_lin_idx) != frontier_flag_.end() 
                  || ganzfree_bit_->test(neighbor_lin_idx)
                  || !voxblox_map_->knownfree(neighbor_index) 
                  || !isInBox(neighbor_index, boundaries_.box_min_, boundaries_.box_max_) ) 
        { searched_.insert( neighbor_lin_idx ); 
          continue; }         
      else if ( !voxblox_map_->isNeighborUnknown(neighbor_index) ) {
        ganzfree_bit_->set(neighbor_lin_idx);
        searched_.insert( neighbor_lin_idx ); 
        continue;
      }
      //   !(knownfree(neighbor_index) && isNeighborUnknown(neighbor_index)) 
      //             || !isInBox(neighbor_index, box_min_, box_max_) ) { 
      //   // ganzfree_bit_->set(neighbor_lin_idx);
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

  ftr.block_idx_.push_back(block_index);

  BlockIndex front_block_index = voxblox_map_->getBlockIndexFromGlobalVoxelIndex( ftr.idx_.front() );
  BlockIndex back_block_index = voxblox_map_->getBlockIndexFromGlobalVoxelIndex( ftr.idx_.back() );
  if( front_block_index != ftr.block_idx_.back() ) 
    ftr.block_idx_.push_back(front_block_index);

  if( back_block_index != front_block_index && back_block_index != block_index ) 
    ftr.block_idx_.push_back(back_block_index);

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
  Eigen::Vector3d frontier_avg = frontier.average_.cast<double>();
  frontier_avg.z() = 1.0;

  for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    for ( double rc = candidate_min; rc <= candidate_max + 1e-3; rc += candidate_diff )
    {   
      const Vector3d sample_pos = frontier_avg + rc * Vector3d(cos_phi, sin_phi, 0);

      if( voxblox_map_->checkVoxelDistance(sample_pos) != VoxbloxMap::VoxelStatus::kFree 
          || voxblox_map_->isNearUnknown(sample_pos) )
        { continue; }

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
      // std::cout<< "visible Cells: " << visib_num << std::endl;
      if (visib_num >= min_visib_num) {
        Viewpoint vp = { sample_pos_f, avg_yaw, visib_num };
        frontier.viewpoints_.push_back(vp);
      }
    }
  }
}


/// @brief 
/// @param yaw 
void FrontierMap::wrapYaw(float& yaw) {
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  while (yaw > M_PI)
    yaw -= 2 * M_PI;
}


bool FrontierMap::ifBlockinFOV( Eigen::Vector3f &odom_pos, float &odom_yaw, voxblox::BlockIndex &goal_idx )
{
  float length = voxel_size_ * voxels_per_side_;
  voxblox::Block<EsdfVoxel>::Ptr esdf_block = esdf_layer_->getBlockPtrByIndex(goal_idx);
  
  if( !esdf_block ) 
    return false;
  
  // voxblox::Point goal_origin = esdf_block->origin();
  // voxblox::Point larger_x = goal_origin;
  // voxblox::Point larger_y = goal_origin;
  // voxblox::Point larger_xy = goal_origin;
  voxblox::Point goal_origin = esdf_block->origin();
  voxblox::Point corners[4] = {
    goal_origin,
    goal_origin + voxblox::Point(length, 0, 0),
    goal_origin + voxblox::Point(0, length, 0),
    goal_origin + voxblox::Point(length, length, 0)
  };
  // larger_x.x() += length;
  // larger_y.y() += length;
  // larger_xy.x() += length;
  // larger_xy.y() += length;
  float theta1 = odom_yaw + half_theta_;
  float theta2 = odom_yaw - half_theta_;
  normalizeYaw( theta1 );
  normalizeYaw( theta2 );

  // if( ifPointinFOV( odom_pos, theta1, theta2, goal_origin ) ||
  //     ifPointinFOV( odom_pos, theta1, theta2, goal_origin ) ||
  //     ifPointinFOV( odom_pos, theta1, theta2, goal_origin ) ||
  //     ifPointinFOV( odom_pos, theta1, theta2, goal_origin ) )
  //   { return true; }

  for (const auto& corner : corners) {
    if (ifPointinFOV(odom_pos, theta1, theta2, corner)) {
      return true;
    }
  }

  return false;
}


bool FrontierMap::ifPointinFOV( Eigen::Vector3f &odom_pos, float &start_theta, float &end_theta, const voxblox::Point &point )
{
  // if( (point - odom_pos).norm() > 3.0f )
  //   return false;

  // 只计算 x 和 y 方向上的距离
  float distance_xy = std::sqrt(std::pow(point.x() - odom_pos.x(), 2) + std::pow(point.y() - odom_pos.y(), 2));
  if (distance_xy > 3.0f) {
    return false;
  }
  
  float point_angle = std::atan2(point.y() - odom_pos.y(), point.x() - odom_pos.x());
  normalizeYaw( point_angle );

  // 确保扇形角度范围是顺时针方向
  if (start_theta > end_theta) {
    return point_angle <= start_theta && point_angle >= end_theta;
  } else {
    return point_angle <= start_theta || point_angle >= end_theta;
  }
}


void FrontierMap::filterBlocksWithZEqualToZero( voxblox::BlockIndexList &updated_blocks )
{
  voxblox::BlockIndexList filtered_blocks;
  std::copy_if(updated_blocks_.begin(), updated_blocks_.end(), std::back_inserter(filtered_blocks), [](const voxblox::BlockIndex& index) {
      return index.z() == 0;
  });
  updated_blocks_ = filtered_blocks; 
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
  Eigen::Vector3f pos_tmp;
  // voxblox::FloatingPoint esdf_max_distance_tmp = esdf_max_distance_m_;
  // float distance_thres_tmp = distance_thres_;

  for (auto cell : cluster) {
    // Check if frontier cell is inside FOV
    if (!percep_utils_->insideFOV(cell)) continue;

    // Check if frontier cell is visible (not occulded by obstacles)
    raycaster_->input(cell, pos);
    bool visib = true;
    

    // while (raycaster_->nextId(idx)) {
    //   if( !whitelist(idx) ) {
    //     visib = false;
    //     break;
    //   }
    // }
    while (raycaster_->nextPos(pos_tmp)) {
      Eigen::Vector3d pos_d = pos_tmp.cast<double>();
      if( voxblox_map_->checkVoxelDistance(pos_d) != VoxbloxMap::VoxelStatus::kFree ) {
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

}  // namespace scoutair_planner