// #include <active_perception/frontier_finder.h>
// #include <plan_env/sdf_map.h>
// #include <plan_env/raycast.h>
// // #include <path_searching/astar2.h>

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <plan_env/edt_environment.h>
// #include <active_perception/perception_utils.h>
// #include <active_perception/graph_node.h>

// // use PCL region growing segmentation
// // #include <pcl/point_types.h>
// // #include <pcl/search/search.h>
// // #include <pcl/search/kdtree.h>
// // #include <pcl/features/normal_3d.h>
// // #include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>

// #include <Eigen/Eigenvalues>

#include "scoutair_planner/frontier_finder.h"

namespace scoutair_planner {

using namespace Eigen;
using namespace std;

FrontierFinder::FrontierFinder( ros::NodeHandle& nh, ros::NodeHandle& nh_private ) 
  : esdf_server_( nh, nh_private )
{
  esdf_layer_ = getESDFLayer();
  CHECK_NOTNULL(esdf_layer_);

  voxel_size_ = esdf_layer_->voxel_size();
  resolution_ = static_cast<double>(voxel_size_);
  constexpr double kVoxelSizeEpsilon = 1e-5;
  if (voxel_size_ <= 0.0f ) {
    ROS_ERROR( "Invalid voxel size: %f. It must be greater than zero.", voxel_size_ );
  } else if ( std::abs(resolution_ - voxel_size_) > kVoxelSizeEpsilon ) {
    ROS_ERROR( "Invalid static_cast. Voxel size: %f, Resolution: %f", voxel_size_, resolution_ );
  }

  nh_private.param( "esdf/esdf_max_distance_m", esdf_max_distance_m_, esdf_max_distance_m_ );

  double x_size, y_size, z_size, ground_height;
  nh.param("frontierfinder/map_size_x", x_size, -1.0);
  nh.param("frontierfinder/map_size_y", y_size, -1.0);
  nh.param("frontierfinder/map_size_z", z_size, -1.0);
  nh.param("frontierfinder/ground_height", ground_height, 1.0);
  Eigen::Vector3d map_size = Eigen::Vector3d(x_size, y_size, z_size);
  Eigen::Vector3i map_voxel_num_;
  for (int i = 0; i < 3; ++i)
    map_voxel_num_(i) = ceil( map_size(i) / voxel_size_ );

  int voxel_num = map_voxel_num_[0] * map_voxel_num_[1] * map_voxel_num_[2];

  frontier_flag_ = std::vector<char>( voxel_num, 0 );
  fill( frontier_flag_.begin(), frontier_flag_.end(), 0 );

  nh.param("frontier/cluster_min", cluster_min_, -1);
  nh.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
  nh.param("frontier/cluster_size_z", cluster_size_z_, -1.0);
  nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
  nh.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
  nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
  nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
  nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
  nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
  nh.param("frontier/down_sample", down_sample_, -1);
  nh.param("frontier/min_visib_num", min_visib_num_, -1);
  nh.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);

  nh_private.param("frontier/occupancy_distance_voxelsize_factor", occupancy_distance_voxelsize_factor_, 1.0F );

  raycaster_.reset(new RayCaster);

  Eigen::Vector3d map_origin = Eigen::Vector3d( -x_size / 2.0, -y_size / 2.0, ground_height );
  raycaster_->setParams( resolution_, map_origin );

  percep_utils_.reset(new PerceptionUtils(nh));
}


FrontierFinder::~FrontierFinder() {
}


void FrontierFinder::searchFrontiers() {
  ros::Time t1 = ros::Time::now();
  tmp_frontiers_.clear();

  // Bounding box of updated region
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, true);

  // Removed changed frontiers in updated map
  auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& frontiers) {
    Eigen::Vector3i idx;
    for (auto cell : iter->cells_) {
      edt_env_->sdf_map_->posToIndex(cell, idx);
      frontier_flag_[toadr(idx)] = 0;
    }
    iter = frontiers.erase(iter);
  };

  std::cout << "Before remove: " << frontiers_.size() << std::endl;

  removed_ids_.clear();
  int rmv_idx = 0;
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter)) {
      resetFlag(iter, frontiers_);
      removed_ids_.push_back(rmv_idx);
    } else {
      ++rmv_idx;
      ++iter;
    }
  }
  std::cout << "After remove: " << frontiers_.size() << std::endl;
  for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter))
      resetFlag(iter, dormant_frontiers_);
    else
      ++iter;
  }

  // Search new frontier within box slightly inflated from updated box
  Vector3d search_min = update_min - Vector3d(1, 1, 0.5);
  Vector3d search_max = update_max + Vector3d(1, 1, 0.5);
  Vector3d box_min, box_max;
  edt_env_->sdf_map_->getBox(box_min, box_max);
  for (int k = 0; k < 3; ++k) {
    search_min[k] = max(search_min[k], box_min[k]);
    search_max[k] = min(search_max[k], box_max[k]);
  }
  Eigen::Vector3i min_id, max_id;
  edt_env_->sdf_map_->posToIndex(search_min, min_id);
  edt_env_->sdf_map_->posToIndex(search_max, max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        // Scanning the updated region to find seeds of frontiers
        Eigen::Vector3i cur(x, y, z);
        if (frontier_flag_[toadr(cur)] == 0 && knownfree(cur) && isNeighborUnknown(cur)) {
          // Expand from the seed cell to find a complete frontier cluster
          expandFrontier(cur);
        }
      }
  splitLargeFrontiers(tmp_frontiers_);

  ROS_WARN_THROTTLE(5.0, "Frontier t: %lf", (ros::Time::now() - t1).toSec());
}


void FrontierFinder::expandFrontier(
    const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */) {
  // std::cout << "depth: " << depth << std::endl;
  auto t1 = ros::Time::now();

  // Data for clustering
  queue<Eigen::Vector3i> cell_queue;
  std::vector<Eigen::Vector3d> expanded;
  Vector3d pos;

  edt_env_->sdf_map_->indexToPos(first, pos);
  expanded.push_back(pos);
  cell_queue.push(first);
  frontier_flag_[toadr(first)] = 1;

  // Search frontier cluster based on region growing (distance clustering)
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);
    for (auto nbr : nbrs) {
      // Qualified cell should be inside bounding box and frontier cell not clustered
      int adr = toadr(nbr);
      if (frontier_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr) ||
          !(knownfree(nbr) && isNeighborUnknown(nbr)))
        continue;

      edt_env_->sdf_map_->indexToPos(nbr, pos);
      if (pos[2] < 0.4) continue;  // Remove noise close to ground
      expanded.push_back(pos);
      cell_queue.push(nbr);
      frontier_flag_[adr] = 1;
    }
  }
  if (expanded.size() > cluster_min_) {
    // Compute detailed info
    Frontier frontier;
    frontier.cells_ = expanded;
    computeFrontierInfo(frontier);
    tmp_frontiers_.push_back(frontier);
  }
}


/// @brief 
/// @param frontiers 
void FrontierFinder::splitLargeFrontiers( list<Frontier>& frontiers ) 
{
  list<Frontier> splits, tmps;
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) {
    // Check if each frontier needs to be split horizontally
    if (splitHorizontally(*it, splits)) {
      tmps.insert(tmps.end(), splits.begin(), splits.end());
      splits.clear();
    } else
      tmps.push_back(*it);
  }
  frontiers = tmps;
}


bool FrontierFinder::splitHorizontally( const Frontier& frontier, list<Frontier>& splits ) 
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
  Eigen::Matrix2d cov;
  cov.setZero();
  for (auto cell : frontier.filtered_cells_) {
    Eigen::Vector2d diff = cell.head<2>() - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(frontier.filtered_cells_.size());

  // Find eigenvector corresponds to maximal eigenvector
  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector2d first_pc = vectors.col(max_idx);
  std::cout << "max idx: " << max_idx << std::endl;
  std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() << std::endl;

  // Split the frontier into two groups along the first PC
  Frontier ftr1, ftr2;
  for (auto cell : frontier.cells_) {
    if ((cell.head<2>() - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
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


/// @brief 检查给定的全局体素索引是否位于任何给定的box内
/// @param boxes 
/// @param global_voxel_index 
/// @return 
bool FrontierFinder::isInBoxes( const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes, 
                                const voxblox::GlobalIndex &global_voxel_index ) 
{
  voxblox::Point point = voxblox::getCenterPointFromGridIndex( global_voxel_index, voxel_size_ );
  Vector3d pt = point.cast<double>();

  for (const auto& box : boxes) {
    // Check if contained by a box
    const auto& min_corner = box.first;
    const auto& max_corner = box.second;
    bool inbox = true;
    for (int i = 0; i < 3; ++i) {
      inbox = inbox && pt[i] > min_corner[i] && pt[i] < max_corner[i];
      if (!inbox) break;
    }
    if (inbox) return true;
  }
  return false;
}

void FrontierFinder::updateFrontierCostMatrix() {
  std::cout << "cost mat size before remove: " << std::endl;
  for (auto ftr : frontiers_)
    std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
  std::cout << "" << std::endl;

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
    std::vector<Vector3d> path_ij;
    double cost_ij = ViewNode::computeCost(
        vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3d(0, 0, 0), 0, path_ij);
    // Insert item for both old and new clusters
    it1->costs_.push_back(cost_ij);
    it1->paths_.push_back(path_ij);
    reverse(path_ij.begin(), path_ij.end());
    it2->costs_.push_back(cost_ij);
    it2->paths_.push_back(path_ij);
  };

  std::cout << "cost mat add: " << std::endl;
  // Compute path and cost between old and new clusters
  for (auto it1 = frontiers_.begin(); it1 != first_new_ftr_; ++it1)
    for (auto it2 = first_new_ftr_; it2 != frontiers_.end(); ++it2)
      updateCost(it1, it2);

  // Compute path and cost between new clusters
  for (auto it1 = first_new_ftr_; it1 != frontiers_.end(); ++it1)
    for (auto it2 = it1; it2 != frontiers_.end(); ++it2) {
      if (it1 == it2) {
        std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
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
}

void FrontierFinder::mergeFrontiers(Frontier& ftr1, const Frontier& ftr2) {
  // Merge ftr2 into ftr1
  ftr1.average_ =
      (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
      (double(ftr1.cells_.size() + ftr2.cells_.size()));
  ftr1.cells_.insert(ftr1.cells_.end(), ftr2.cells_.begin(), ftr2.cells_.end());
  computeFrontierInfo(ftr1);
}

bool FrontierFinder::canBeMerged(const Frontier& ftr1, const Frontier& ftr2) {
  Vector3d merged_avg =
      (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
      (double(ftr1.cells_.size() + ftr2.cells_.size()));
  // Check if it can merge two frontier without exceeding size limit
  for (auto c1 : ftr1.cells_) {
    auto diff = c1 - merged_avg;
    if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_) return false;
  }
  for (auto c2 : ftr2.cells_) {
    auto diff = c2 - merged_avg;
    if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_) return false;
  }
  return true;
}


/// @brief 
/// @param index_min1 
/// @param index_max1 
/// @param index_min2 
/// @param index_max2 
/// @return 
bool FrontierFinder::haveOverlap( const voxblox::GlobalIndex& index_min1, const voxblox::GlobalIndex& index_max1,
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
bool FrontierFinder::isFrontierChanged( const Frontier& ft ) 
{
  for (auto idx_cell : ft.idx_cells ) {
    if (!(knownfree(idx_cell) && isNeighborUnknown(idx_cell))) return true;
  }
  return false;
}


/// @brief 
/// @param ftr 
void FrontierFinder::computeFrontierInfo( Frontier& ftr ) 
{
  // Compute average position and bounding box of cluster
  ftr.average_.setZero();
  ftr.idx_box_max_ = ftr.idx_cells.front();
  ftr.idx_box_min_ = ftr.idx_cells.front();
  ftr.box_max_ = ftr.cells_.front();
  ftr.box_min_ = ftr.cells_.front();
  for (auto& idx : ftr.idx_cells )
  {
    for (int i = 0; i < 3; ++i) {
      ftr.idx_box_min_[i] = min(ftr.idx_box_min_[i], idx[i]);
      ftr.idx_box_max_[i] = max(ftr.idx_box_max_[i], idx[i]);
    }
  }
  
  for (auto cell : ftr.cells_) {
    ftr.average_ += cell;
    for (int i = 0; i < 3; ++i) {
      ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
      ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
    }
  }
  ftr.average_ /= double(ftr.cells_.size());

  // Compute downsampled cluster
  downsample(ftr.cells_, ftr.filtered_cells_);
}

void FrontierFinder::computeFrontiersToVisit() {
  first_new_ftr_ = frontiers_.end();
  int new_num = 0;
  int new_dormant_num = 0;
  // Try find viewpoints for each cluster and categorize them according to viewpoint number
  for (auto& tmp_ftr : tmp_frontiers_) {
    // Search viewpoints around frontier
    sampleViewpoints(tmp_ftr);
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
      ++new_dormant_num;
    }
  }
  // Reset indices of frontiers
  int idx = 0;
  for (auto& ft : frontiers_) {
    ft.id_ = idx++;
    std::cout << ft.id_ << ", ";
  }
  std::cout << "\nnew num: " << new_num << ", new dormant: " << new_dormant_num << std::endl;
  std::cout << "to visit: " << frontiers_.size() << ", dormant: " << dormant_frontiers_.size()
            << std::endl;
}

void FrontierFinder::getTopViewpointsInfo(
    const Vector3d& cur_pos, std::vector<Eigen::Vector3d>& points, std::vector<double>& yaws,
    std::vector<Eigen::Vector3d>& averages) {
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


void FrontierFinder::getViewpointsInfo( const Eigen::Vector3d& cur_pos, const std::vector<int>& ids, const int& view_num,
                                        const double& max_decay, std::vector<std::vector<Eigen::Vector3d>>& points,
                                        std::vector<std::vector<double>>& yaws ) 
{
  points.clear();
  yaws.clear();
  for (auto id : ids) {
    // Scan all frontiers to find one with the same id
    for (auto frontier : frontiers_) {
      if (frontier.id_ == id) {
        // Get several top viewpoints that are far enough
        std::vector<Eigen::Vector3d> pts;
        std::vector<double> ys;
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


void FrontierFinder::getFrontiers(std::vector<std::vector<Eigen::Vector3d>>& clusters) {
  clusters.clear();
  for (auto frontier : frontiers_)
    clusters.push_back(frontier.cells_);
  // clusters.push_back(frontier.filtered_cells_);
}

void FrontierFinder::getDormantFrontiers(std::vector<std::vector<Vector3d>>& clusters) {
  clusters.clear();
  for (auto ft : dormant_frontiers_)
    clusters.push_back(ft.cells_);
}

void FrontierFinder::getFrontierBoxes(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes) {
  boxes.clear();
  for (auto frontier : frontiers_) {
    Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
    Eigen::Vector3d scale = frontier.box_max_ - frontier.box_min_;
    boxes.push_back(make_pair(center, scale));
  }
}

void FrontierFinder::getPathForTour(
    const Eigen::Vector3d& pos, const std::vector<int>& frontier_ids, std::vector<Eigen::Vector3d>& path) {
  // Make an frontier_indexer to access the frontier list easier
  std::vector<list<Frontier>::iterator> frontier_indexer;
  for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it)
    frontier_indexer.push_back(it);

  // Compute the path from current pos to the first frontier
  std::vector<Vector3d> segment;
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

void FrontierFinder::getFullCostMatrix(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
    Eigen::MatrixXd& mat) {
  if (false) {
    // Use symmetric TSP formulation
    int dim = frontiers_.size() + 2;
    mat.resize(dim, dim);  // current pose (0), sites, and virtual depot finally

    int i = 1, j = 1;
    for (auto ftr : frontiers_) {
      for (auto cs : ftr.costs_)
        mat(i, j++) = cs;
      ++i;
      j = 1;
    }

    // Costs from current pose to sites
    for (auto ftr : frontiers_) {
      Viewpoint vj = ftr.viewpoints_.front();
      std::vector<Vector3d> path;
      mat(0, j) = mat(j, 0) =
          ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
      ++j;
    }
    // Costs from depot to sites, the same large vaule
    for (j = 1; j < dim - 1; ++j) {
      mat(dim - 1, j) = mat(j, dim - 1) = 100;
    }
    // Zero cost to depot to ensure connection
    mat(0, dim - 1) = mat(dim - 1, 0) = -10000;

  } else {
    // Use Asymmetric TSP
    int dimen = frontiers_.size();
    mat.resize(dimen + 1, dimen + 1);
    // std::cout << "mat size: " << mat.rows() << ", " << mat.cols() << std::endl;
    // Fill block for clusters
    int i = 1, j = 1;
    for (auto ftr : frontiers_) {
      for (auto cs : ftr.costs_) {
        // std::cout << "(" << i << ", " << j << ")"
        // << ", ";
        mat(i, j++) = cs;
      }
      ++i;
      j = 1;
    }
    // std::cout << "" << std::endl;

    // Fill block from current state to clusters
    mat.leftCols<1>().setZero();
    for (auto ftr : frontiers_) {
      // std::cout << "(0, " << j << ")"
      // << ", ";
      Viewpoint vj = ftr.viewpoints_.front();
      std::vector<Vector3d> path;
      mat(0, j++) =
          ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
    }
    // std::cout << "" << std::endl;
  }
}

void FrontierFinder::findViewpoints(
    const Vector3d& sample, const Vector3d& ftr_avg, std::vector<Viewpoint>& vps) {
  if (!edt_env_->sdf_map_->isInBox(sample) ||
      edt_env_->sdf_map_->getInflateOccupancy(sample) == 1 || isNearUnknown(sample))
    return;

  double left_angle_, right_angle_, vertical_angle_, ray_length_;

  // Central yaw is determined by frontier's average position and sample
  auto dir = ftr_avg - sample;
  double hc = atan2(dir[1], dir[0]);

  std::vector<int> slice_gains;
  // Evaluate info gain of different slices
  for (double phi_h = -M_PI_2; phi_h <= M_PI_2 + 1e-3; phi_h += M_PI / 18) {
    // Compute gain of one slice
    int gain = 0;
    for (double phi_v = -vertical_angle_; phi_v <= vertical_angle_; phi_v += vertical_angle_ / 3) {
      // Find endpoint of a ray
      Vector3d end;
      end[0] = sample[0] + ray_length_ * cos(phi_v) * cos(hc + phi_h);
      end[1] = sample[1] + ray_length_ * cos(phi_v) * sin(hc + phi_h);
      end[2] = sample[2] + ray_length_ * sin(phi_v);

      // Do raycasting to check info gain
      Vector3i idx;
      raycaster_->input(sample, end);
      while (raycaster_->nextId(idx)) {
        // Hit obstacle, stop the ray
        if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || !edt_env_->sdf_map_->isInBox(idx))
          break;
        // Count number of unknown cells
        if (edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) ++gain;
      }
    }
    slice_gains.push_back(gain);
  }

  // Sum up slices' gain to get different yaw's gain
  std::vector<std::pair<double, int>> yaw_gains;
  for (int i = 0; i < 6; ++i)  // [-90,-10]-> [10,90], delta_yaw = 20, 6 groups
  {
    double yaw = hc - M_PI_2 + M_PI / 9.0 * i + right_angle_;
    int gain = 0;
    for (int j = 2 * i; j < 2 * i + 9; ++j)  // 80 degree hFOV, 9 slices
      gain += slice_gains[j];
    yaw_gains.push_back(make_pair(yaw, gain));
  }

  // Get several yaws with highest gain
  vps.clear();
  sort(
      yaw_gains.begin(), yaw_gains.end(),
      [](const std::pair<double, int>& p1, const std::pair<double, int>& p2) {
        return p1.second > p2.second;
      });
  for (int i = 0; i < 3; ++i) {
    if (yaw_gains[i].second < min_visib_num_) break;
    Viewpoint vp = { sample, yaw_gains[i].first, yaw_gains[i].second };
    while (vp.yaw_ < -M_PI)
      vp.yaw_ += 2 * M_PI;
    while (vp.yaw_ > M_PI)
      vp.yaw_ -= 2 * M_PI;
    vps.push_back(vp);
  }
}


// Sample viewpoints around frontier's average position, check coverage to the frontier cells
void FrontierFinder::sampleViewpoints( Frontier& frontier ) 
{
  // Evaluate sample viewpoints on circles, find ones that cover most cells
  for( double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
       rc <= candidate_rmax_ + 1e-3; rc += dr ) 
  {
    for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
      const Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);

      float voxel_size_inv = 1.0 / voxel_size_;

      voxblox::GlobalIndex center_voxel_index = voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>( sample_pos.cast<voxblox::FloatingPoint>(), voxel_size_inv );

      // Qualified viewpoint is in bounding box and in safe region
      if (!edt_env_->sdf_map_->isInBox(sample_pos) ||
          edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 || isNearUnknown(sample_pos))
        continue;

      // Compute average yaw
      auto& cells = frontier.filtered_cells_;
      Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
      double avg_yaw = 0.0;
      for (int i = 1; i < cells.size(); ++i) {
        Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
        double yaw = acos(dir.dot(ref_dir));
        if (ref_dir.cross(dir)[2] < 0) yaw = -yaw;
        avg_yaw += yaw;
      }
      avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
      wrapYaw(avg_yaw);
      // Compute the fraction of covered and visible cells
      int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
      if (visib_num > min_visib_num_) {
        Viewpoint vp = { sample_pos, avg_yaw, visib_num };
        frontier.viewpoints_.push_back(vp);
        // int gain = findMaxGainYaw(sample_pos, frontier, sample_yaw);
      }
      // }
    }
  }
}


/// @brief 
/// @return 
bool FrontierFinder::isFrontierCovered() {
  voxblox::GlobalIndex update_min, update_max;
  map_manager_.getUpdatedBoxIndex( update_min, update_max );

  auto checkChanges = [&](const list<Frontier>& frontiers) {
    for (auto ftr : frontiers) {
      if (!haveOverlap(ftr.idx_box_min_, ftr.idx_box_max_, update_min, update_max)) continue;
      const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
      int change_num = 0;
      for (auto idx : ftr.idx_cells) {
        if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh)
          return true;
      }
    }
    return false;
  };

  if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_)) return true;

  return false;
}


/// @brief 
/// @param global_voxel_index 
/// @return 
bool FrontierFinder::isNearUnknown( const voxblox::GlobalIndex &global_voxel_index ) 
{
  const int vox_num = floor(min_candidate_clearance_ / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -1; z <= 1; ++z) {
        voxblox::GlobalIndex new_index;
        new_index << global_voxel_index[0] + x, global_voxel_index[1] + y, global_voxel_index[2] + z;
        voxblox::EsdfVoxel* voxel = esdf_layer_->getVoxelPtrByGlobalIndex( new_index );
        if( checkVoxelStatus(voxel) == VoxelStatus::kUnknown ) return true;
      }
  return false;
}


/// @brief 
/// @param pos 
/// @param yaw 
/// @param cluster 
/// @return 
int FrontierFinder::countVisibleCells(
    const Eigen::Vector3d& pos, const double& yaw, const std::vector<Eigen::Vector3d>& cluster) {
  percep_utils_->setPose(pos, yaw);
  int visib_num = 0;
  Eigen::Vector3i idx;
  for (auto cell : cluster) {
    // Check if frontier cell is inside FOV
    if (!percep_utils_->insideFOV(cell)) continue;

    // Check if frontier cell is visible (not occulded by obstacles)
    raycaster_->input(cell, pos);
    bool visib = true;
    while (raycaster_->nextId(idx)) {
      voxblox::GlobalIndex voxblox_id = eigenVector3iToGlobalIndex( idx );
      voxblox::EsdfVoxel* voxel = esdf_layer_->getVoxelPtrByGlobalIndex( voxblox_id );
      if ( checkVoxelStatus(voxel) == VoxelStatus::kUnknown ) {
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
Eigen::Vector3i FrontierFinder::globalIndexToEigenVector3i( voxblox::GlobalIndex& global_index ) 
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
voxblox::GlobalIndex FrontierFinder::eigenVector3iToGlobalIndex( Eigen::Vector3i& idx ) 
{
  voxblox::GlobalIndex global_index;
  global_index[0] = static_cast<voxblox::LongIndexElement>(idx[0]);
  global_index[1] = static_cast<voxblox::LongIndexElement>(idx[1]);
  global_index[2] = static_cast<voxblox::LongIndexElement>(idx[2]);
  return global_index;
}


/// @brief 
/// @param cluster_in 
/// @param cluster_out 
void FrontierFinder::downsample(
    const std::vector<Eigen::Vector3d>& cluster_in, std::vector<Eigen::Vector3d>& cluster_out) {
  // downsamping cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto cell : cluster_in)
    cloud->points.emplace_back(cell[0], cell[1], cell[2]);

  const double leaf_size = voxel_size_ * down_sample_;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudf);

  cluster_out.clear();
  for (auto pt : cloudf->points)
    cluster_out.emplace_back(pt.x, pt.y, pt.z);
}


/// @brief 
/// @param yaw 
void FrontierFinder::wrapYaw(double& yaw) {
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  while (yaw > M_PI)
    yaw -= 2 * M_PI;
}

inline std::vector<Eigen::Vector3i> FrontierFinder::sixNeighbors(const Eigen::Vector3i& voxel) {
  std::vector<Eigen::Vector3i> neighbors(6);
  Eigen::Vector3i tmp;

  tmp = voxel - Eigen::Vector3i(1, 0, 0);
  neighbors[0] = tmp;
  tmp = voxel + Eigen::Vector3i(1, 0, 0);
  neighbors[1] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 1, 0);
  neighbors[2] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 1, 0);
  neighbors[3] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 0, 1);
  neighbors[4] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 0, 1);
  neighbors[5] = tmp;

  return neighbors;
}

inline std::vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i& voxel) {
  std::vector<Eigen::Vector3i> neighbors(10);
  Eigen::Vector3i tmp;
  int count = 0;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0) continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}

inline std::vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i& voxel) {
  std::vector<Eigen::Vector3i> neighbors(26);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0) continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}


/// @brief 
/// @param global_voxel_index 
/// @return 
inline bool FrontierFinder::isNeighborUnknown( const voxblox::GlobalIndex &global_voxel_index ) 
{
  // At least one neighbor is unknown
  voxblox::EsdfVoxel* voxel = esdf_layer_->getVoxelPtrByGlobalIndex( global_voxel_index );
  CHECK_NOTNULL(voxel);
  voxblox::Neighborhood<>::IndexMatrix neighbor_indices;
  voxblox::Neighborhood<>::getFromGlobalIndex(global_voxel_index, &neighbor_indices);

  for (unsigned int idx = 0u; idx < neighbor_indices.cols(); ++idx) {
    const voxblox::GlobalIndex& neighbor_index = neighbor_indices.col(idx);
    voxblox::EsdfVoxel* neighbor_voxel = esdf_layer_->getVoxelPtrByGlobalIndex(neighbor_index);
    if( checkVoxelStatus( neighbor_voxel ) == VoxelStatus::kUnknown ) 
      return true ;
  }
  return false;
}

inline int FrontierFinder::toadr(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->toAddress(idx);
}


/// @brief 
/// @param global_voxel_index 
/// @return 
inline bool FrontierFinder::knownfree( const voxblox::GlobalIndex &global_voxel_index ) 
{
  voxblox::EsdfVoxel* voxel = esdf_layer_->getVoxelPtrByGlobalIndex( global_voxel_index );
  return checkVoxelStatus(voxel) == VoxelStatus::kFree;
}

inline bool FrontierFinder::inmap(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->isInMap(idx);
}


/// @brief 
/// @param global_voxel_index 
/// @return 
inline bool FrontierFinder::validDataIdx( const voxblox::GlobalIndex &global_voxel_index ) {
  voxblox::EsdfVoxel* voxel = esdf_layer_->getVoxelPtrByGlobalIndex( global_voxel_index );
  CHECK_NOTNULL(voxel);

  // Skip voxels that are unobserved or outside the ranges we care about.
  if (!voxel->observed || voxel->distance >= esdf_max_distance_m_ ||
      voxel->distance <= -esdf_max_distance_m_ ) 
  { return true; } 
  else return false;
}


/// @brief 
/// @param voxel 
/// @return 
inline FrontierFinder::VoxelStatus FrontierFinder::checkVoxelStatus( const voxblox::EsdfVoxel* voxel ) const
{
  VoxelStatus current_status = VoxelStatus::kFree;
  const float distance_thres =
      occupancy_distance_voxelsize_factor_ * voxel_size_ + 1e-6;

  if( voxel == nullptr || !voxel->observed || 
       voxel->distance >= esdf_max_distance_m_ ||
       voxel->distance <= -esdf_max_distance_m_ ) 
  { return VoxelStatus::kUnknown; } 
  else if( voxel->distance <= distance_thres ) { return VoxelStatus::kOccupied; }
  return current_status;
}


}  // namespace scoutair_planner