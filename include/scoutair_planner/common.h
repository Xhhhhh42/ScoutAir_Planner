#ifndef PLANNER_COMMON_H_
#define PLANNER_COMMON_H_

#include <vector>
#include <Eigen/Eigen>
#include <unordered_set>
#include <string>
#include <list>
#include <functional>
#include <voxblox/core/common.h>

// // 为 GlobalIndex 提供 hash 函数
// #ifndef VOXBLOX_GLOBAL_INDEX_HASH_DEFINED
// #define VOXBLOX_GLOBAL_INDEX_HASH_DEFINED
// namespace std {
//   template <>
//   struct hash<voxblox::GlobalIndex> {
//     std::size_t operator()(const voxblox::GlobalIndex& k) const {
//       return std::hash<voxblox::LongIndexElement>()(k.x()) ^ std::hash<voxblox::LongIndexElement>()(k.y()) ^ std::hash<voxblox::LongIndexElement>()(k.z());
//     }
//   };
// }
// #endif // VOXBLOX_GLOBAL_INDEX_HASH_DEFINED


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
  // std::unordered_set<voxblox::BlockIndex> block_indices_;
  std::vector<voxblox::BlockIndex> block_idx_;
  // voxblox::BlockIndex main_block_index_;
  // voxblox::BlockIndex second_block_index_;

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

  // // 默认构造函数
  // Frontier()
  //   : main_block_index_(voxblox::BlockIndex::Zero()),
  //     second_block_index_(voxblox::BlockIndex::Zero()),
  //     average_(Eigen::Vector3f::Zero()),
  //     id_(0) {}
};


struct Box_boundaries {
  // Boundary--float
  Eigen::Vector3f box_minf_, box_maxf_;

  // GlobalIndex Boundary--int
  Eigen::Vector3i box_min_, box_max_; 

  // BlockIndex Boundary--int  
  Eigen::Vector3i block_min_i_, block_max_i_;

  Eigen::Vector3i map_voxel_num_;
};


struct ExplorationParam {
  // params
  bool refine_local_;
  int refined_num_;
  double refined_radius_;
  int top_view_num_;
  double max_decay_;
  std::string tsp_dir_;  // resource dir of tsp solver
  double relax_time_;
};

struct ExplorationData {
  std::vector<std::vector<Eigen::Vector3f>> frontiers_;
  std::vector<std::vector<Eigen::Vector3f>> dead_frontiers_;
  // std::vector<pair<Eigen::Vector3f, Eigen::Vector3f>> frontier_boxes_;
  std::vector<Eigen::Vector3f> points_;
  std::vector<Eigen::Vector3f> averages_;
  std::vector<Eigen::Vector3f> views_;
  std::vector<float> yaws_;
  std::vector<Eigen::Vector3f> global_tour_;

  std::vector<int> refined_ids_;
  std::vector<std::vector<Eigen::Vector3f>> n_points_;
  std::vector<Eigen::Vector3f> unrefined_points_;
  std::vector<Eigen::Vector3f> refined_points_;
  std::vector<Eigen::Vector3f> refined_views_;  // points + dir(yaw)
  std::vector<Eigen::Vector3f> refined_views1_, refined_views2_;
  std::vector<Eigen::Vector3f> refined_tour_;
};


template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};


inline void normalizeYaw(float &odom_yaw) 
{
  // 保证odom_yaw在(0, 2 * pi)范围内
  while (odom_yaw < 0) {
      odom_yaw += 2 * M_PI;
  }
  while (odom_yaw >= 2 * M_PI) {
      odom_yaw -= 2 * M_PI;
  }
}

} // namespace scoutair_planner 

#endif  // PLANNER_COMMON_H_