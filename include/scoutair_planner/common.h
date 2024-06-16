#ifndef PLANNER_COMMON_H_
#define PLANNER_COMMON_H_

#include <voxblox/core/common.h>

// 为 GlobalIndex 提供 hash 函数
namespace std {
  template <>
  struct hash<voxblox::GlobalIndex> {
    std::size_t operator()(const voxblox::GlobalIndex& k) const {
      return std::hash<voxblox::LongIndexElement>()(k.x()) ^ std::hash<voxblox::LongIndexElement>()(k.y()) ^ std::hash<voxblox::LongIndexElement>()(k.z());
    }
  };
}

#endif  // PLANNER_COMMON_H_