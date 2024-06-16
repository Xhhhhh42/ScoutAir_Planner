#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

namespace scoutair_planner {

float signum(float x);

float mod(float value, float modulus);

float intbound(float s, float ds);

void Raycast(const Eigen::Vector3f& start, const Eigen::Vector3f& end, const Eigen::Vector3f& min,
             const Eigen::Vector3f& max, int& output_points_cnt, Eigen::Vector3f* output);

void Raycast(const Eigen::Vector3f& start, const Eigen::Vector3f& end, const Eigen::Vector3f& min,
             const Eigen::Vector3f& max, std::vector<Eigen::Vector3f>* output);

class RayCaster {
private:
  /* data */
  Eigen::Vector3f start_;
  Eigen::Vector3f end_;
  Eigen::Vector3f direction_;
  Eigen::Vector3f min_;
  Eigen::Vector3f max_;
  int x_;
  int y_;
  int z_;
  int endX_;
  int endY_;
  int endZ_;
  float maxDist_;
  float dx_;
  float dy_;
  float dz_;
  int stepX_;
  int stepY_;
  int stepZ_;
  float tMaxX_;
  float tMaxY_;
  float tMaxZ_;
  float tDeltaX_;
  float tDeltaY_;
  float tDeltaZ_;
  float dist_;

  int step_num_;

  float resolution_;
  Eigen::Vector3f offset_;
  Eigen::Vector3f half_;

public:
  RayCaster(/* args */) {
  }
  ~RayCaster() {
  }

  void setParams(const float& res, const Eigen::Vector3f& origin);
  bool input(const Eigen::Vector3f& start, const Eigen::Vector3f& end);
  bool nextId(Eigen::Vector3i& idx);
  bool nextPos(Eigen::Vector3f& pos);

  // deprecated
  bool setInput(const Eigen::Vector3f& start, const Eigen::Vector3f& end /* , const Eigen::Vector3f& min,
                const Eigen::Vector3f& max */);
  bool step(Eigen::Vector3f& ray_pt);
};

}  // namespace scoutair_planner

#endif  // RAYCAST_H_