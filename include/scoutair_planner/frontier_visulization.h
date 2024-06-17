#ifndef _FRONTIER_VIS_H_
#define _FRONTIER_VIS_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <scoutair_planner/voxblox_map.h>

namespace scoutair_planner {

class FtrVisulization 
{
public:
  FtrVisulization( const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                   std::shared_ptr<scoutair_planner::VoxbloxMap> &voxblox_map );
  ~FtrVisulization();

  void drawCubes( const std::vector<Eigen::Vector3f>& list, const float& scale, const float& color_h, float color_alpha,
                  const std::string& ns, const int& id /* const int& pub_id */ );

  void publishBlock( const std::vector<voxblox::BlockIndex>& block_index_list ); 

  void drawFOV( const std::vector<Eigen::Vector3f>& list1, const std::vector<Eigen::Vector3f>& list2 );               

private:
  void fillBasicInfo( visualization_msgs::Marker& mk, const Eigen::Vector3f& scale,
                      const Eigen::Vector4f& color, const std::string& ns, const int& id, const int& shape );
  
  void fillGeometryInfo( visualization_msgs::Marker& mk, const std::vector<Eigen::Vector3f>& list );
  void fillGeometryInfo( visualization_msgs::Marker& mk, const std::vector<Eigen::Vector3f>& list1,
                         const std::vector<Eigen::Vector3f>& list2);
  
  Eigen::Vector4f getColor(const float& h, float alpha = 1.0);

  void initVisualizationMarker( visualization_msgs::Marker &mk, const Eigen::Vector3f& scale, const Eigen::Vector4f& color );

// ROS Parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher frontier_pub_;
  ros::Publisher uccu_pub_, free_pub_, unkno_pub_;
  ros::Publisher fov_pub_;

  std::shared_ptr<VoxbloxMap> voxblox_map_;
};


}  // namespace scoutair_planner

#endif // _FRONTIER_VIS_H_