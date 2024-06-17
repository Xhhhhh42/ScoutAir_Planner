#include <scoutair_planner/frontier_visulization.h>

namespace scoutair_planner {

using namespace Eigen;
using namespace std;

FtrVisulization::FtrVisulization( const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                                  std::shared_ptr<scoutair_planner::VoxbloxMap> &voxblox_map ) 
  : nh_(nh),
    nh_private_(nh_private),
    voxblox_map_(voxblox_map)
{
  frontier_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/planning_vis/frontier", 10000);

  uccu_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/planning_vis/kOccupied", 10000);
  free_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/planning_vis/kFree", 10000);
  unkno_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/planning_vis/kUnknown", 10000);

  fov_pub_ = nh_private_.advertise<visualization_msgs::Marker>("/planning_vis/position_cmd_vis", 10);
}


FtrVisulization::~FtrVisulization() {
}


void FtrVisulization::drawCubes( const std::vector<Eigen::Vector3f>& list, const float& scale, const float& color_h, float color_alpha,
                                 const std::string& ns, const int& id /* const int& pub_id */ ) 
{
  visualization_msgs::Marker mk;
  Eigen::Vector4f color = getColor(color_h, color_alpha);
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


void FtrVisulization::fillBasicInfo( visualization_msgs::Marker& mk, const Eigen::Vector3f& scale,
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


void FtrVisulization::fillGeometryInfo( visualization_msgs::Marker& mk,
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


void FtrVisulization::fillGeometryInfo( visualization_msgs::Marker& mk,
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


Eigen::Vector4f FtrVisulization::getColor( const float& h, float alpha ) 
{
  float h1 = h;
  if (h1 < 0.0 || h1 > 1.0) {
    std::cout << "h out of range" << std::endl;
    h1 = 0.0;
  }

  // Predefined colors for the intervals
  static const Eigen::Vector4f colors[] = {
    Eigen::Vector4f(1, 0, 0, 1), // Red
    Eigen::Vector4f(1, 0, 1, 1), // Magenta
    Eigen::Vector4f(0, 0, 1, 1), // Blue
    Eigen::Vector4f(0, 1, 1, 1), // Cyan
    Eigen::Vector4f(0, 1, 0, 1), // Green
    Eigen::Vector4f(1, 1, 0, 1), // Yellow
    Eigen::Vector4f(1, 0, 0, 1)  // Red again
  };

  float lambda;
  int index;

  // Eigen::Vector4f color1, color2;
  // if (h1 >= -1e-4 && h1 < 1.0 / 6) {
  //   lambda = (h1 - 0.0) * 6;
  //   color1 = Eigen::Vector4f(1, 0, 0, 1);
  //   color2 = Eigen::Vector4f(1, 0, 1, 1);
  // } else if (h1 >= 1.0 / 6 && h1 < 2.0 / 6) {
  //   lambda = (h1 - 1.0 / 6) * 6;
  //   color1 = Eigen::Vector4f(1, 0, 1, 1);
  //   color2 = Eigen::Vector4f(0, 0, 1, 1);
  // } else if (h1 >= 2.0 / 6 && h1 < 3.0 / 6) {
  //   lambda = (h1 - 2.0 / 6) * 6;
  //   color1 = Eigen::Vector4f(0, 0, 1, 1);
  //   color2 = Eigen::Vector4f(0, 1, 1, 1);
  // } else if (h1 >= 3.0 / 6 && h1 < 4.0 / 6) {
  //   lambda = (h1 - 3.0 / 6) * 6;
  //   color1 = Eigen::Vector4f(0, 1, 1, 1);
  //   color2 = Eigen::Vector4f(0, 1, 0, 1);
  // } else if (h1 >= 4.0 / 6 && h1 < 5.0 / 6) {
  //   lambda = (h1 - 4.0 / 6) * 6;
  //   color1 = Eigen::Vector4f(0, 1, 0, 1);
  //   color2 = Eigen::Vector4f(1, 1, 0, 1);
  // } else if (h1 >= 5.0 / 6 && h1 <= 1.0 + 1e-4) {
  //   lambda = (h1 - 5.0 / 6) * 6;
  //   color1 = Eigen::Vector4f(1, 1, 0, 1);
  //   color2 = Eigen::Vector4f(1, 0, 0, 1);
  // }

  if (h1 >= -1e-4 && h1 < 1.0 / 6) {
    lambda = h1 * 6;
    index = 0;
  } else if (h1 < 2.0 / 6) {
    lambda = (h1 - 1.0 / 6) * 6;
    index = 1;
  } else if (h1 < 3.0 / 6) {
    lambda = (h1 - 2.0 / 6) * 6;
    index = 2;
  } else if (h1 < 4.0 / 6) {
    lambda = (h1 - 3.0 / 6) * 6;
    index = 3;
  } else if (h1 < 5.0 / 6) {
    lambda = (h1 - 4.0 / 6) * 6;
    index = 4;
  } else {
    lambda = (h1 - 5.0 / 6) * 6;
    index = 5;
  }
  
  Eigen::Vector4f color1 = colors[index];
  Eigen::Vector4f color2 = colors[index + 1];

  Eigen::Vector4f fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;

  return fcolor;

  // Eigen::Vector4f fcolor = (1 - lambda) * color1 + lambda * color2;
  // fcolor(3) = alpha;

  // return fcolor;
}


void FtrVisulization::publishBlock( const std::vector<voxblox::BlockIndex>& block_index_list )
{
  if( block_index_list.empty() ) 
    { return; }

  visualization_msgs::Marker mk_occu, mk_free, mk_unk;
  Eigen::Vector3f scale( 0.05, 0.05, 0.05 );
  initVisualizationMarker( mk_occu, scale, Eigen::Vector4f(1.0, 0.0, 0.0, 1.0 ) );
  initVisualizationMarker( mk_free, scale, Eigen::Vector4f(1.0, 1.0, 1.0, 1.0 ) );
  initVisualizationMarker( mk_unk, scale, Eigen::Vector4f(0.0, 0.0, 1.0, 1.0 ) );
  ros::Duration lifetime(1000); // 200s
  mk_occu.lifetime = lifetime;
  mk_free.lifetime = lifetime;
  mk_unk.lifetime = lifetime;

  for(const auto block_index: block_index_list ) {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<voxblox::EsdfVoxel>::Ptr esdf_block = voxblox_map_->getBlockPtrByIndex(block_index);
    if (!esdf_block) {
      std::cout << "Getting empty block index, publishBlock interrupt" << std::endl;
      return;
    }
    const size_t num_voxels_per_block = esdf_block->num_voxels();
    
    for (size_t id = 0u; id < num_voxels_per_block; ++id) {
      voxblox::Point point_tmp = esdf_block->computeCoordinatesFromLinearIndex(id);
      const voxblox::EsdfVoxel esdf_voxel = esdf_block->getVoxelByLinearIndex(id);
      VoxbloxMap::VoxelStatus status = voxblox_map_->checkVoxelStatus( &esdf_voxel );

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


void FtrVisulization::initVisualizationMarker( visualization_msgs::Marker &mk, const Eigen::Vector3f& scale, const Eigen::Vector4f& color )
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


void FtrVisulization::drawFOV( const std::vector<Eigen::Vector3f>& list1, const std::vector<Eigen::Vector3f>& list2 ) 
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.04;
  mk.scale.y = 0.04;
  mk.scale.z = 0.04;

  // Clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  fov_pub_.publish(mk);

  if (list1.size() == 0) return;

  // Pub new marker
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
  mk.action = visualization_msgs::Marker::ADD;
  fov_pub_.publish(mk);
}



}  // namespace scoutair_planner