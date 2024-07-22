#include <scoutair_planner/exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <lkh_tsp_solver/lkh_interface.h>
#include <scoutair_planner/graph_node.h>
#include <scoutair_planner/graph_search.h>
#include <scoutair_planner/perception_utils.h>
#include <scoutair_planner/raycast.h>
#include <scoutair_planner/frontier_map.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

namespace scoutair_planner {

using namespace Eigen;
using namespace std;

ExplorationManager::ExplorationManager( const ros::NodeHandle& nh, const ros::NodeHandle& nh_private ) 
  : nh_(nh),
    nh_private_(nh_private)
{
  initialize();
}

ExplorationManager::~ExplorationManager() 
{
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::free_bit_.reset();
}

void ExplorationManager::initialize() 
{ 
  frontiermap_.reset(new FrontierMap( nh_, nh_private_ ));
  frontiermap_->init();

  ep_.reset(new ExplorationParam);
  ed_.reset(new ExplorationData);

  ViewNode::astar_.reset(new Astar);  
  ViewNode::astar_->init(nh_private_, frontiermap_);

  float resolution_ = frontiermap_->voxel_size_;
  Eigen::Vector3f origin, size;
  origin = frontiermap_->voxbloxmap_origin_;
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);

  manager_visu_ = frontiermap_->ftr_visu_;

  nh_private_.param("exploration/refine_local", ep_->refine_local_, true);
  nh_private_.param("exploration/refined_num", ep_->refined_num_, -1);
  nh_private_.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh_private_.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh_private_.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh_private_.param("exploration/tsp_dir", ep_->tsp_dir_, std::string("null"));
  nh_private_.param("exploration/relax_time", ep_->relax_time_, 1.0);

  double vm, am, yd, ydd, w_dir;
  nh_private_.param("exploration/vm", vm, -1.0);
  nh_private_.param("exploration/am", am, -1.0);
  nh_private_.param("exploration/yd", yd, -1.0);
  nh_private_.param("exploration/ydd", ydd, -1.0);
  nh_private_.param("exploration/w_dir", w_dir, -1.0);
  ViewNode::vm_ = static_cast<float>(vm);
  ViewNode::am_ = static_cast<float>(am);
  ViewNode::yd_ = static_cast<float>(yd);
  ViewNode::ydd_ = static_cast<float>(ydd);
  ViewNode::w_dir_ = static_cast<float>(w_dir);

  // Initialize TSP par file
  ofstream par_file(ep_->tsp_dir_ + "/single.par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 1\n";

  ROS_INFO("Exploration Manager inited");
}


void ExplorationManager::frontierCallback( const ros::TimerEvent& e )
{
  // static int delay = 0;
  // if (delay++ < 2) return;

  // frontiermap_->updateFrontierMap();
  // Eigen::Vector3f pos(0,0,1); 
  // Eigen::Vector3f vel(0,0,0);
  // Eigen::Vector3f acc(0,0,0);
  // Eigen::Vector3f yaw(0,0,0);
  // planExploreMotion( pos, vel,acc, yaw );
}


int ExplorationManager::planExploreMotion( const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, const Eigen::Vector3f& acc, 
                                           const Eigen::Vector3f& yaw, std::vector<Eigen::Vector3f> &next_pos_vec, std::vector<float> &next_yaw_vec ) 
{
  ros::Time t1 = ros::Time::now();
  // auto t2 = t1;
  vector<Vector3f> views_;
  vector<vector<Vector3f>> frontiers_;
  vector<Vector3f> points_;
  vector<Vector3f> averages_;
  vector<float> yaws_;
  vector<Vector3f> refined_views_;
  // views_.clear();
  ed_->global_tour_.clear();

  std::cout << "start pos: " << pos.transpose() << "\nvel:      " << vel.transpose() << std::endl;

  t1 = ros::Time::now();

  // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
  frontiermap_->getFrontiers(frontiers_);

  if (frontiers_.empty()) {
    ROS_WARN("No coverable frontier.");
    return NO_FRONTIER;
  }
  frontiermap_->getTopViewpointsInfo(pos, points_, yaws_, averages_);
  for (int i = 0; i < points_.size(); ++i)
    views_.push_back(
        points_[i] + 2.0 * Eigen::Vector3f(cos(yaws_[i]), sin(yaws_[i]), 0));

  // double view_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN( "Frontier: %ld, t: %lf, viewpoint: %ld, t: %lf", frontiers_.size(), frontier_time, points_.size(), view_time);

  // Do global and local tour planning and retrieve the next viewpoint
  if (points_.size() > 1) {
    // Find the global tour passing through all viewpoints
    // Create TSP and solve by LKH
    // Optimal tour is returned as indices of frontier
    std::vector<int> indices;
    findGlobalTour(pos, vel, yaw, indices);

    if (ep_->refine_local_) {
      // Do refinement for the next few viewpoints in the global tour
      // Idx of the first K frontier in optimal tour
      t1 = ros::Time::now();

      ed_->refined_ids_.clear();
      ed_->unrefined_points_.clear();
      int knum = min(int(indices.size()), ep_->refined_num_);
      for (int i = 0; i < knum; ++i) {
        auto tmp = points_[indices[i]];
        ed_->unrefined_points_.push_back(tmp);
        ed_->refined_ids_.push_back(indices[i]);
        if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2) break;
      }

      // Get top N viewpoints for the next K frontiers
      ed_->n_points_.clear();
      std::vector<std::vector<float>> n_yaws;
      frontiermap_->getViewpointsInfo( pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws );

      ed_->refined_points_.clear();
      refined_views_.clear();
      std::vector<float> refined_yaws;
      refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
      next_pos_vec = ed_->refined_points_;
      next_yaw_vec = refined_yaws;
      // next_pos = ed_->refined_points_[0];
      // next_yaw = refined_yaws[0];

    } else {
      // Choose the next viewpoint from global tour
      if( indices.size() != points_.size() ) {
        ROS_ERROR("ExplorationManager::planExploreMotion ERROR with findGlobalTour.");
      }
      for( auto ele: indices ) {
        next_pos_vec.push_back(points_[ele]);
        next_yaw_vec.push_back(yaws_[ele]);
      }
      // next_pos = points_[indices[0]];
      // next_yaw = yaws_[indices[0]];
    }
  } else if (points_.size() == 1) {
    // Only 1 destination, no need to find global tour through TSP
    ed_->global_tour_ = { pos, points_[0] };
    ed_->refined_tour_.clear();    

    // if (ep_->refine_local_) {
    //   // Find the min cost viewpoint for next frontier
    //   ed_->refined_ids_ = { 0 };
    //   ed_->unrefined_points_ = { points_[0] };
    //   ed_->n_points_.clear();
    //   std::vector<std::vector<float>> n_yaws;
    //   frontiermap_->getViewpointsInfo(
    //       pos, { 0 }, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

    //   double min_cost = 100000;
    //   int min_cost_id = -1;
    //   std::vector<Eigen::Vector3f> tmp_path;
    //   for (int i = 0; i < ed_->n_points_[0].size(); ++i) {
    //     auto tmp_cost = ViewNode::computeCost(
    //         pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
    //     if (tmp_cost < min_cost) {
    //       min_cost = tmp_cost;
    //       min_cost_id = i;
    //     }
    //   }
    //   next_pos = ed_->n_points_[0][min_cost_id];
    //   next_yaw = n_yaws[0][min_cost_id];
    //   ed_->refined_points_ = { next_pos };
    //   refined_views_ = { next_pos + 2.0 * Eigen::Vector3f(cos(next_yaw), sin(next_yaw), 0) };
    // } else {
    //   next_pos = points_[0];
    //   next_yaw = yaws_[0];
    // }
    next_pos_vec.push_back(points_[0]);
    next_yaw_vec.push_back(yaws_[0]);
  } else
    ROS_ERROR("Empty destination.");

  // std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;

  manager_visu_->publishGlobalTour(ed_->global_tour_);

  return SUCCEED;
}


void ExplorationManager::findGlobalTour( const Eigen::Vector3f& cur_pos, const Eigen::Vector3f& cur_vel, 
                                         const Eigen::Vector3f cur_yaw, std::vector<int>& indices ) 
{
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXf cost_mat;
  // frontiermap_->updateFrontierCostMatrix();
  frontiermap_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();

  // std::cout<<"cost_matrix rows" <<std::endl;

  // double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  
  // Problem specification part, follow the format of TSPLIB
  std::string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;
 
  // Problem data part
  const int scale = 100;
  if (false) {
    // Use symmetric TSP
    for (int i = 1; i < dimension; ++i) {
      for (int j = 0; j < i; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }

  } else {
    // Use Asymmetric TSP
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  std::string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  if (false) {
    // Read path for Symmetric TSP formulation
    getline(res_file, res);  // Skip current pose
    getline(res_file, res);
    int id = stoi(res);
    bool rev = (id == dimension);  // The next node is virutal depot?

    while (id != -1) {
      indices.push_back(id - 2);
      getline(res_file, res);
      id = stoi(res);
    }
    if (rev) reverse(indices.begin(), indices.end());
    indices.pop_back();  // Remove the depot

  } else {
    // Read path for ATSP formulation
    while (getline(res_file, res)) {
      // Read indices of frontiers in optimal tour
      int id = stoi(res);
      if (id == 1)  // Ignore the current state
        continue;
      if (id == -1) break;
      indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
    }
  }

  res_file.close();

  // Get the path of optimal tour from path matrix
  frontiermap_->getPathForTour(cur_pos, indices, ed_->global_tour_);

  // double tsp_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}


/// @brief 细化一个局部路径（tour），从当前的位置和状态出发，利用Dijkstra算法进行路径规划，最终生成一个细化后的路径和航向（yaw）序列
/// @param cur_pos 
/// @param cur_vel 
/// @param cur_yaw 
/// @param n_points 多个候选点的集合，每个候选点表示一个可能的观测点
/// @param n_yaws 
/// @param refined_pts 
/// @param refined_yaws 
void ExplorationManager::refineLocalTour( const Eigen::Vector3f& cur_pos, const Eigen::Vector3f& cur_vel, const Eigen::Vector3f& cur_yaw,
                                          const std::vector<std::vector<Eigen::Vector3f>>& n_points, const std::vector<std::vector<float>>& n_yaws,
                                          std::vector<Eigen::Vector3f>& refined_pts, std::vector<float>& refined_yaws) 
{
  // double create_time, search_time, parse_time;
  // auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  scoutair_planner::GraphSearch<ViewNode> g_search;
  std::vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  std::cout << "Local tour graph: ";
  for (int i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group)
        g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  // create_time = (ros::Time::now() - t1).toSec();
  // t1 = ros::Time::now();

  // Search optimal sequence
  std::vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  // search_time = (ros::Time::now() - t1).toSec();
  // t1 = ros::Time::now();

  // Return searched sequence
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  // 清空并初始化ed_->refined_tour_，使用A*算法进一步细化路径点，确保路径段的平滑和连贯性
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  ViewNode::astar_->lambda_heu_ = 1.0;
  ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    std::vector<Eigen::Vector3f> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  ViewNode::astar_->lambda_heu_ = 10000;

  // parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

}  // namespace scoutair_planner
