// #include <fstream>
#include <scoutair_planner/exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <lkh_tsp_solver/lkh_interface.h>
#include <scoutair_planner/graph_node.h>
// #include <active_perception/graph_search.h>
#include <scoutair_planner/perception_utils.h>
#include <scoutair_planner/raycast.h>
// #include <plan_env/sdf_map.h>
// #include <plan_env/edt_environment.h>
#include <scoutair_planner/frontier_map.h>
// #include <plan_manage/planner_manager.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

namespace scoutair_planner {
// SECTION interfaces for setup and query

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
//   planner_manager_.reset(new FastPlannerManager);
//   planner_manager_->initPlanModules(nh);
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));

//   ed_.reset(new ExplorationData);
  
  frontiermap_.reset(new FrontierMap( nh_, nh_private_ ));
  frontiermap_->init();

  ep_.reset(new ExplorationParam);

  ViewNode::astar_.reset(new Astar);  
  ViewNode::astar_->init(nh_private_, frontiermap_);

  float resolution_ = frontiermap_->voxel_size_;
  Eigen::Vector3f origin, size;
  origin = frontiermap_->voxbloxmap_origin_;
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);

  frontier_timer_ = nh_private_.createTimer(ros::Duration(0.1), &ExplorationManager::frontierCallback, this);

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
  static int delay = 0;
  if (delay++ < 2) return;

  frontiermap_->updateFrontierMap();
  Eigen::Vector3f pos(0,0,0); 
  Eigen::Vector3f vel(0,0,0);
  Eigen::Vector3f acc(0,0,0);
  Eigen::Vector3f yaw(0,0,0);
  planExploreMotion( pos, vel,acc, yaw );
}


int ExplorationManager::planExploreMotion( const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, const Eigen::Vector3f& acc, const Eigen::Vector3f& yaw ) 
{
  ros::Time t1 = ros::Time::now();
  auto t2 = t1;
  vector<Vector3f> views_;
  // vector<Vector3f> global_tour_;
  vector<vector<Vector3f>> frontiers_;
  vector<Vector3f> points_;
  vector<Vector3f> averages_;
  vector<float> yaws_;
  vector<int> refined_ids_;
  vector<vector<Vector3f>> n_points_;
  vector<Vector3f> unrefined_points_;
  // views_.clear();
  // global_tour_.clear();

  std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
            << ", acc: " << acc.transpose() << std::endl;

  // // Search frontiers and group them into clusters
  // frontiermap_->searchFrontiers();

  double frontier_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
  // frontiermap_->computeFrontiersToVisit();
  frontiermap_->getFrontiers(frontiers_);

  if (frontiers_.empty()) {
    ROS_WARN("No coverable frontier.");
    return NO_FRONTIER;
  }
  frontiermap_->getTopViewpointsInfo(pos, points_, yaws_, averages_);
  for (int i = 0; i < points_.size(); ++i)
    views_.push_back(
        points_[i] + 2.0 * Eigen::Vector3f(cos(yaws_[i]), sin(yaws_[i]), 0));

  double view_time = (ros::Time::now() - t1).toSec();
  ROS_WARN(
      "Frontier: %ld, t: %lf, viewpoint: %ld, t: %lf", frontiers_.size(), frontier_time,
      points_.size(), view_time);

  // Do global and local tour planning and retrieve the next viewpoint
  Eigen::Vector3f next_pos;
  float next_yaw;
  if (points_.size() > 1) {
    // Find the global tour passing through all viewpoints
    // Create TSP and solve by LKH
    // Optimal tour is returned as indices of frontier
    std::vector<int> indices;
    findGlobalTour(pos, vel, yaw, indices);

    if (ep_->refine_local_) {
    //   // Do refinement for the next few viewpoints in the global tour
    //   // Idx of the first K frontier in optimal tour
    //   // t1 = ros::Time::now();

    //   refined_ids_.clear();
    //   unrefined_points_.clear();
    //   int knum = min(int(indices.size()), ep_->refined_num_);
    //   for (int i = 0; i < knum; ++i) {
    //     auto tmp = points_[indices[i]];
    //     unrefined_points_.push_back(tmp);
    //     refined_ids_.push_back(indices[i]);
    //     if ((tmp - pos).norm() > ep_->refined_radius_ && refined_ids_.size() >= 2) break;
    //   }

    //   // Get top N viewpoints for the next K frontiers
    //   n_points_.clear();
    //   std::vector<std::vector<double>> n_yaws;
    //   frontiermap_->getViewpointsInfo(
    //       pos, refined_ids_, ep_->top_view_num_, ep_->max_decay_, n_points_, n_yaws);

    //   refined_points_.clear();
    //   refined_views_.clear();
    //   std::vector<double> refined_yaws;
    //   refineLocalTour(pos, vel, yaw, n_points_, n_yaws, refined_points_, refined_yaws);
    //   next_pos = refined_points_[0];
    //   next_yaw = refined_yaws[0];

    //   // Get marker for view visualization
    //   for (int i = 0; i < refined_points_.size(); ++i) {
    //     Eigen::Vector3f view =
    //         refined_points_[i] + 2.0 * Eigen::Vector3f(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
    //     refined_views_.push_back(view);
    //   }
    //   refined_views1_.clear();
    //   refined_views2_.clear();
    //   for (int i = 0; i < refined_points_.size(); ++i) {
    //     std::vector<Eigen::Vector3f> v1, v2;
    //     frontiermap_->percep_utils_->setPose(refined_points_[i], refined_yaws[i]);
    //     frontiermap_->percep_utils_->getFOV(v1, v2);
    //     refined_views1_.insert(refined_views1_.end(), v1.begin(), v1.end());
    //     refined_views2_.insert(refined_views2_.end(), v2.begin(), v2.end());
    //   }
    //   double local_time = (ros::Time::now() - t1).toSec();
    //   ROS_WARN("Local refine time: %lf", local_time);

    } else {
      // Choose the next viewpoint from global tour
      next_pos = points_[indices[0]];
      next_yaw = yaws_[indices[0]];
    }
  } else if (points_.size() == 1) {
    // Only 1 destination, no need to find global tour through TSP
    global_tour_ = { pos, points_[0] };
    // refined_tour_.clear();
    // refined_views1_.clear();
    // refined_views2_.clear();

    if (ep_->refine_local_) {
  //     // Find the min cost viewpoint for next frontier
  //     refined_ids_ = { 0 };
  //     unrefined_points_ = { points_[0] };
  //     n_points_.clear();
  //     std::vector<std::vector<double>> n_yaws;
  //     frontiermap_->getViewpointsInfo(
  //         pos, { 0 }, ep_->top_view_num_, ep_->max_decay_, n_points_, n_yaws);

  //     double min_cost = 100000;
  //     int min_cost_id = -1;
  //     std::vector<Eigen::Vector3f> tmp_path;
  //     for (int i = 0; i < n_points_[0].size(); ++i) {
  //       auto tmp_cost = ViewNode::computeCost(
  //           pos, n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
  //       if (tmp_cost < min_cost) {
  //         min_cost = tmp_cost;
  //         min_cost_id = i;
  //       }
  //     }
  //     next_pos = n_points_[0][min_cost_id];
  //     next_yaw = n_yaws[0][min_cost_id];
  //     refined_points_ = { next_pos };
  //     refined_views_ = { next_pos + 2.0 * Eigen::Vector3f(cos(next_yaw), sin(next_yaw), 0) };
    } else {
      next_pos = points_[0];
      next_yaw = yaws_[0];
    }
  } else
    ROS_ERROR("Empty destination.");

  std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;

  // // Plan trajectory (position and yaw) to the next viewpoint
  // t1 = ros::Time::now();

  // // Compute time lower bound of yaw and use in trajectory generation
  // double diff = fabs(next_yaw - yaw[0]);
  // double time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;

  // // Generate trajectory of x,y,z
  // planner_manager_->path_finder_->reset();
  // if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END) {
  //   ROS_ERROR("No path to next viewpoint");
  //   return FAIL;
  // }
  // path_next_goal_ = planner_manager_->path_finder_->getPath();
  // shortenPath(path_next_goal_);

  // const double radius_far = 5.0;
  // const double radius_close = 1.5;
  // const double len = Astar::pathLength(path_next_goal_);
  // if (len < radius_close) {
  //   // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
  //   // optimization
  //   planner_manager_->planExploreTraj(path_next_goal_, vel, acc, time_lb);
  //   next_goal_ = next_pos;

  // } else if (len > radius_far) {
  //   // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
  //   // dead end)
  //   std::cout << "Far goal." << std::endl;
  //   double len2 = 0.0;
  //   std::vector<Eigen::Vector3f> truncated_path = { path_next_goal_.front() };
  //   for (int i = 1; i < path_next_goal_.size() && len2 < radius_far; ++i) {
  //     auto cur_pt = path_next_goal_[i];
  //     len2 += (cur_pt - truncated_path.back()).norm();
  //     truncated_path.push_back(cur_pt);
  //   }
  //   next_goal_ = truncated_path.back();
  //   planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
  //   // if (!planner_manager_->kinodynamicReplan(
  //   //         pos, vel, acc, next_goal_, Eigen::Vector3f(0, 0, 0), time_lb))
  //   //   return FAIL;
  //   // kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  // } else {
  //   // Search kino path to exactly next viewpoint and optimize
  //   std::cout << "Mid goal" << std::endl;
  //   next_goal_ = next_pos;

  //   if (!planner_manager_->kinodynamicReplan(
  //           pos, vel, acc, next_goal_, Eigen::Vector3f(0, 0, 0), time_lb))
  //     return FAIL;
  // }

  // if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
  //   ROS_ERROR("Lower bound not satified!");

  // planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);

  // double traj_plan_time = (ros::Time::now() - t1).toSec();
  // t1 = ros::Time::now();

  // double yaw_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);
  // double total = (ros::Time::now() - t2).toSec();
  // ROS_WARN("Total time: %lf", total);
  // ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

  return SUCCEED;
}

// void ExplorationManager::shortenPath(std::vector<Eigen::Vector3f>& path) {
//   if (path.empty()) {
//     ROS_ERROR("Empty path to shorten");
//     return;
//   }
//   // Shorten the tour, only critical intermediate points are reserved.
//   const double dist_thresh = 3.0;
//   std::vector<Eigen::Vector3f> short_tour = { path.front() };
//   for (int i = 1; i < path.size() - 1; ++i) {
//     if ((path[i] - short_tour.back()).norm() > dist_thresh)
//       short_tour.push_back(path[i]);
//     else {
//       // Add waypoints to shorten path only to avoid collision
//       ViewNode::caster_->input(short_tour.back(), path[i + 1]);
//       Eigen::Vector3i idx;
//       while (ViewNode::caster_->nextId(idx) && ros::ok()) {
//         if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
//             edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
//           short_tour.push_back(path[i]);
//           break;
//         }
//       }
//     }
//   }
//   if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

//   // Ensure at least three points in the path
//   if (short_tour.size() == 2)
//     short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
//   path = short_tour;
// }


void ExplorationManager::findGlobalTour( const Eigen::Vector3f& cur_pos, const Eigen::Vector3f& cur_vel, 
                                         const Eigen::Vector3f cur_yaw, std::vector<int>& indices ) 
{
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXf cost_mat;
  // frontiermap_->updateFrontierCostMatrix();
  frontiermap_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();

  std::cout<<"cost_matrix rows" <<std::endl;

  double mat_time = (ros::Time::now() - t1).toSec();
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
  frontiermap_->getPathForTour(cur_pos, indices, global_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}

// void ExplorationManager::refineLocalTour(
//     const Eigen::Vector3f& cur_pos, const Eigen::Vector3f& cur_vel, const Eigen::Vector3f& cur_yaw,
//     const std::vector<std::vector<Eigen::Vector3f>>& n_points, const std::vector<std::vector<double>>& n_yaws,
//     std::vector<Eigen::Vector3f>& refined_pts, std::vector<double>& refined_yaws) {
//   double create_time, search_time, parse_time;
//   auto t1 = ros::Time::now();

//   // Create graph for viewpoints selection
//   GraphSearch<ViewNode> g_search;
//   std::vector<ViewNode::Ptr> last_group, cur_group;

//   // Add the current state
//   ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
//   first->vel_ = cur_vel;
//   g_search.addNode(first);
//   last_group.push_back(first);
//   ViewNode::Ptr final_node;

//   // Add viewpoints
//   std::cout << "Local tour graph: ";
//   for (int i = 0; i < n_points.size(); ++i) {
//     // Create nodes for viewpoints of one frontier
//     for (int j = 0; j < n_points[i].size(); ++j) {
//       ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
//       g_search.addNode(node);
//       // Connect a node to nodes in last group
//       for (auto nd : last_group)
//         g_search.addEdge(nd->id_, node->id_);
//       cur_group.push_back(node);

//       // Only keep the first viewpoint of the last local frontier
//       if (i == n_points.size() - 1) {
//         final_node = node;
//         break;
//       }
//     }
//     // Store nodes for this group for connecting edges
//     std::cout << cur_group.size() << ", ";
//     last_group = cur_group;
//     cur_group.clear();
//   }
//   std::cout << "" << std::endl;
//   create_time = (ros::Time::now() - t1).toSec();
//   t1 = ros::Time::now();

//   // Search optimal sequence
//   std::vector<ViewNode::Ptr> path;
//   g_search.DijkstraSearch(first->id_, final_node->id_, path);

//   search_time = (ros::Time::now() - t1).toSec();
//   t1 = ros::Time::now();

//   // Return searched sequence
//   for (int i = 1; i < path.size(); ++i) {
//     refined_pts.push_back(path[i]->pos_);
//     refined_yaws.push_back(path[i]->yaw_);
//   }

//   // Extract optimal local tour (for visualization)
//   refined_tour_.clear();
//   refined_tour_.push_back(cur_pos);
//   ViewNode::astar_->lambda_heu_ = 1.0;
//   ViewNode::astar_->setResolution(0.2);
//   for (auto pt : refined_pts) {
//     std::vector<Eigen::Vector3f> path;
//     if (ViewNode::searchPath(refined_tour_.back(), pt, path))
//       refined_tour_.insert(refined_tour_.end(), path.begin(), path.end());
//     else
//       refined_tour_.push_back(pt);
//   }
//   ViewNode::astar_->lambda_heu_ = 10000;

//   parse_time = (ros::Time::now() - t1).toSec();
//   // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
// }

}  // namespace scoutair_planner
