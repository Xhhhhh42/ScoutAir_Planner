#include <ros/ros.h>
// #include "scoutair_planner/frontier_map.h"
#include "scoutair_planner/exploration_manager.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_manager_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    scoutair_planner::ExplorationManager exploration_manager( nh, nh_private );

    ros::spin();

    return 0;
}