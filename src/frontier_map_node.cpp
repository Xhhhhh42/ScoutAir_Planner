#include <ros/ros.h>
#include "scoutair_planner/frontier_map.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_map_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    scoutair_planner::FrontierMap frontier_map( nh, nh_private );

    ros::spin();

    return 0;
}