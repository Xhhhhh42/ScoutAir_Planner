#include <ros/ros.h>
#include "scoutair_planner/exploration_fsm.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_fsm_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // std::cout << "main start" << std::endl;

    scoutair_planner::ExplorationFSM exploration_fsm( nh, nh_private );
    exploration_fsm.init();

    // std::cout << "main init" << std::endl;

    ros::spin();

    return 0;
}