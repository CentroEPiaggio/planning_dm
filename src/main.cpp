#include <iostream>
#include <graph_creator.h>
#include "ros_server.h"
int main(int argc, char **argv) {
    
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> Planner running"<<std::endl;
    std::cout<<std::endl;
    
    ros::init(argc, argv, "dual_manipulation_planner");
    dual_manipulation::planner::ros_server server;
    ros::spin();

    return 0;
}
