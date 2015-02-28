#include <iostream>
#include "ros_server.h"
int main(int argc, char **argv) {
    
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> Planner running"<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "dual_manipulation_planner");
    
    int x,offx,y,offy;
    if (argc > 1)
    {
        x=atoi(argv[1]);
        offx=atoi(argv[2]);
        y=atoi(argv[3]);
        offy=atoi(argv[4]);
        dual_manipulation::planner::ros_server* server= new dual_manipulation::planner::ros_server(x,offx,y,offy);
    }
    else
        dual_manipulation::planner::ros_server* server = new dual_manipulation::planner::ros_server();
    ros::spin();

    return 0;
}
