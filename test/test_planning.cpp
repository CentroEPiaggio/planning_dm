#include "ros/ros.h"
#include <std_msgs/String.h>
#include <iostream>
#include "dual_manipulation_shared/planner_service.h"

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> planning_test "<<std::endl;
    std::cout<<std::endl;
    
    ros::init(argc, argv, "planning_test");
    
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<dual_manipulation_shared::planner_service>("planner_ros_service");
    dual_manipulation_shared::planner_service srv;

    srv.request.command="set object";
    srv.request.time = 2;
    srv.request.object_id=1;
    srv.request.object_name="test object";
    
    if (client.call(srv))
    {
        ROS_INFO("Object id set: %d", (int)srv.response.ack);
    }
    else
    {
        ROS_ERROR("Failed to call service dual_manipulation_shared::planner_service");
        return 1;
    }
    
    srv.request.command="plan";
    srv.request.source.grasp_id=15;
    srv.request.source.workspace_id=3;
    srv.request.destination.grasp_id=14;
    srv.request.destination.workspace_id=1;
    
    if (client.call(srv))
    {
        ROS_INFO("Planning Request accepted: %d", (int)srv.response.ack);
        for (auto node:srv.response.path)
            std::cout<<node.grasp_id<<" "<<node.workspace_id<<std::endl;
    }
    else
    {
        ROS_ERROR("Failed to call service dual_manipulation_shared::planner_service");
        return 1;
    }
        
    ros::spin();
    
    return 0;
}