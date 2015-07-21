#include "ros/ros.h"
#include <std_msgs/String.h>
#include <iostream>
#include "dual_manipulation_shared/planner_service.h"
#include <dual_manipulation_planner/planner_lib.h>

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
    srv.request.object_id=10;
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
    srv.request.source.grasp_id=205;
    srv.request.source.workspace_id=1;
    srv.request.destination.grasp_id=514;
    srv.request.destination.workspace_id=6;
    
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
    srv.response.path.clear();
    dual_manipulation::planner::planner_lib a;
    a.set_object(10, "test_object");
    if (a.plan(205,1,514,6,srv.response.path))
    {
        ROS_INFO("Planning library returned a path");
        for (auto node:srv.response.path)
            std::cout<<node.grasp_id<<" "<<node.workspace_id<<std::endl;
    }
    else
    {
        ROS_ERROR("Failed to plan using the planner library");
        return 1;
    }
    ros::spin();
    
    return 0;
}