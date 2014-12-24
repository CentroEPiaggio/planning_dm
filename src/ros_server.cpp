#include "ros_server.h"
#include <graph_creator.h>
#include <lemon/smart_graph.h>

using namespace dual_manipulation::planner;

ros_server::ros_server():graph_creator(graph)
{
    Object a;
    graph_creator.create_graph(a);
    service = node.advertiseService("planner_ros_service", &ros_server::planner_ros_service, this);
}

bool ros_server::planner_ros_service(dual_manipulation_shared::planner_service::Request &req, dual_manipulation_shared::planner_service::Response &res)
{

//     if(IKControl.perform_ik(req))
//     {
// 	res.ack = true;
// 	ROS_INFO("Accepted request to perform ",
// 		  req.ee_name.c_str(),req.time,req.ee_pose.position.x,req.ee_pose.position.y,req.ee_pose.position.z,
// 		  req.ee_pose.orientation.x,req.ee_pose.orientation.y,req.ee_pose.orientation.z,req.ee_pose.orientation.w);
//     }
//     else
//     {
//         ROS_WARN("!! Request to perform ik denied !!");
// 	res.ack = false;
//     }

    return res.ack;
}

ros_server::~ros_server()
{

}