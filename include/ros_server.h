#ifndef PLANNER_ROS_SERVER
#define PLANNER_ROS_SERVER

#include <ros/ros.h>
#include <lemon/smart_graph.h>
#include "graph_creator.h"
#include "dual_manipulation_shared/planner_service.h"

/**
 * @brief service planner_service:
 * string source_workspace_name
 * uint64_t source_grasp_id
 * string target_workspace_name
 * uint64_t target_grasp_id
 * float64 time
 *   ---
 *   bool ack
 */

namespace dual_manipulation
{
namespace planner
{

class ros_server
{
public:
    ros_server();
    ros_server(int x,int offx,int y,int offy);
    ~ros_server();
private:
    ros::NodeHandle node;

    ros::ServiceServer service_server;

    bool planner_ros_service(dual_manipulation_shared::planner_service::Request &req, dual_manipulation_shared::planner_service::Response &res);

    ros::ServiceServer service;
    
    graphCreator graph_creator;
    Object obj;
};

}
}

#endif // PLANNER_ROS_SERVER