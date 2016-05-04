#ifndef DUAL_MANIPULATION_PLANNER_LIB
#define DUAL_MANIPULATION_PLANNER_LIB
#include "graph_creator.h"
#include <dual_manipulation_shared/planner_item.h>
#include <dual_manipulation_shared/planner_serviceRequest.h>
#include <dual_manipulation_shared/planner_serviceResponse.h>
#include <dual_manipulation_shared/planner_service_response.h>
#include <std_msgs/Header.h>

namespace dual_manipulation
{
namespace planner
{

class planner_lib : protected graphCreator
{

public:

    planner_lib();
    bool set_object(object_id id, std::string name, int priority=0);

    void clear_filtered_arcs();

    /**
    * @brief You can add filtered arcs progressively as long as you don't want to reset the planner to the full graph status
    * 
    * @return void
    */
    void add_filtered_arc(grasp_id source_grasp_id, workspace_id source_workspace_id, grasp_id target_grasp_id, workspace_id target_workspace_id);

    dual_manipulation_shared::planner_serviceRequest filtered_arcs;
    bool plan(grasp_id source_grasp_id, workspace_id source_workspace_id, grasp_id target_grasp_id, workspace_id target_workspace_id, std::vector< dual_manipulation_shared::planner_item >& path);

    void draw_path();
    ~planner_lib(){}
    inline std::string getName(){return name;}
    bool remote_plan(grasp_id source_grasp_id, workspace_id source_workspace_id, grasp_id target_grasp_id, workspace_id target_workspace_id, std::vector< dual_manipulation_shared::planner_item >& path);
    bool barrier();
protected:
    void barrier_callback(const std_msgs::HeaderConstPtr & msg);
    void plan_callback(const dual_manipulation_shared::planner_service_responseConstPtr& msg);    
    
    lemon::SmartDigraph::ArcMap<bool>* arc_filter;
    Object obj;
    lemon::Path<lemon::SmartDigraph> computed_path;
//     lemon::SmartDigraph real_graph;

    
    //Ros stuff to get and send plans and synchronization
    ros::NodeHandle nh;
    ros::Publisher plan_request_pub;
    ros::Subscriber plan_sub;
    ros::Subscriber multiplan_barrier_sub;
    ros::Publisher multiplan_barrier_pub;
    
    bool barrier_received;
    bool barrier_ok;
    bool plan_received;
    dual_manipulation_shared::planner_service_response::_path_type last_plan_received;
    int priority;
    std::string name;
};



}
}


#endif //DUAL_MANIPULATION_PLANNER_LIB