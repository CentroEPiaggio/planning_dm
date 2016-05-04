#include "dual_manipulation_planner/planner_lib.h"
#include <lemon/adaptors.h>
#include <lemon/dijkstra.h>
#include <dual_manipulation_shared/planner_item.h>
#include <dual_manipulation_shared/planner_serviceResponse.h>
#include <dual_manipulation_shared/planner_serviceRequest.h>

dual_manipulation::planner::planner_lib::planner_lib():graphCreator()
{
    multiplan_barrier_pub = nh.advertise<std_msgs::Header>("/barrier_request",1);
    multiplan_barrier_sub = nh.subscribe<std_msgs::Header>("/barrier_result",1,&dual_manipulation::planner::planner_lib::barrier_callback,this);
    plan_request_pub = nh.advertise<dual_manipulation_shared::planner_serviceRequest>("/requestplan",1);
    plan_sub = nh.subscribe<dual_manipulation_shared::planner_serviceResponse>("/remoteplan",1,&dual_manipulation::planner::planner_lib::plan_callback,this);
}


void dual_manipulation::planner::planner_lib::draw_path()
{
    graphCreator::draw_path(computed_path,*arc_filter);
}

void dual_manipulation::planner::planner_lib::clear_filtered_arcs()
{
    for ( lemon::SmartDigraph::ArcIt i ( graph ); i != lemon::INVALID; ++i )
        (*arc_filter)[i]= true;
    filtered_arcs.filtered_source_nodes.clear();
    filtered_arcs.filtered_target_nodes.clear();   
}

void dual_manipulation::planner::planner_lib::add_filtered_arc(grasp_id source_grasp_id, workspace_id source_workspace_id, grasp_id target_grasp_id, workspace_id target_workspace_id)
{
    lemon::SmartDigraph::Arc to_be_filtered_arc;
    dual_manipulation_shared::planner_item temp;
    if (getArc(source_grasp_id,source_workspace_id,
        target_grasp_id,target_workspace_id,
        to_be_filtered_arc))
    {
//         std::cout << "Filtering arc between grasp:" << source_grasp_id << " in ws:" << source_workspace_id;
//         std::cout << " and grasp:" << target_grasp_id << " in ws:" << target_workspace_id << std::endl;
        (*arc_filter)[to_be_filtered_arc]=false;
        temp.grasp_id=source_grasp_id;
        temp.workspace_id=source_workspace_id;
        filtered_arcs.filtered_source_nodes.push_back(temp);
        temp.grasp_id=target_grasp_id;
        temp.workspace_id=target_workspace_id;
        filtered_arcs.filtered_target_nodes.push_back(temp);
    }
}

bool dual_manipulation::planner::planner_lib::set_object(object_id id, std::string name, int priority)
{
    std::cout<<"object set to "<<id<<" with name "<<name<<std::endl;
    obj.id=id;
    create_graph(obj);
    arc_filter = new lemon::SmartDigraph::ArcMap<bool>(graph,true);
    this->priority=priority;
    return true;
}

void dual_manipulation::planner::planner_lib::barrier_callback(const std_msgs::HeaderConstPtr& msg)
{
    //Set a variable to unlock the waiting of barrier, also provide a true/false in that variable
    barrier_received=true;
    barrier_ok=msg->seq;
}

void dual_manipulation::planner::planner_lib::plan_callback(const dual_manipulation_shared::planner_serviceResponseConstPtr& msg)
{
    //TODO Set a variable to unlock the waiting of remote_plan
    plan_received=true;
    last_plan_received=msg->path;
    if (msg->status=="failed") ROS_ERROR("Semantic plan failed, this should not happen unless the desired configuration is unreachable, check the grasps capabilities");
}


bool dual_manipulation::planner::planner_lib::remote_plan(grasp_id source_grasp_id, workspace_id source_workspace_id, grasp_id target_grasp_id, workspace_id target_workspace_id, std::vector<dual_manipulation_shared::planner_item> & path)
{
    lemon::SmartDigraph::Node source, target;
    if (!getNode(source_grasp_id,source_workspace_id,source))
    {
        std::cout<<"cannot find requested source grasp/workspace"<<std::endl;
        return false;
    }
    if (!getNode(target_grasp_id,target_workspace_id,target)) 
    {
        std::cout<<"cannot find requested target grasp/workspace"<<std::endl;
        return false;
    }
    //Send a plan request to the multiobject planner
    
    dual_manipulation_shared::planner_serviceRequest msg;
    msg.command="plan";
    msg.source.grasp_id=source_grasp_id;
    msg.source.workspace_id=source_workspace_id;
    msg.destination.grasp_id=target_grasp_id;
    msg.destination.workspace_id=target_workspace_id;
    msg.filtered_source_nodes=filtered_arcs.filtered_source_nodes;
    msg.filtered_target_nodes=filtered_arcs.filtered_target_nodes;
    msg.object_id=obj.id;
    msg.priority=priority;
    //TODO filtered arcs?
    plan_request_pub.publish(msg);
    
    //Wait for a plan response by sleeping and checking a variable set in a callback
    while(ros::ok())
    {
        ros::spinOnce();
        usleep(10000);
        if (plan_received)
        {
            break;
        }
    }
    path=last_plan_received;
    plan_received=false;
    return true;//returning false does not make sense, the solution is not reachable
}

bool dual_manipulation::planner::planner_lib::barrier()
{
    barrier_received=false;
    
    //Send a message to wait for a synchronization, block until a "go" or a "replan" arrive on barrier_callback
    std_msgs::Header msg;
    msg.seq=priority;
    multiplan_barrier_pub.publish(msg);
    while(ros::ok())
    {
        ros::spinOnce();
        usleep(10000);
        if (barrier_received)
        {
            return barrier_ok;
        }
    }
}


bool dual_manipulation::planner::planner_lib::plan(grasp_id source_grasp_id, workspace_id source_workspace_id, grasp_id target_grasp_id, workspace_id target_workspace_id, std::vector<dual_manipulation_shared::planner_item> & path)
{
    path.clear();
    ros::Time start = ros::Time::now();
//     std::cout<<"planning from "<<source_grasp_id<<" in workspace "<<source_workspace_id<<" to "<<target_grasp_id<<" in workspace "<<target_workspace_id<<std::endl;
    lemon::SmartDigraph::Node source, target;
    if (!getNode(source_grasp_id,source_workspace_id,source))
    {
        std::cout<<"cannot find requested source grasp/workspace"<<std::endl;
        return false;
    }
    if (!getNode(target_grasp_id,target_workspace_id,target)) 
    {
        std::cout<<"cannot find requested target grasp/workspace"<<std::endl;
        return false;
    }
    int distance;
    ros::Time before = ros::Time::now();
    bool reached = lemon::dijkstra (lemon::filterArcs<lemon::SmartDigraph>(graph, *arc_filter), length ).path ( computed_path ).dist ( distance ).run ( source, target );
    ros::Time after = ros::Time::now();
    std::cout<<(after-before).toSec()<<std::endl;
    for ( lemon::PathNodeIt<lemon::Path<lemon::SmartDigraph> > i ( graph, computed_path ); i != lemon::INVALID; ++i )
    {
        dual_manipulation_shared::planner_item temp;
        temp.grasp_id=grasps_ids[i];
        temp.workspace_id=grasps_positions[i];
        path.push_back(temp);
//         std::cout<<temp.grasp_id<<" "<<temp.workspace_id<<std::endl;
    }
    if (reached)
    {
//         std::cout<<"path found"<<std::endl;
    }
    else
        std::cout<<"could not find a valid path"<<std::endl;
    ros::Time end = ros::Time::now();
//     std::cout<<(end-start).toSec()<<std::endl;
    return reached;
}
