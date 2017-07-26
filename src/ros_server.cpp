#include "ros_server.h"
#include <dual_manipulation_planner/graph_creator.h>
#include <lemon/adaptors.h>
#include <lemon/smart_graph.h>
#include <lemon/dijkstra.h>
#include <lemon/path.h>

#include <dual_manipulation_shared/planner_serviceRequest.h>
#include <dual_manipulation_shared/planner_serviceResponse.h>

using namespace dual_manipulation::planner;


ros_server::ros_server():graph_creator()
{
    service = node.advertiseService("planner_ros_service", &ros_server::planner_ros_service, this);
}

ros_server::ros_server(int x,int offx,int y,int offy):graph_creator(x,offx,y,offy)
{
    service = node.advertiseService("planner_ros_service", &ros_server::planner_ros_service, this);
}

bool ros_server::planner_ros_service(dual_manipulation_shared::planner_service::Request &req, dual_manipulation_shared::planner_service::Response &res)
{
    if (req.command=="set_object" || req.command=="set object" || req.command=="setObject")
    {
        std::cout<<"object set to "<<req.object_id<<" with name "<<req.object_name<<std::endl;
        obj.id=req.object_id;
        graph_creator.create_graph(obj);
        res.ack=true;
    }
    else if (req.command=="plan" || req.command=="Plan")
    {
        ros::Time start = ros::Time::now();
        std::cout<<"planning from "<<req.source.grasp_id<<" in workspace "<<req.source.workspace_id<<" to "<<req.destination.grasp_id<<" in workspace "<<req.destination.workspace_id<<std::endl;
        lemon::SmartDigraph::Node source, target;
        if (!graph_creator.getNode(req.source.grasp_id,req.source.workspace_id,source))
        {
            std::cout<<"cannot find requested source grasp/workspace"<<std::endl;
            res.status="cannot find requested source grasp/workspace";
            return false;
        }
        if (!graph_creator.getNode(req.destination.grasp_id,req.destination.workspace_id,target)) 
        {
            std::cout<<"cannot find requested target grasp/workspace"<<std::endl;
            res.status="cannot find requested target grasp/workspace";
            return false;
        }
        lemon::SmartDigraph::ArcMap<bool> arc_filter(graph_creator.public_graph,true);
        if (req.filtered_source_nodes.size()!=req.filtered_target_nodes.size())
        {
            std::cout<<"requested a filtering but source and target nodes vectors were not the same size!"<<std::endl;
            res.status="requested a filtering but source and target nodes vectors were not the same size!";
            return false;
        }
        for (int i=0;i<req.filtered_source_nodes.size();i++)
        {
            lemon::SmartDigraph::Arc to_be_filtered_arc;
            if (graph_creator.getArc(req.filtered_source_nodes[i].grasp_id,req.filtered_source_nodes[i].workspace_id,
                                     req.filtered_target_nodes[i].grasp_id,req.filtered_target_nodes[i].workspace_id,
                                     to_be_filtered_arc))
            {
		std::cout << "Filtering arc between grasp:" << req.filtered_source_nodes[i].grasp_id << " in ws:" << req.filtered_source_nodes[i].workspace_id;
		std::cout << " and grasp:" << req.filtered_target_nodes[i].grasp_id << " in ws:" << req.filtered_target_nodes[i].workspace_id << std::endl;
                arc_filter[to_be_filtered_arc]=false;
            }
        }
        lemon::Path<lemon::SmartDigraph> computed_path;
        int distance;
        ros::Time before = ros::Time::now();
        bool reached = graph_creator.find_path(arc_filter,source,target,distance,computed_path);
        ros::Time after = ros::Time::now();
        std::cout<<(after-before).toSec()<<std::endl;
        // Checking if number of edges in path is zero. If so, creating an edge with same departing and arriving nodes.
        std::cout<<"Dijkstra computed a path long "<< computed_path.length() << " edges."<<std::endl;
        if(computed_path.length()==0 && req.source.workspace_id==req.destination.workspace_id && req.source.grasp_id==req.destination.grasp_id)
        {
            lemon::SmartDigraph::Arc temp_arc;
            if(graph_creator.getArc(req.source.grasp_id, req.source.workspace_id, req.destination.grasp_id, req.destination.workspace_id, temp_arc))
            {
                temp_arc = findArc(graph_creator.public_graph, source, target);
                computed_path.addFront(temp_arc);
            }
            else
            {
                res.status="could not find a valid path";
            }
        }
        for ( lemon::PathNodeIt<lemon::Path<lemon::SmartDigraph> > i ( graph_creator.public_graph, computed_path ); i != lemon::INVALID; ++i )
        {
            dual_manipulation_shared::planner_item temp;
            graph_creator.getNodeInfo(i,temp.grasp_id,temp.workspace_id);
            res.path.push_back(temp);
            std::cout<<temp.grasp_id<<" "<<temp.workspace_id<<std::endl;
        }
        graph_creator.draw_path(computed_path,arc_filter);
        res.ack=reached;
        if (reached)
            res.status="path found";
        else
            res.status="could not find a valid path";
        ros::Time end = ros::Time::now();
        std::cout<<(end-start).toSec()<<std::endl;
    }
    return true;
}

ros_server::~ros_server()
{
    service.shutdown();
}