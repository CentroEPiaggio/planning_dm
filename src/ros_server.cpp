#include "ros_server.h"
#include <graph_creator.h>
#include <lemon/adaptors.h>
#include <lemon/smart_graph.h>
#include <lemon/dijkstra.h>
#include <lemon/path.h>

#include <dual_manipulation_shared/planner_serviceRequest.h>
#include <dual_manipulation_shared/planner_serviceResponse.h>

using namespace dual_manipulation::planner;


ros_server::ros_server():graph_creator(graph)
{
    service = node.advertiseService("planner_ros_service", &ros_server::planner_ros_service, this);
}

ros_server::ros_server(int x,int offx,int y,int offy):graph_creator(graph,x,offx,y,offy)
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
        std::cout<<"planning from "<<req.source.grasp_id<<" in workspace "<<req.source.workspace_id<<" to "<<req.destination.grasp_id<<" in workspace "<<req.destination.workspace_id<<std::endl;
        lemon::SmartDigraph::Node source, target;
        if (!graph_creator.getNode(req.source.grasp_id,req.source.workspace_id,source))
        {
            std::cout<<"cannot find requested source grasp/workspace"<<std::endl;
            return false;
        }
        if (!graph_creator.getNode(req.destination.grasp_id,req.destination.workspace_id,target)) 
        {
            std::cout<<"cannot find requested target grasp/workspace"<<std::endl;
            return false;
        }
        lemon::SmartDigraph::ArcMap<bool> arc_filter(graph,true);
        if (req.filtered_source_nodes.size()!=req.filtered_target_nodes.size())
        {
            std::cout<<"requested a filtering but source and target nodes vectors were not the same size!"<<std::endl;
            return false;
        }
        for (int i=0;i<req.filtered_source_nodes.size();i++)
        {
            lemon::SmartDigraph::Arc to_be_filtered_arc;
            if (graph_creator.getArc(req.filtered_source_nodes[i].grasp_id,req.filtered_source_nodes[i].workspace_id,
                                     req.filtered_source_nodes[i].grasp_id,req.filtered_source_nodes[i].workspace_id,
                                     to_be_filtered_arc))
            {
                arc_filter[to_be_filtered_arc]=false;
            }
        }
        lemon::Path<lemon::SmartDigraph> computed_path;
        int distance;
        bool reached = lemon::dijkstra (lemon::filterArcs<lemon::SmartDigraph>(graph, arc_filter), graph_creator.length ).path ( computed_path ).dist ( distance ).run ( source, target );
        for ( lemon::PathNodeIt<lemon::Path<lemon::SmartDigraph> > i ( graph, computed_path ); i != lemon::INVALID; ++i )
        {
            dual_manipulation_shared::planner_item temp;
            temp.grasp_id=graph_creator.grasps_ids[i];
            temp.workspace_id=graph_creator.grasps_positions[i];
            res.path.push_back(temp);
            std::cout<<temp.grasp_id<<" "<<temp.workspace_id<<std::endl;
            graph_creator.draw_path(computed_path);
        }
        res.ack=reached;
    }
    return true;
}

ros_server::~ros_server()
{
    service.shutdown();
}