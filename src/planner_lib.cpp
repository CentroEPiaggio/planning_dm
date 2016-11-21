#include "dual_manipulation_planner/planner_lib.h"
#include <lemon/adaptors.h>
#include <lemon/dijkstra.h>
#include <dual_manipulation_shared/planner_item.h>

#define CLASS_NAMESPACE "planner_lib::"

dual_manipulation::planner::planner_lib::planner_lib():graphCreator()
{
}


void dual_manipulation::planner::planner_lib::draw_path()
{
    graphCreator::draw_path(computed_path,*arc_filter);
}

void dual_manipulation::planner::planner_lib::clear_filtered_arcs()
{
    for ( lemon::SmartDigraph::ArcIt i ( graph ); i != lemon::INVALID; ++i )
        (*arc_filter)[i]= true;
}

void dual_manipulation::planner::planner_lib::add_filtered_arc(grasp_id source_grasp_id, workspace_id source_workspace_id, grasp_id target_grasp_id, workspace_id target_workspace_id)
{
    lemon::SmartDigraph::Arc to_be_filtered_arc;
    if (getArc(source_grasp_id,source_workspace_id,
        target_grasp_id,target_workspace_id,
        to_be_filtered_arc))
    {
//         std::cout << "Filtering arc between grasp:" << source_grasp_id << " in ws:" << source_workspace_id;
//         std::cout << " and grasp:" << target_grasp_id << " in ws:" << target_workspace_id << std::endl;
        (*arc_filter)[to_be_filtered_arc]=false;
    }
}

bool dual_manipulation::planner::planner_lib::set_object(object_id id, std::string name)
{
    std::cout << CLASS_NAMESPACE << __func__ << " : object set to " << id << " with name " << name << std::endl;
    obj.id=id;
    create_graph(obj);
    arc_filter = new lemon::SmartDigraph::ArcMap<bool>(graph,true);
    return true;
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
