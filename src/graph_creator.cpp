#include "dual_manipulation_planner/graph_creator.h"

#include <lemon/adaptors.h>
#include <lemon/dijkstra.h>
#include <lemon/concepts/maps.h>
#include <lemon/graph_to_eps.h>
#include <dual_manipulation_shared/object.h>
#include <dual_manipulation_shared/databasemapper.h>
#include "ros/package.h"
#include <dual_manipulation_shared/graph.h>
#include <dual_manipulation_shared/stream_utils.h>
using namespace std;
using namespace dual_manipulation::planner;


graphCreator::graphCreator(int x, int offx, int y, int offy)
:public_graph(graph),grasps_ids(graph),grasps_positions(graph),coords(graph),length(graph),ncolors(graph),nshapes(graph),x(x),y(y),offx(offx),offy(offy),grasps_texts(graph)
{
    std::string path=ros::package::getPath("dual_manipulation_planner");
    img_path=path+"/image.eps";
    graph_publisher=node.advertise<dual_manipulation_shared::graph>("computed_graph",100,this);
}

graphCreator::graphCreator():graphCreator(300,8,300,6)
{
}

bool graphCreator::getNode(grasp_id graspId, workspace_id workspaceId, lemon::SmartDigraphBase::Node& node)
{
    if (!nodeIds.count(graspId) || !nodeIds[graspId].count(workspaceId)) return false;
    node = graph.nodeFromId(nodeIds[graspId][workspaceId]);
    return true;
}

bool graphCreator::getArc(grasp_id s_graspId, workspace_id s_workspaceId,grasp_id t_graspId, workspace_id t_workspaceId, lemon::SmartDigraphBase::Arc& arc)
{
    lemon::SmartDigraph::Node s_node,t_node;
//     lemon::SmartDigraph::Arc temp;
    if (!nodeIds.count(s_graspId) || !nodeIds[s_graspId].count(s_workspaceId)) return false;
    s_node = graph.nodeFromId(nodeIds[s_graspId][s_workspaceId]);
    if (!nodeIds.count(t_graspId) || !nodeIds[t_graspId].count(t_workspaceId)) return false;
    t_node = graph.nodeFromId(nodeIds[t_graspId][t_workspaceId]);
    arc = lemon::findArc(graph,s_node,t_node);
//     if (graph.id(s_node)==45 && graph.id(t_node)==7) std::cout<<"22 - 3, 4 - 3 ->asfas 45 7"<<" "<<graph.id(temp);
//     arc=temp,
//     lemon::findArc(graph,s_node,t_node,temp);
//     assert(temp==lemon::INVALID);
    if (arc==lemon::INVALID) return false;
    return true;
}

bool graphCreator::create_graph(Object obj)
{
    using namespace lemon;
    graph.clear();
    graph_as_map.clear();
    //Ask to Grasp_DB the list of objects
    //Check if obj is in the list
    if (!database.Objects.count(obj.id))
        return false;
    //Ask to Grasp_DB the grasps for obj
    //Ask to Grasp_DB the transitions grasps for obj/grasp
    //Fill internal maps with database data

    int i=0;
    //Create nodes in each workspace for each grasp
    for ( auto grasp:database.Grasps )
    {
        if (std::get<0>(grasp.second)!=obj.id) continue;
        for (auto workspace:database.Workspaces)
        {
            endeffector_id eeId= std::get<1>(grasp.second);
            //if ee associated to grasp is reachable in the workspace
            if(database.Reachability[eeId].count(workspace.first))
            {
                SmartDigraph::Node n = graph.addNode();
                ncolors[n]=eeId+1%3;
                nshapes[n]=eeId%3;
                grasps_ids[n] = grasp.first;
                grasps_texts[n] = graph.id(n);
                grasps_positions[n] = workspace.first;
//                 coords[n].x=workspace.first*x+grasp.first*offx;
//                 coords[n].y=eeId*y-grasp.first*offy;
                nodeIds[grasp.first][workspace.first]=graph.id(n);
                graph_as_map[eeId][workspace.first].push_back(make_pair(grasp.first,graph.id(n)));
            }
        }
    }
    int max_ee_id=0;
    for (auto ee: database.EndEffectors)
    {
        if (ee.first>max_ee_id)
            max_ee_id=ee.first;
    }
    auto x=2*this->x;
    int offsetx = max_ee_id*x/4;
    //Choose appropriate coordinates
    for (auto ee: graph_as_map)
    {
        for (auto workspace: ee.second)
        {
            int num_grasps = workspace.second.size();
            int i=0;
            double spacex = double(x/2)/((num_grasps+2));
            double spacey = double(y/2)/((num_grasps+2));
            for (auto grasp: workspace.second)
            {
                coords[graph.nodeFromId(grasp.second)].x=offsetx+ workspace.first*x+pow(-1,ee.first)*ee.first*x/4+(num_grasps-i)*spacex;
                coords[graph.nodeFromId(grasp.second)].y=(ee.first+1)*y-i*spacey;
                i++;
            }
        }
    }

    //For each node in the graph with id ID, connect with all the nodes with ID in the map transition_grasps and adjacent workspace
    for (SmartDigraph::NodeIt n(graph); n!=INVALID; ++n) //for all the sources
    {
        for (SmartDigraph::NodeIt ntarget(graph); ntarget!=INVALID; ++ntarget) //for all the targets
        {
            transition_info t_info;
            if(database.getTransitionInfo(object_state(grasps_ids[n],grasps_positions[n]),object_state(grasps_ids[ntarget],grasps_positions[ntarget]),t_info))
            {
                SmartDigraph::Arc a=graph.addArc ( n,ntarget );
                length[a] = t_info.transition_cost_;
            }
        }
    }
    Palette p;
  
    lemon::graphToEps<lemon::SmartDigraph> ( graph,img_path ).
    coords ( coords ).
    nodeColors ( composeMap ( p,ncolors ) ).
    nodeShapes(nshapes).
//    arcColors ( composeMap ( p,acolors ) ).
    nodeTexts ( grasps_ids ).
//     nodeTexts( grasps_texts).
    nodeTextSize ( 4 ).
    nodeScale ( 0.008 ).
    arcWidthScale ( 0.0008 ).
    drawArrows ( true ).
    arrowWidth ( 3 ).
    arrowLength ( 5 ).
//     enableParallel ( true ).
//     enableParallel().parArcDist(1.5).
    distantColorNodeTexts().
    run();
   
    dual_manipulation_shared::graph message;
    for (lemon::SmartDigraph::NodeIt i(graph); i!=lemon::INVALID; ++i)
    {
        message.node_id.push_back(graph.id(i));
        message.x.push_back(coords[i].x);
        message.y.push_back(coords[i].y);
        message.text.push_back(std::to_string(grasps_ids[i]));
//         message.text.push_back(std::to_string(graph.id(i)));
        
    }
    for ( lemon::SmartDigraph::ArcIt i ( graph ); i != lemon::INVALID; ++i )
    {
        message.source.push_back(graph.id(graph.source(i)));
        message.target.push_back(graph.id(graph.target(i)));
    }
    graph_publisher.publish(message);
}

bool graphCreator::find_path(const lemon::SmartDigraph::ArcMap< bool >& arc_filter, const lemon::SmartDigraph::Node& source, const lemon::SmartDigraph::Node& target, int& distance, lemon::Path< lemon::SmartDigraph >& computed_path)
{
    bool reached = lemon::dijkstra (lemon::filterArcs<lemon::SmartDigraph>(graph, arc_filter), length ).path ( computed_path ).dist ( distance ).run ( source, target );
    return reached;
}


void graphCreator::draw_path(const lemon::Path< lemon::SmartDigraph >& computed_path, const lemon::SmartDigraph::ArcMap< bool >& arc_filter)
{
    using namespace lemon;

    dual_manipulation_shared::graph message;
    for (lemon::SmartDigraph::NodeIt i(graph); i!=lemon::INVALID; ++i)
    {
        message.node_id.push_back(graph.id(i));
        message.x.push_back(coords[i].x);
        message.y.push_back(coords[i].y);
        message.text.push_back(std::to_string(grasps_ids[i]));
        //         message.text.push_back(std::to_string(graph.id(i)));
    }
    for ( lemon::SmartDigraph::ArcIt i ( graph ); i != lemon::INVALID; ++i )
    {
	if(!arc_filter[i])
    {
        message.filtered_source.push_back(graph.id(graph.source(i)));
        message.filtered_target.push_back(graph.id(graph.target(i)));
        continue;
    }
	
        message.source.push_back(graph.id(graph.source(i)));
        message.target.push_back(graph.id(graph.target(i)));
    }
    for ( lemon::PathNodeIt<lemon::Path<lemon::SmartDigraph> > i ( graph, computed_path ); i != lemon::INVALID; ++i )
    {
        message.path_node_ids.push_back(graph.id(i));
    }
    graph_publisher.publish(message);
}

