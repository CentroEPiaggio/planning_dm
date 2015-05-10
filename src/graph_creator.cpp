#include "graph_creator.h"

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


graphCreator::graphCreator(lemon::SmartDigraph& graph, int x, int offx, int y, int offy)
:graph(graph),grasps_ids(graph),grasps_positions(graph),coords(graph),length(graph),ncolors(graph),nshapes(graph),x(x),y(y),offx(offx),offy(offy),grasps_texts(graph)
{
    std::string path=ros::package::getPath("dual_manipulation_planner");
    img_path=path+"/image.eps";
    graph_publisher=node.advertise<dual_manipulation_shared::graph>("computed_graph",1,this);
}

graphCreator::graphCreator(lemon::SmartDigraph& graph):graphCreator(graph,300,8,300,6)
{
}


void graphCreator::create_fake_map()
{
/*    Grasp temp;
    temp.name="test";
    grasps[0]=temp;
    temp.name="test1";
    grasps[1]=temp;
    temp.name="test2";
    grasps[2]=temp;
    transition_grasps[0].push_back(1);
//     transition_grasps[0].push_back(2);
//     transition_grasps[1].push_back(2);
//     transition_grasps[2].push_back(0);
    transition_grasps[2].push_back(1);*/
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
            if (grasps_ids[n]==grasps_ids[ntarget] && 
                std::get<1>(database.EndEffectors[std::get<1>(database.Grasps[grasps_ids[n]])]) && //E.E is movable
                database.WorkspacesAdjacency[grasps_positions[n]].count(grasps_positions[ntarget]))
            {
                SmartDigraph::Arc a=graph.addArc ( n,ntarget );
                length[a]=1;
                a=graph.addArc ( ntarget,n );
                length[a]=1;
            }
            else
            if (grasps_positions[n]==grasps_positions[ntarget] && database.Grasp_transitions[grasps_ids[n]].count(grasps_ids[ntarget])) 
            {
                SmartDigraph::Arc a=graph.addArc ( n,ntarget );
                length[a]=2;
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


void graphCreator::draw_path(const lemon::Path< lemon::SmartDigraph >& computed_path, const lemon::SmartDigraph::ArcMap< bool >& arc_filter)
{
    using namespace lemon;
    SmartDigraph::NodeMap<int> nlocalcolors(graph);
    lemon::SmartDigraph::ArcMap<lemon::Color> arccolors(graph);
    lemon::Palette p;
    for (lemon::SmartDigraph::NodeIt i(graph); i!=lemon::INVALID; ++i)
    {
        nlocalcolors[i]=ncolors[i];
    }
    for ( lemon::PathNodeIt<lemon::Path<lemon::SmartDigraph> > i ( graph, computed_path ); i != lemon::INVALID; ++i )
    {
        nlocalcolors[i]=5;
    }
    for ( lemon::SmartDigraph::ArcIt i ( graph ); i != lemon::INVALID; ++i )
    {
        arccolors[i].set(0,0,0);
    }
    for ( lemon::Path<lemon::SmartDigraph>::ArcIt i ( computed_path ); i != lemon::INVALID; ++i )
    {
        arccolors[i].set(1,0,0);
    }

    lemon::graphToEps<lemon::SmartDigraph> ( graph,img_path ).
    coords ( coords ).
    nodeColors ( lemon::composeMap ( p,nlocalcolors ) ).
    nodeShapes(nshapes).
    //    arcColors ( composeMap ( p,acolors ) ).
    nodeTexts ( grasps_ids ).
    nodeTextSize ( 4 ).
    nodeScale ( 0.008 ).
    arcWidthScale ( 0.0008 ).
    drawArrows ( true ).
    arrowWidth ( 3 ).
    arrowLength ( 5 ).
    preScale(false).
    scale(20).
    arcColors(arccolors).
        enableParallel ( true ).
//         enableParallel().parArcDist(1.5).
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
	if(!arc_filter[i])
	  continue;
	
        message.source.push_back(graph.id(graph.source(i)));
        message.target.push_back(graph.id(graph.target(i)));
    }
    for ( lemon::PathNodeIt<lemon::Path<lemon::SmartDigraph> > i ( graph, computed_path ); i != lemon::INVALID; ++i )
    {
        message.path_node_ids.push_back(graph.id(i));
    }
    graph_publisher.publish(message);
    
}










































/*int Graph_creator::createGraph ( int floors, string graphName )
{
    this->floors=floors;
    try
    {
        if (parseGraph ( graphName ))
	  return graph_node_size;
	else
	  return 0;
    }
    catch ( lemon::Exception const& e )
    {
        std::cout<< "Error during graph creation:"<<e.what()<<std::endl;
        return 0;
    }
}

Graph_creator::Graph_creator ( lemon::SmartDigraph &_3Dgraph,lemon::SmartDigraph::ArcMap<int> &length,
                               lemon::SmartDigraph::NodeMap<int> &coord_x,lemon::SmartDigraph::NodeMap<int> &coord_y ) :
    _3Dgraph ( _3Dgraph ),_3Dlength ( length ),_3Dcoord_x ( coord_x ),_3Dcoord_y ( coord_y ),length ( graph ),coord_x ( graph ),
    coord_y ( graph )
{
    floors=0;
    graph_node_size=0;
}

void Graph_creator::addNodes ( lemon::SmartDigraph::NodeMap<lemon::dim2::Point<int> >& coords,
                               lemon::SmartDigraph::NodeMap<int>& ncolors,lemon::SmartDigraph::ArcMap<int>& acolors,unsigned int floorNumber )
{
    using namespace lemon;
    for ( unsigned int i=0; i<graph_node_size; i++ )
    {
        SmartDigraph::Node n = _3Dgraph.addNode();
        assert ( _3Dgraph.id ( n ) == ( i+floorNumber*graph_node_size ) );
        ( _3Dcoord_x ) [n]= ( coord_x ) [graph.nodeFromId ( i )];
        ( _3Dcoord_y ) [n]= ( coord_y ) [graph.nodeFromId ( i )];
        coords[n]=dim2::Point<int> ( ( _3Dcoord_x ) [n]+ ( _3Dcoord_y ) [n]/3, ( _3Dcoord_y ) [n]/3+XYOFFSET* ( floorNumber+1 ) );
        ncolors[n]=floorNumber+1;
        if ( floorNumber>0 )
        {
            SmartDigraph::Arc a=_3Dgraph.addArc ( n,_3Dgraph.nodeFromId ( i ) );
            ( _3Dlength ) [a]=VERTICAL_LENGTH;
            acolors[a]=floors+3;
        }
    }
}

void Graph_creator::addFloor ( lemon::SmartDigraph::NodeMap<lemon::dim2::Point<int> >& coords,
                               lemon::SmartDigraph::NodeMap<int>& ncolors,
                               lemon::SmartDigraph::ArcMap<int>& acolors,int startId )
{
    using namespace lemon;
    unsigned int floorNumber=startId/graph_node_size;
    addNodes ( coords,ncolors,acolors,floorNumber );
    if ( floorNumber==0 ) return;
    SmartDigraph::Node source;
    SmartDigraph::Node target;
    for ( unsigned int i=0; i<graph_node_size; i++ )
    {
        for ( SmartDigraph::OutArcIt arcit ( graph,graph.nodeFromId ( i ) ); arcit!=INVALID; ++arcit )
        {
            for ( unsigned int j=0; j<MAX_ARC_FLOORS; j++ )
            {
                target=_3Dgraph.nodeFromId ( graph.id ( graph.target ( arcit ) ) +floorNumber*graph_node_size );
                if ( floorNumber>j )
                {
                    source=_3Dgraph.nodeFromId ( i+ ( floorNumber-1-j ) *graph_node_size );
                    SmartDigraph::Arc a=_3Dgraph.addArc ( source,target );
                    ( _3Dlength ) [a]= ( 1+j*1.1+0.1*floorNumber ) *sqrt ( ( double ) sqr ( _3Dcoord_x[source]-_3Dcoord_x[target] ) + ( double ) sqr ( _3Dcoord_y[source]-_3Dcoord_y[target] ) ); //1; prendo la distanza reale invece dell'unità
                    acolors[a]=floorNumber+1;
                }
            }
        }

    }

}

void Graph_creator::finalizeFloor ( lemon::SmartDigraph::NodeMap<lemon::dim2::Point<int> >& coords,
                                    lemon::SmartDigraph::NodeMap<int>& ncolors,
                                    lemon::SmartDigraph::ArcMap<int>& acolors,int startId )
{
    using namespace lemon;
    int floorNumber=startId/graph_node_size;
    addNodes ( coords,ncolors,acolors,floorNumber );
    SmartDigraph::Node target,source;
    for ( unsigned int i=0; i<graph_node_size; i++ )
    {
        for ( SmartDigraph::OutArcIt arcit ( graph,graph.nodeFromId ( i ) ); arcit!=INVALID; ++arcit )
        {
            SmartDigraph::Arc a=_3Dgraph.addArc ( _3Dgraph.nodeFromId ( i+floorNumber*graph_node_size ),_3Dgraph.nodeFromId ( graph.id ( graph.target ( arcit ) ) +floorNumber*graph_node_size ) );
            ( _3Dlength ) [a]=TOP_FLOOR_LENGTH; 
            acolors[a]=floorNumber+1;
            target=_3Dgraph.nodeFromId ( graph.id ( graph.target ( arcit ) ) +floorNumber*graph_node_size );
	    for (int j=1;j<MAX_ARC_FLOORS;j++)
	    {
	    source=_3Dgraph.nodeFromId ( i+ ( floorNumber-j ) *graph_node_size );
	    a=_3Dgraph.addArc (source,target );
            ( _3Dlength ) [a]=(1+1.1*j+0.1*floorNumber)*sqrt ( sqr ( _3Dcoord_x[source]-_3Dcoord_x[target] ) +sqr ( _3Dcoord_y[source]-_3Dcoord_y[target] ) ); //2;
            acolors[a]=floorNumber+1;
	    }
        }
    }
}

bool Graph_creator::parseGraph ( string graphName )
{
    using namespace lemon;
    boost::algorithm::to_lower ( graphName );

    try
    {
        digraphReader ( graph, graphName ). // read the directed graph into g
        nodeMap ( "coordinates_x", coord_x ).	//read the coordinates of nodes
        nodeMap ( "coordinates_y", coord_y ).	//read the coordinates of nodes
        arcMap ( "length", length ).    // read the 'capacity' arc map into cap
        run();
    }
    catch ( Exception& er )   // check if there was any error
    {
        std::cout<< "parsing exception"<<er.what()<<std::endl;
	return false;
    }
    graph_node_size=graph.nodeNum();
    SmartDigraph::NodeMap<dim2::Point<int> > coords ( _3Dgraph );
    SmartDigraph::NodeMap<int> ncolors ( _3Dgraph,1 );
    SmartDigraph::ArcMap<int> acolors ( _3Dgraph,1 );

    for ( unsigned int i=0; i<floors; i++ )
    {
        addFloor ( coords,ncolors,acolors,graph_node_size*i );
    }
    finalizeFloor ( coords,ncolors,acolors,graph_node_size*floors );
    IdMap<SmartDigraph,SmartDigraph::Node> id ( _3Dgraph );
    Palette p;
    lemon::graphToEps<lemon::SmartDigraph> ( _3Dgraph,"image.eps" ).
    coords ( coords ).
    nodeColors ( composeMap ( p,ncolors ) ).
    arcColors ( composeMap ( p,acolors ) ).
    nodeTexts ( id ).
    nodeTextSize ( 4 ).
    nodeScale ( 0.008 ).
    arcWidthScale ( 0.0008 ).
    drawArrows ( true ).
    arrowWidth ( 3 ).
    arrowLength ( 5 ).
    enableParallel ( true ).
    distantColorNodeTexts().
    run();

    std::cout << "A digraph is read from "<<graphName << std::endl;
    std::cout << "Number of nodes: " << lemon::countNodes ( _3Dgraph ) << std::endl;
    std::cout << "Number of arcs: " << lemon::countArcs ( _3Dgraph ) << std::endl;
    return true;
}
*/