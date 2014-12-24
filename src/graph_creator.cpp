#include "graph_creator.h"

#include <lemon/dijkstra.h>
#include <lemon/concepts/maps.h>
#include <lemon/graph_to_eps.h>
#include <lemon/color.cc>
#include <dual_manipulation_shared/object.h>
#include <dual_manipulation_shared/databasemapper.h>
using namespace std;
using namespace dual_manipulation::planner;


graphCreator::graphCreator(    lemon::SmartDigraph& graph
):graph(graph),grasps_ids(graph),grasps_positions(graph),coords(graph)
{
databaseMapper test;
}


void graphCreator::create_fake_map()
{
    Grasp temp;
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
    transition_grasps[2].push_back(1);
}

bool graphCreator::create_graph(Object obj)
{
    using namespace lemon;
    
    //Ask to Grasp_DB the list of objects
    //Check if obj is in the list
    //Ask to Grasp_DB the grasps for obj
    //Ask to Grasp_DB the transitions grasps for obj/grasp
    //Fill internal maps with database data
    create_fake_map();
    
    //Create nodes in each workspace for each grasp
    for ( unsigned int i=0; i<grasps.size(); i++ )
    {
        for (int j=0;j<WORKSPACES;j++)
        {
            SmartDigraph::Node n = graph.addNode();
            grasps_ids[n] = i;
            grasps_positions[n] = j;
            coords[n].x=j*100+i*10;
            coords[n].y=i*20;
        }
    }

    //For each node in the graph with id ID, connect with all the nodes with ID in the map transition_grasps and adjacent workspace
    for (SmartDigraph::NodeIt n(graph); n!=INVALID; ++n) 
    {
        for (auto target_grasp: transition_grasps[grasps_ids[n]])
        {
            for (SmartDigraph::NodeIt ntarget(graph); ntarget!=INVALID; ++ntarget) 
            {
                if ((grasps_ids[ntarget]==target_grasp) && (abs(grasps_positions[ntarget]-grasps_positions[n])<=1))
                {
                    SmartDigraph::Arc a=graph.addArc ( n,ntarget );
                }
                if (grasps_ids[n]==grasps_ids[ntarget] && (abs(grasps_positions[ntarget]-grasps_positions[n])<=1))
                    SmartDigraph::Arc a=graph.addArc ( n,ntarget );
            }
        }
    }
    
    lemon::graphToEps<lemon::SmartDigraph> ( graph,"image.eps" ).
    coords ( coords ).
//    nodeColors ( composeMap ( p,ncolors ) ).
//    arcColors ( composeMap ( p,acolors ) ).
    nodeTexts ( grasps_ids ).
    nodeTextSize ( 4 ).
    nodeScale ( 0.008 ).
    arcWidthScale ( 0.0008 ).
    drawArrows ( true ).
    arrowWidth ( 3 ).
    arrowLength ( 5 ).
//     enableParallel ( true ).
    enableParallel().parArcDist(1.5).
    distantColorNodeTexts().
    run();
    
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