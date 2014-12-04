#ifndef GRAPH_CREATOR_H
#define GRAPH_CREATOR_H

#include <lemon/smart_graph.h>
#include <lemon/path.h>
#include <vector>
#include <lemon/random.h>

#define XYOFFSET 80
#define VERTICAL_LENGTH 100000
#define TOP_FLOOR_LENGTH 500

#define MAXFLOORS 8
#define FLOORS_SENT 6
#define MAX_LENGTH 2*VERTICAL_LENGTH //TODO



class graphCreator
{
    const std::string left_workspace="left_workspace";
    const std::string right_workspace="right_workspace";
    const std::string shared_workspace="shared_workspace";
    
};










class Graph_creator
{
public:	
	Graph_creator(lemon::SmartDigraph &graph,lemon::SmartDigraph::ArcMap<int> &length,
    lemon::SmartDigraph::NodeMap<int> &coord_x,lemon::SmartDigraph::NodeMap<int> &coord_y);
	int createGraph(int floors, std::string graphName);
private:
	unsigned int graph_node_size;
	unsigned int floors;
	lemon::SmartDigraph graph;
	lemon::SmartDigraph &_3Dgraph;
	lemon::SmartDigraph::ArcMap<int> &_3Dlength;
    lemon::SmartDigraph::NodeMap<int> &_3Dcoord_x, &_3Dcoord_y;
 
    lemon::SmartDigraph::ArcMap<int> length;
    lemon::SmartDigraph::NodeMap<int> coord_x, coord_y;
	
	bool parseGraph(std::string graphName);
	void addFloor(lemon::SmartDigraph::NodeMap< lemon::dim2::Point< int > >& coords,
					lemon::SmartDigraph::NodeMap< int >& ncolors, lemon::SmartDigraph::ArcMap< int >& acolors,
					int startId);
	void finalizeFloor(lemon::SmartDigraph::NodeMap<lemon::dim2::Point<int> >& coords,
	lemon::SmartDigraph::NodeMap<int>& ncolors,
	lemon::SmartDigraph::ArcMap<int>& acolors,int startId);
	void addNodes(lemon::SmartDigraph::NodeMap<lemon::dim2::Point<int> >& coords,
				  lemon::SmartDigraph::NodeMap<int>& ncolors,lemon::SmartDigraph::ArcMap<int>& acolors,unsigned int floorNumber);
};

#endif // GRAPH_CREATOR_H
