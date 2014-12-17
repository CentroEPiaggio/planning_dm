#ifndef GRAPH_CREATOR_H
#define GRAPH_CREATOR_H

#include <lemon/smart_graph.h>
#include <lemon/path.h>
#include <vector>
#include <map>
#include <lemon/random.h>
#include <dual_manipulation_shared/grasp.h>
#include <dual_manipulation_shared/object.h>

#define WORKSPACES 3

/**
 * @brief The class used to handle the generation of a graph for the planner
 * 
 */
class graphCreator
{
    const std::vector<std::string> workspaces = {"left_workspace","right_workspace","shared_workspace"};
    const std::string left_workspace="left_workspace";
    const std::string right_workspace="right_workspace";
    const std::string shared_workspace="shared_workspace";
public:
    std::map<int,Grasp> grasps;//TODO make private
    std::map<int,std::vector<int>> transition_grasps; //TODO make private

    graphCreator();

    /**
     * @brief Creates a graph of all the possible interactions between an object \param obj and the grasps
     * 
     * @param obj the object used in this graph
     * @return true on success, false otherwise
     */
    bool create_graph(Object obj);
    
    void create_fake_map();

private:
    lemon::SmartDigraph graph;
    unsigned int graph_node_size;

    //lemon::SmartDigraph::ArcMap<int> length; //Future arcs lengths

    lemon::SmartDigraph::NodeMap<int> grasps_ids;
    lemon::SmartDigraph::NodeMap<int> grasps_positions;
    lemon::SmartDigraph::NodeMap<lemon::dim2::Point<int>> coords;

    //lemon::SmartDigraph::NodeMap<int> coord_x, coord_y; //Future grasps_positions???

};








/*

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
*/
#endif // GRAPH_CREATOR_H
