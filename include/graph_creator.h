#ifndef GRAPH_CREATOR_H
#define GRAPH_CREATOR_H

#include <lemon/smart_graph.h>
#include <lemon/path.h>
#include <vector>
#include <map>
#include <lemon/random.h>
#include <dual_manipulation_shared/grasp.h>
#include <dual_manipulation_shared/object.h>
#include <dual_manipulation_shared/databasemapper.h>

namespace dual_manipulation
{
    namespace planner
{
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
    graphCreator(lemon::SmartDigraph& graph);

    /**
     * @brief Creates a graph of all the possible interactions between an object \param obj and the grasps
     * 
     * @param obj the object used in this graph
     * @return true on success, false otherwise
     */
    bool create_graph(Object obj);
    
    bool getNode(grasp_id graspId, workspace_id workspaceId,lemon::SmartDigraph::Node& node);
    
    void create_fake_map();
    lemon::SmartDigraph::ArcMap<int> length; //TODO make private
    lemon::SmartDigraph::NodeMap<int> grasps_ids;//TODO make private
    lemon::SmartDigraph::NodeMap<int> grasps_positions;//TODO make private
    
private:
    lemon::SmartDigraph& graph;
    unsigned int graph_node_size;
    databaseMapper database;
    
    lemon::SmartDigraph::NodeMap<lemon::dim2::Point<int>> coords;

    std::map<workspace_id,std::map<grasp_id,int>> nodeIds;
    //lemon::SmartDigraph::NodeMap<int> coord_x, coord_y; //Future grasps_positions???

};
}
}

#endif