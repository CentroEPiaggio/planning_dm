#ifndef DUAL_MANIPULATION_PLANNER_LIB
#define DUAL_MANIPULATION_PLANNER_LIB
#include "graph_creator.h"
#include <dual_manipulation_shared/planner_item.h>


namespace dual_manipulation
{
namespace planner
{

class planner_lib : protected graphCreator
{

public:

    planner_lib();
    bool set_object(object_id id, std::string name);

    void clear_filtered_arcs();

    /**
    * @brief You can add filtered arcs progressively as long as you don't want to reset the planner to the full graph status
    * 
    * @return void
    */
    void add_filtered_arc(grasp_id source_grasp_id, workspace_id source_workspace_id, grasp_id target_grasp_id, workspace_id target_workspace_id);


    bool plan(grasp_id source_grasp_id, workspace_id source_workspace_id, grasp_id target_grasp_id, workspace_id target_workspace_id, std::vector< dual_manipulation_shared::planner_item >& path);

    void draw_path();
    ~planner_lib(){}

private:

    lemon::SmartDigraph::ArcMap<bool>* arc_filter;
    Object obj;
    lemon::Path<lemon::SmartDigraph> computed_path;
//     lemon::SmartDigraph real_graph;

};



}
}


#endif //DUAL_MANIPULATION_PLANNER_LIB