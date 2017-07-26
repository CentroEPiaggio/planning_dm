/*********************************************************************
*
* Software License Agreement (BSD License)
*
*
*  Copyright (c) 2016, Hamal Marino <hamal dot marino at gmail dot com>
*  Copyright (c) 2017, George Jose Pollayil <gpollayil at gmail dot com>
*  Copyright (c) 2017, Mathew Jose Pollayil <mathewjosepollayil at gmail dot com>
*  Copyright (c) 2017, Centro di Ricerca "E. Piaggio", University of Pisa
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

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
#include <ros/node_handle.h>

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
protected:
    lemon::SmartDigraph graph;
    
public:
    graphCreator();
    graphCreator(int x,int offx,int y,int offy);
    /**
     * @brief Creates a graph of all the possible interactions between an object \param obj and the grasps
     * 
     * @param obj the object used in this graph
     * @return true on success, false otherwise
     */
    bool create_graph(Object obj);
    
    bool getNode(grasp_id graspId, workspace_id workspaceId,lemon::SmartDigraph::Node& node);
    bool getArc(grasp_id s_graspId, workspace_id s_workspaceId,grasp_id t_graspId, workspace_id t_workspaceId, lemon::SmartDigraphBase::Arc& arc);

    void draw_path(const lemon::Path< lemon::SmartDigraph >& computed_path, const lemon::SmartDigraph::ArcMap< bool >& arc_filter);
    const lemon::SmartDigraph & public_graph;

    bool find_path(const lemon::SmartDigraph::ArcMap<bool>& arc_filter, const lemon::SmartDigraph::Node& source,
                   const lemon::SmartDigraph::Node& target, int& distance, lemon::Path<lemon::SmartDigraph>& computed_path);
    
    inline void getNodeInfo(lemon::SmartDigraph::Node i, grasp_id& g_id, workspace_id& w_id) const
    {
        g_id=grasps_ids[i];
        w_id=grasps_positions[i];
    }
    
    inline bool isEndEffectorMovable(endeffector_id ee_id) const
    {
        return database.EndEffectors.at(ee_id).movable;
    }
    
protected:
    lemon::SmartDigraph::ArcMap<int> length;
    lemon::SmartDigraph::NodeMap<int> grasps_ids;
    lemon::SmartDigraph::NodeMap<int> grasps_texts;
    lemon::SmartDigraph::NodeMap<int> grasps_positions;
    lemon::SmartDigraph::NodeMap<int> ncolors;
    lemon::SmartDigraph::NodeMap<int> nshapes;
protected:
    unsigned int graph_node_size;
    databaseMapper database;
    ros::NodeHandle node;
    ros::Publisher graph_publisher;
    lemon::SmartDigraph::NodeMap<lemon::dim2::Point<int>> coords;
    int x;int offx;int y;int offy;
    std::map<grasp_id,std::map<workspace_id,int>> nodeIds;
    std::map<endeffector_id, std::map<workspace_id, std::vector<std::pair<grasp_id, int>>>> graph_as_map;
    //lemon::SmartDigraph::NodeMap<int> coord_x, coord_y; //Future grasps_positions???
    std::string img_path;
    
private:
};
}
}

#endif