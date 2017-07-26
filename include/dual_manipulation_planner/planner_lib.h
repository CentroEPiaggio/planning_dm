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