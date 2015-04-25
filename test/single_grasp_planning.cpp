#include "ros/ros.h"
#include <std_msgs/String.h>
#include <iostream>
#include "dual_manipulation_shared/planner_service.h"
#include <dual_manipulation_shared/object.h>
#include <dual_manipulation_shared/databasemapper.h>

/**
 * @brief service planner_service:
 * string source_workspace_name
 * uint64_t source_grasp_id
 * string target_workspace_name
 * uint64_t target_grasp_id
 * float64 time
 *   ---
 *   bool ack
 */



namespace dual_manipulation
{
    namespace planner
    {
        
        class fake_ros_server
        {
        public:
            fake_ros_server(grasp_id my_grasp_id):my_grasp_id(my_grasp_id)
            {
                service = node.advertiseService("planner_ros_service", &fake_ros_server::planner_ros_service, this);
            }
            ~fake_ros_server(){    service.shutdown();   }
        private:
            ros::NodeHandle node;
            
            ros::ServiceServer service_server;
            
            /**
             * This will return the following plan:
             * - source grasp in source workspace (from table)
             * - ee_grasp (command line parameter) in source workspace
             * - ee_grasp in target workspace
             * - target grasp in target workspace
             */
            bool planner_ros_service(dual_manipulation_shared::planner_service::Request &req, dual_manipulation_shared::planner_service::Response &res)
            {
                if (req.command=="set_object" || req.command=="set object" || req.command=="setObject")
                {
                    std::cout<<"object set to "<<req.object_id<<" with name "<<req.object_name<<std::endl;
                    obj.id=req.object_id;
                    res.ack=true;
                }
                else if (req.command=="plan" || req.command=="Plan")
                {
                    std::cout<<"planning from "<<req.source.grasp_id<<" in workspace "<<req.source.workspace_id<<" to "<<req.destination.grasp_id<<" in workspace "<<req.destination.workspace_id<<std::endl;
                    std::cout<<"ignoring the check if source or target exists"<<std::endl;
                    std::cout<<"ignoring filtered arcs"<<std::endl;

                    dual_manipulation_shared::planner_item temp;
                    temp.grasp_id=req.source.grasp_id;
                    temp.workspace_id=req.source.workspace_id;
                    res.path.push_back(temp);
                    temp.grasp_id=my_grasp_id;
                    temp.workspace_id=req.source.workspace_id;
                    res.path.push_back(temp);
                    temp.grasp_id=my_grasp_id;
                    temp.workspace_id=req.destination.workspace_id;
                    res.path.push_back(temp);
                    temp.grasp_id=req.destination.grasp_id;
                    temp.workspace_id=req.destination.workspace_id;
                    res.path.push_back(temp);
                    std::cout<<temp.grasp_id<<" "<<temp.workspace_id<<std::endl;

                    res.ack=true;
                    res.status="path found";
                }
                return true;
            }
            ros::ServiceServer service;
            Object obj;
            grasp_id my_grasp_id;
        };
    }
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> single_grasp_test "<<std::endl;
    std::cout<<std::endl;
    if (argc != 2)
    {
        std::cout<<"usage: \"simple_grasp_test grasp_id\" where grasp id is the integer id of the grasp to be tested"<<std::endl;
        return 0;
    }
    ros::init(argc, argv, "single_grasp_test");
    int grasp_id=atoi(argv[1]);
    std::cout<<"using grasp: "<<grasp_id<<std::endl;
    dual_manipulation::planner::fake_ros_server* server = new dual_manipulation::planner::fake_ros_server(grasp_id);
    ros::spin();
    
    return 0;
}