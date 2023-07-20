#include <pluginlib/class_list_macros.h>
#include "way_coverage_planning/simple_global_planner.h"

#define PLAN_FILE ros::package::getPath("way_mapping") + "/map/simple_path.csv"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(simple_global_planner::SimpleGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace simple_global_planner {

    SimpleGlobalPlanner::SimpleGlobalPlanner (){
        
    }

    SimpleGlobalPlanner::SimpleGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }


    void SimpleGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        
    }

    bool SimpleGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
        geometry_msgs::PoseStamped plan_pose;
        plan_pose.header.frame_id = "map";

        std::ifstream ifs;
        std::string line;
        ifs.open(PLAN_FILE, std::ios::in);
        if(!ifs) {
            ROS_ERROR("Open file error.");
            exit(1);
        }

        while (getline(ifs, line))
        {
            istringstream sin(line);
            vector<string> fields;
            string field;
            while (getline(sin, field, ',')) 
            {
                fields.push_back(field);
            }
            // string number = fields[0];
            // string project_name = fields[1];
            plan_pose.header.stamp = ros::Time::now();
            plan_pose.pose.position.x = std::stod(fields[0]);
            plan_pose.pose.position.y = std::stod(fields[1]);
            plan_pose.pose.position.z = std::stod(fields[2]);
            plan_pose.pose.orientation.x = std::stod(fields[3]);
            plan_pose.pose.orientation.y = std::stod(fields[4]);
            plan_pose.pose.orientation.z = std::stod(fields[5]);
            plan_pose.pose.orientation.w = std::stod(fields[6]);

            plan.push_back(plan_pose);
        }

        return true;
    }
};