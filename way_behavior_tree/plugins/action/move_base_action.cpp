#include "way_behavior_tree/plugins/action/move_base_action.h"

using namespace BT;

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time

namespace way_behavior_tree
{

bool MoveBaseAction::sendGoal(GoalType& msg)
{
    auto move_status = getInput<std::string>("goal");
    if (!move_status) {
        // if I can't get this, there is something wrong with your BT.
        // For this reason throw an exception instead of returning FAILURE
        throw BT::RuntimeError("missing required input [goal]");
    }

    // Reset this flag
    _aborted = false;

    if(move_status.value() == "A2B") {
        // ros::param::set("/movebase/base_global_planner", "navfn/NavfnROS");
        system("rosrun dynamic_reconfigure dynparam set /move_base base_global_planner navfn/NavfnROS");
        msg.target_pose.header.frame_id = "map";
        msg.target_pose.header.stamp = ros::Time::now();
        msg.target_pose.pose.position.x = 0;
        msg.target_pose.pose.position.y = 0;
        msg.target_pose.pose.position.z = 0;
        msg.target_pose.pose.orientation.x = 0;
        msg.target_pose.pose.orientation.y = 0;
        msg.target_pose.pose.orientation.z = 0;
        msg.target_pose.pose.orientation.w = 1;
    }else if(move_status.value() == "FOLLOWING") {
        // <param name="base_global_planner" value="simple_global_planner/SimpleGlobalPlanner"/>
        system("rosrun dynamic_reconfigure dynparam set /move_base base_global_planner simple_global_planner/SimpleGlobalPlanner");
        // ros::param::set("/movebase/base_global_planner", "simple_global_planner/SimpleGlobalPlanner");
        msg.target_pose.header.frame_id = "map";
        msg.target_pose.header.stamp = ros::Time::now();
        msg.target_pose.pose.position.x = -2.060713;
        msg.target_pose.pose.position.y = -0.094490;
        msg.target_pose.pose.position.z = 0.000106;
        msg.target_pose.pose.orientation.x = 0.000187;
        msg.target_pose.pose.orientation.y = -0.000108;
        msg.target_pose.pose.orientation.z = 0.414473;
        msg.target_pose.pose.orientation.w = 0.910062;
    }

    ROS_INFO("Sending goal %f %f %f %f %f %f %f", msg.target_pose.pose.position.x, 
                                                  msg.target_pose.pose.position.y,
                                                  msg.target_pose.pose.position.z,
                                                  msg.target_pose.pose.orientation.x,
                                                  msg.target_pose.pose.orientation.y,
                                                  msg.target_pose.pose.orientation.z,
                                                  msg.target_pose.pose.orientation.w);

    return true;
}

NodeStatus MoveBaseAction::onResult( const ResultType& res)
{
    ROS_INFO("MoveBaseAction: result received");
    return NodeStatus::SUCCESS;
}

NodeStatus MoveBaseAction::onFailedRequest(FailureCause failure)
{
    ROS_ERROR("MoveBaseAction request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
}

void MoveBaseAction::halt()
{
    if( status() == NodeStatus::RUNNING )
    {
        ROS_WARN("MoveBaseAction halted");
        BaseClass::halt();
    }
}

} // namespace way_behavior_tree


// #include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  RegisterRosAction<way_behavior_tree::MoveBaseAction>(factory, "MoveBase");
}