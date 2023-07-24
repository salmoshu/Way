#include "way_behavior_tree/plugins/action/move_a2b_action.h"

using namespace BT;

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time

namespace way_behavior_tree
{

bool MoveA2BAction::sendGoal(GoalType& msg)
{
  // Take the goal from the InputPort of the Node
  Pose2D goal;

  if (!getInput<Pose2D>("goal", goal)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input [goal]");
  }

  // Reset this flag
  _aborted = false;

  ROS_INFO("Sending goal %f %f %f %f", goal.x, goal.y, goal.quaternion_z, goal.quaternion_w);

  system("rosrun dynamic_reconfigure dynparam set /move_base base_global_planner navfn/NavfnROS");

  // Build the message from Pose2D
  msg.target_pose.header.frame_id = "map";
  msg.target_pose.header.stamp = ros::Time::now();
  msg.target_pose.pose.position.x = goal.x;
  msg.target_pose.pose.position.y = goal.y;
  msg.target_pose.pose.orientation.z = goal.quaternion_z;
  msg.target_pose.pose.orientation.w = goal.quaternion_w;

  return true;
}

NodeStatus MoveA2BAction::onResult( const ResultType& res)
{
    ROS_INFO("MoveA2BAction: result received");
    return NodeStatus::SUCCESS;
}

NodeStatus MoveA2BAction::onFailedRequest(FailureCause failure)
{
    ROS_ERROR("MoveA2BAction request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
}

void MoveA2BAction::halt()
{
    if( status() == NodeStatus::RUNNING )
    {
        ROS_WARN("MoveA2BAction halted");
        BaseClass::halt();
    }
}

} // namespace way_behavior_tree


// #include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  RegisterRosAction<way_behavior_tree::MoveA2BAction>(factory, "MoveA2B");
}