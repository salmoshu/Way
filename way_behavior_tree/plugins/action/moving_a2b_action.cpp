#include "way_behavior_tree/plugins/action/moving_a2b_action.h"

using namespace BT;

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time

namespace way_behavior_tree
{

bool MovingA2BAction::sendGoal(GoalType& msg)
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

  // Build the message from Pose2D
  msg.target_pose.header.frame_id = "map";
  msg.target_pose.header.stamp = ros::Time::now();
  msg.target_pose.pose.position.x = goal.x;
  msg.target_pose.pose.position.y = goal.y;
  msg.target_pose.pose.orientation.z = goal.quaternion_z;
  msg.target_pose.pose.orientation.w = goal.quaternion_w;

  return true;
}

NodeStatus MovingA2BAction::onResult( const ResultType& res)
{
    ROS_INFO("MovingA2BAction: result received");
    return NodeStatus::SUCCESS;
}

NodeStatus MovingA2BAction::onFailedRequest(FailureCause failure)
{
    ROS_ERROR("MovingA2BAction request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
}

void MovingA2BAction::halt()
{
    if( status() == NodeStatus::RUNNING )
    {
        ROS_WARN("MovingA2BAction halted");
        BaseClass::halt();
    }
}

} // namespace way_behavior_tree


// #include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  RegisterRosAction<way_behavior_tree::MovingA2BAction>(factory, "MovingA2B");
}