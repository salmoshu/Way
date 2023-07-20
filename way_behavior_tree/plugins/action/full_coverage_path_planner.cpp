#include "way_behavior_tree/plugins/action/full_coverage_path_planner.h"

using namespace BT;

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time

BT_REGISTER_NODES(factory)
{
  way_behavior_tree::RegisterNodes(factory);
}

namespace way_behavior_tree
{

bool FullCoveragePathPlanner::sendGoal(GoalType& msg)
{
  bool use_test_maps = true;
  double resolution = 0.1;
  std::vector<double> origin (3,0);
  double robot_radius = 0.2;
  double coverage_radius = 1.0;
  std::vector<double> start_pos = {0, 0, 0};


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

NodeStatus FullCoveragePathPlanner::onResult( const ResultType& res)
{
    ROS_INFO("FullCoveragePathPlanner: result received");
    return NodeStatus::SUCCESS;
}

NodeStatus FullCoveragePathPlanner::onFailedRequest(FailureCause failure)
{
    ROS_ERROR("FullCoveragePathPlanner request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
}

void FullCoveragePathPlanner::halt()
{
    if( status() == NodeStatus::RUNNING )
    {
        ROS_WARN("FullCoveragePathPlanner halted");
        BaseClass::halt();
    }
}

} // namespace way_behavior_tree
