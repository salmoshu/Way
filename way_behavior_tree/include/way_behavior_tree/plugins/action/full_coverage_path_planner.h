#ifndef WAY_BEHAVIOR_TREE__PLUGINS__ACTION__MOVING_A2B_H_
#define WAY_BEHAVIOR_TREE__PLUGINS__ACTION__MOVING_A2B_H_

#include <behaviortree_cpp_v3/action_node.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include <way_behavior_tree/bt_action_node.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>


#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ipa_building_msgs/RoomExplorationAction.h>
#include <ipa_room_exploration/dynamic_reconfigure_client.h>
#include <ipa_room_exploration/timer.h>
#include <Eigen/Dense>

// Custom type
struct Pose2D
{
  double x, y, quaternion_z, quaternion_w;
};

// This template specialization is needed only if you want
// to AUTOMATICALLY convert a NodeParameter into a Pose2D
// In other words, implement it if you want to be able to do:
//
//   TreeNode::getInput<Pose2D>(key, ...)
//
template <> inline
Pose2D BT::convertFromString(BT::StringView key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');

  if (parts.size() != 4)
  {
    throw BT::RuntimeError("invalid input)");
  }
  else
  {
    Pose2D output;
    output.x     = convertFromString<double>(parts[0]);
    output.y     = convertFromString<double>(parts[1]);
    output.quaternion_z = convertFromString<double>(parts[2]);
    output.quaternion_w = convertFromString<double>(parts[3]);
    return output;
  }
}

using namespace BT;

namespace way_behavior_tree
{

class FullCoveragePathPlanner: public RosActionNode<ipa_building_msgs::RoomExplorationAction>
{

public:
  FullCoveragePathPlanner( const std::string& name, const NodeConfiguration & conf):
RosActionNode<ipa_building_msgs::RoomExplorationAction>(name, conf) {}

  static PortsList providedPorts()
  {
    return{ BT::InputPort<Pose2D>("goal") };
  }

  bool sendGoal(GoalType& goal) override;
  NodeStatus onResult( const ResultType& res) override;
  virtual NodeStatus onFailedRequest(FailureCause failure) override;
  void halt() override;

private:
  bool _aborted;
};

inline void RegisterNodes(BehaviorTreeFactory& factory)
{
  RegisterRosAction<FullCoveragePathPlanner>(factory, "FullCoveragePathPlanner");
}

}

#endif  // WAY_BEHAVIOR_TREE__PLUGINS__ACTION__MOVING_A2B_H_
