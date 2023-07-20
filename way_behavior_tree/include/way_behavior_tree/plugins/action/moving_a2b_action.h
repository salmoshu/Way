#ifndef WAY_BEHAVIOR_TREE__PLUGINS__ACTION__MOVING_A2B_H_
#define WAY_BEHAVIOR_TREE__PLUGINS__ACTION__MOVING_A2B_H_

#include <behaviortree_cpp_v3/action_node.h>

#include <way_behavior_tree/bt_action_node.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>

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

class MovingA2BAction: public RosActionNode<move_base_msgs::MoveBaseAction>
{

public:
  MovingA2BAction( const std::string& name, const NodeConfiguration & conf):
RosActionNode<move_base_msgs::MoveBaseAction>(name, conf) {}

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

}

#endif  // WAY_BEHAVIOR_TREE__PLUGINS__ACTION__MOVING_A2B_H_
