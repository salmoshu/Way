#ifndef WAY_BEHAVIOR_TREE__PLUGINS__ACTION__MOVING_BASE_H_
#define WAY_BEHAVIOR_TREE__PLUGINS__ACTION__MOVING_BASE_H_

#include <behaviortree_cpp_v3/action_node.h>

#include <way_behavior_tree/bt_action_node.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>

using namespace BT;

namespace way_behavior_tree
{

class MovingBaseAction: public RosActionNode<move_base_msgs::MoveBaseAction>
{

public:
  MovingBaseAction( const std::string& name, const NodeConfiguration & conf):
RosActionNode<move_base_msgs::MoveBaseAction>(name, conf) {}

  static PortsList providedPorts()
  {
    return{ BT::InputPort<std::string>("goal") };
  }

  bool sendGoal(GoalType& goal) override;
  NodeStatus onResult( const ResultType& res) override;
  virtual NodeStatus onFailedRequest(FailureCause failure) override;
  void halt() override;

private:
  bool _aborted;
};

}

#endif  // WAY_BEHAVIOR_TREE__PLUGINS__ACTION__MOVING_BASE_H_
