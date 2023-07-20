#ifndef WAY_BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_VALUE_H_
#define WAY_BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_VALUE_H_

#include <behaviortree_cpp_v3/action_node.h>

namespace way_behavior_tree
{

using BT::NodeStatus;

//-------------------------------------------------------------
// Simple Behavior to print a number
//-------------------------------------------------------------

class PrintValue : public BT::SyncActionNode
{
public:
  PrintValue(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
  {
  }

  // You must override the virtual function tick()
  NodeStatus tick() override;

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
      return{ BT::InputPort<int>("message") };
  }
};

}

#endif  // WAY_BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_VALUE_H_
