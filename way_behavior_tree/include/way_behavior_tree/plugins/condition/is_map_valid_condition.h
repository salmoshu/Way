#ifndef WAY_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_MAP_VALID_CONDITION_H_
#define WAY_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_MAP_VALID_CONDITION_H_

#include <ros/ros.h>
#include "behaviortree_cpp_v3/condition_node.h"

namespace way_behavior_tree
{

/**
 * @brief A BT::ConditionNode that monitor the map topic and
 * returns SUCCESS when map is invalid and FAILURE otherwise
 */
class IsMapValidCondition : public BT::ConditionNode
{
public:
  IsMapValidCondition(const std::string& name);

  void setExpectedResult(BT::NodeStatus res);

  // The method that is going to be executed by the thread
  virtual BT::NodeStatus tick() override;

  int tickCount() const
  {
    return tick_count_;
  }

private:
  BT::NodeStatus expected_result_;
  int tick_count_;
};


}  // namespace way_behavior_tree

#endif  // WAY_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_MAP_VALID_CONDITION_H_