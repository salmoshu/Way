#include "way_behavior_tree/plugins/condition/is_map_valid_condition.h"

namespace way_behavior_tree
{

IsMapValidCondition::IsMapValidCondition(const std::string& name) :
  BT::ConditionNode::ConditionNode(name, {})
{
  expected_result_ = BT::NodeStatus::SUCCESS;
  tick_count_ = 0;
}

BT::NodeStatus IsMapValidCondition::tick()
{
    ros::Duration du(3);
    du.sleep();

    tick_count_++;
    return expected_result_;
}

void IsMapValidCondition::setExpectedResult(BT::NodeStatus res)
{
  expected_result_ = res;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<way_behavior_tree::IsMapValidCondition>("IsMapValid");
}