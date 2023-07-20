#include "way_behavior_tree/plugins/condition/is_battery_low_condition.h"

namespace way_behavior_tree
{

IsBatteryLowCondition::IsBatteryLowCondition(const std::string& name) :
  BT::ConditionNode::ConditionNode(name, {})
{
  expected_result_ = BT::NodeStatus::SUCCESS;
  tick_count_ = 0;
}

BT::NodeStatus IsBatteryLowCondition::tick()
{
    ros::Duration du(3);
    du.sleep();

    tick_count_++;
    return expected_result_;
}

void IsBatteryLowCondition::setExpectedResult(BT::NodeStatus res)
{
  expected_result_ = res;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<way_behavior_tree::IsBatteryLowCondition>("IsBatteryLow");
}