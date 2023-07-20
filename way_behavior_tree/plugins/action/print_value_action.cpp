#include "way_behavior_tree/plugins/action/print_value_action.h"
#include <ros/ros.h>

using namespace BT;
//-------------------------------------------------------------
// Simple Behavior to print a number
//-------------------------------------------------------------

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
namespace way_behavior_tree
{

BT::NodeStatus PrintValue::tick()
{
    ros::Duration du(3);
    du.sleep();
    auto msg = getInput<std::string>("message");
    if (!msg)
    {
        throw BT::RuntimeError( "missing required input [message]: ", msg.error() );
    }

    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

} // namespace way_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<way_behavior_tree::PrintValue>("PrintValue");
  
}