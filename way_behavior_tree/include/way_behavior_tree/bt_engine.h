#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

const std::vector<std::string> plugin_libraries = {
  /* action */
  "print_value_action",
  // "print_value_action2",
  "move_a2b_action",
  "move_base_action",
  
  /* condition */
  "is_battery_low_condition",
  "is_component_valid_condition",
  "is_map_valid_condition"
};

void BehaviorTreeEngine(BehaviorTreeFactory& factory_)
{
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    std::cout << "PLUGIN: " << p << std::endl;
    factory_.registerFromPlugin(loader.getOSName(p));
  }
}