cmake_minimum_required(VERSION 3.5.1) # version on Ubuntu Trusty
project(way_behavior_tree)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

######################################################

set(ROS_DEPENDENCIES
    roscpp
    rospy
    std_msgs
    actionlib
    actionlib_msgs
    behaviortree_cpp_v3
    # ipa_building_msgs
    # ipa_room_exploration
    message_generation
)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )
find_package(GTest)

add_message_files(
  FILES
  BehaviorTree.msg
  NodeParameter.msg
  NodeStatus.msg
  StatusChange.msg
  StatusChangeLog.msg
  TreeNode.msg
)

add_service_files(
   FILES
   AddTwoInts.srv
 )

add_action_files(
   DIRECTORY action
   FILES
   Fibonacci.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories( include ${catkin_INCLUDE_DIRS})

######################################################
# PLUGINS #
######################################################

### action: print_value
add_library(print_value_action SHARED
  include/way_behavior_tree/plugins/action/print_value_action.h
  plugins/action/print_value_action.cpp
)
add_dependencies(print_value_action ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(print_value_action PRIVATE ${BEHAVIOR_TREE_LIBRARY})
target_compile_definitions(print_value_action  PRIVATE BT_PLUGIN_EXPORT)

### action: move_a2b
add_library(move_a2b_action SHARED
  include/way_behavior_tree/plugins/action/move_a2b_action.h
  plugins/action/move_a2b_action.cpp
)
add_dependencies(move_a2b_action ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(move_a2b_action PRIVATE ${BEHAVIOR_TREE_LIBRARY} ${catkin_LIBRARIES})
target_compile_definitions(move_a2b_action  PRIVATE BT_PLUGIN_EXPORT)

### action: move_base
add_library(move_base_action SHARED
  include/way_behavior_tree/plugins/action/move_base_action.h
  plugins/action/move_base_action.cpp
)
add_dependencies(move_base_action ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(move_base_action PRIVATE ${BEHAVIOR_TREE_LIBRARY} ${catkin_LIBRARIES})
target_compile_definitions(move_base_action  PRIVATE BT_PLUGIN_EXPORT)

### action: follow_path
add_library(follow_path_action SHARED
  include/way_behavior_tree/plugins/action/follow_path_action.h
  plugins/action/follow_path_action.cpp
)
add_dependencies(follow_path_action ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(follow_path_action PRIVATE ${BEHAVIOR_TREE_LIBRARY} ${catkin_LIBRARIES})
target_compile_definitions(follow_path_action  PRIVATE BT_PLUGIN_EXPORT)

### condiction: is_battery_low
add_library(is_battery_low_condition SHARED
  include/way_behavior_tree/plugins/condition/is_battery_low_condition.h
  plugins/condition/is_battery_low_condition.cpp
)
add_dependencies(is_battery_low_condition ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(is_battery_low_condition PRIVATE ${BEHAVIOR_TREE_LIBRARY} ${catkin_LIBRARIES})
target_compile_definitions(is_battery_low_condition  PRIVATE BT_PLUGIN_EXPORT)

### condiction: is_component_valid
add_library(is_component_valid_condition SHARED
  include/way_behavior_tree/plugins/condition/is_component_valid_condition.h
  plugins/condition/is_component_valid_condition.cpp
)
add_dependencies(is_component_valid_condition ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(is_component_valid_condition PRIVATE ${BEHAVIOR_TREE_LIBRARY} ${catkin_LIBRARIES})
target_compile_definitions(is_component_valid_condition  PRIVATE BT_PLUGIN_EXPORT)

### condiction: is_map_valid
add_library(is_map_valid_condition SHARED
  include/way_behavior_tree/plugins/condition/is_map_valid_condition.h
  plugins/condition/is_map_valid_condition.cpp
)
add_dependencies(is_map_valid_condition ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(is_map_valid_condition PRIVATE ${BEHAVIOR_TREE_LIBRARY} ${catkin_LIBRARIES})
target_compile_definitions(is_map_valid_condition  PRIVATE BT_PLUGIN_EXPORT)

######################################################
# MAIN #
######################################################

### bt_engine
add_executable(bt_engine src/bt_engine.cpp)
add_dependencies(bt_engine ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(bt_engine ${catkin_LIBRARIES})