#include <ros/ros.h>
#include <ros/package.h>
#include "way_behavior_tree/bt_engine.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"


using namespace BT;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_robot");
  // ros::NodeHandle nh;

  BehaviorTreeFactory factory_;
  BehaviorTreeEngine(factory_);

  std::string pkg_path = ros::package::getPath("way_behavior_tree");

  // auto tree = factory_.createTreeFromFile(pkg_path + "/trees/simple_robot_tree.xml");
  // auto tree = factory_.createTreeFromFile(pkg_path + "/trees/move_robot.xml");
  auto tree = factory_.createTreeFromFile(pkg_path + "/trees/follow_path.xml");
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
  BT::PublisherZMQ publisher_zmq(tree);
  BT::StdCoutLogger logger_cout(tree);

  NodeStatus status = NodeStatus::IDLE;

  // while( ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
  while( ros::ok())
  {
    ros::spinOnce();
    status = tree.tickRoot();
    // std::cout << status << std::endl;
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  return 0;
}
