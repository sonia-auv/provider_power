// ROS includes
#include "provider_power_node.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "provider_power_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
  provider_power::ProviderPowerNode ppn(nh);
  while (ros::ok()) {
    ppn.PublishPowerMsg();
    ros::spinOnce();
  }
  return 0;
}
