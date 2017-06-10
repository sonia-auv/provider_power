// ROS includes
#include "provider_power_node.h"
#include <thread>


int main(int argc, char **argv) {
    ros::init(argc, argv, "provider_power_node");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    provider_power::ProviderPowerNode ppn(nh);
    std::thread publishData(&provider_power::ProviderPowerNode::PollAllPs, ppn);
    ros::spin();
    publishData.join();
    return 0;
}
