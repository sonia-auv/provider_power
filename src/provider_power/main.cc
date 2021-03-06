// ROS includes
#include "provider_power_node.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "provider_power_node");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    provider_power::ProviderPowerNode ppn(nh);
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ppn.PublishPowerData();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
