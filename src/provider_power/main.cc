// ROS includes
#include "provider_power_node.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "provider_power_node");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    //ros::NodeHandle n;
    provider_power::ProviderPowerNode ppn(nh);
    //ros::Timer timer = n.createTimer(ros::Duration(0.1), ppn.wattCallBack);
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ppn.PublishPowerData();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
