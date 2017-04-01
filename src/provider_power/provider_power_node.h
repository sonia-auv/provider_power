/**
 * \file	sonar_node.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Pierluc Bédard <blindmiror@gmail.com>
 * \author  Francis Masse <francis.masse05@gmail.com>
 * \date	11/02/2016
 *
 * \copyright Copyright (c) 2016 Copyright (C) 2011 Randolph Voorhies
 *
 * \section LICENSE http://www.gnu.org/licenses/gpl-3.0.en.html
 *
 * Changes by: S.O.N.I.A.
 * \copyright Copyright (c) 2016 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PROVIDER_POWER_PROVIDER_POWER_NODE_H
#define PROVIDER_POWER_PROVIDER_POWER_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "provider_power/powerMsg.h"
#include <interface_rs485/SendRS485Msg.h>
#include <provider_power/ManagePowerSupplyBus.h>


namespace provider_power {

class ProviderPowerNode {
 public:

    union powerData {
        uint8_t Bytes[2];
        uint16_t fraction;
    };

  //============================================================================
  // P U B L I C   C / D T O R S

  ProviderPowerNode(ros::NodeHandlePtr &nh);

  ~ProviderPowerNode();

  //============================================================================
  // P U B L I C   M E T H O D S
    void PublishPowerMsg(const interface_rs485::SendRS485Msg::ConstPtr& publishData);
    void PublishPowerData();
    void PowerDataCallBack(const interface_rs485::SendRS485Msg::ConstPtr& receiveData);
    bool powerServer(provider_power::ManagePowerSupplyBus::Request  &req,
                     provider_power::ManagePowerSupplyBus::Response &res);

    void pollPower(uint8_t slave);
    void pollCmd(uint8_t slave, uint8_t cmd);




 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

    ros::NodeHandlePtr nh_;
    ros::Publisher  power_publisher_;
    ros::Publisher  power_publisherRx_;
    ros::Subscriber power_subscriberTx_;
    ros::ServiceServer power_serviceServer_;



};
}  // namespace provider_power

#endif //PROVIDER_POWER_PROVIDER_POWER_NODE_H