/**
 * \file	provier_power.h
 * \author	Olivier Lavoie <olavoie0795@gmail.com>
 * \date	03/2017
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
//#include <geometry_msgs/Twist.h>
#include <sonia_common/PowerMsg.h>
//#include <sonia_common/PowerInfo.h>
#include <sonia_common/SendRS485Msg.h>
//#include <sonia_common/ManagePowerSupplyBus.h>
#include <sonia_common/ActivateAllPS.h>
#include <condition_variable>
#include <mutex>

namespace provider_power {

    class ProviderPowerNode {
    public:
        //============================================================================
        // P U B L I C   C / D T O R S
        ProviderPowerNode(ros::NodeHandlePtr &nh);
        ~ProviderPowerNode();

        //==========================================================================
        // P U B L I C   M E T H O D S
        void Spin();

    private:
        //============================================================================
        // P R I V A T E   M E T H O D S
        
        void PublishPowerMsg(const sonia_common::SendRS485Msg::ConstPtr &publishData);

        void ObtainPowerData();

        void PowerDataCallBack(const sonia_common::SendRS485Msg::ConstPtr &receiveData);

        void ActivateAllPsCallBack(const sonia_common::ActivateAllPS::ConstPtr &receiveData);

        /*bool powerActivation(sonia_common::ManagePowerSupplyBus::Request &req,
                         sonia_common::ManagePowerSupplyBus::Response &res);*/

        void powerActivation(uint8_t slave, uint8_t cmd, uint8_t state);

        void pollPower(uint8_t slave);

        void pollCmd(uint8_t slave, uint8_t cmd);

        uint8_t swapCmd[3] = {0,1,3}; // Voltage, current, motor read
        std::string voltageArray[10] = {"V_M1", "V_M2", "V_M3", "V_M4", "V_M5", "V_M6", "V_M7", "V_M8", "V_BAT1", "V_BAT2"};
        std::string currentArray[10] = {"C_M1", "C_M2", "C_M3", "C_M4", "C_M5", "C_M6", "C_M7", "C_M8", "C_BAT1", "C_BAT2"};

        ros::NodeHandlePtr nh_;
        ros::Publisher power_publisher_;
        ros::Publisher power_publisherRx_;
        //ros::Publisher power_publisherInfo_;
        ros::Subscriber power_subscriberTx_;
        ros::Subscriber activate_all_ps_;
        ros::ServiceServer power_activation_;

        //const uint8_t next = 3;
        //const uint16_t convert = 1000;

        union powerData {
            uint8_t Bytes[4];
            float_t info;
        };

        uint8_t nb_sensor = 10;
        uint8_t nb_motor = 8;
        uint8_t salve_received;
        uint8_t cmd_received;

        std::condition_variable cv;
        std::mutex mtx;
    };
}  // namespace provider_power


#endif //PROVIDER_POWER_PROVIDER_POWER_NODE_H