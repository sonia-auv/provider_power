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
#include <sonia_common/PowerMsg.h>
#include <sonia_common/SendRS485Msg.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sonia_common/ActivateAllPS.h>

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

        void ObtainPowerData();

        void PowerDataCallBack(const sonia_common::SendRS485Msg::ConstPtr &receiveData);

        void ActivateAllPsCallBack(const sonia_common::ActivateAllPS::ConstPtr &receiveData);

        void pollCmd(uint8_t slave, uint8_t cmd);

        uint8_t swapCmd[3] = {sonia_common::SendRS485Msg::CMD_VOLTAGE,sonia_common::SendRS485Msg::CMD_CURRENT,
                                sonia_common::SendRS485Msg::CMD_READ_MOTOR}; // Voltage, current, motor read
        std::string voltageString = "Voltage_M1_M2_M3_M4_M5_M6_M7_M8_BAT1_BAT2";
        std::string currentString = "Current_M1_M2_M3_M4_M5_M6_M7_M8_BAT1_BAT2";

        ros::NodeHandlePtr nh_;
        ros::Publisher power_publisher_;
        ros::Publisher power_publisherRx_;

        ros::Subscriber power_subscriberTx_;
        ros::Subscriber activate_all_ps_;
        ros::ServiceServer power_activation_;

        union powerData {
            uint8_t Bytes[4];
            float_t info;
        };

        uint8_t nb_motor = 8;
    };
}  // namespace provider_power


#endif //PROVIDER_POWER_PROVIDER_POWER_NODE_H