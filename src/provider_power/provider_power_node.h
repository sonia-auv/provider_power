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
#include <geometry_msgs/Twist.h>
#include <sonia_common/PowerMsg.h>
#include <sonia_common/PowerInfo.h>
#include <sonia_common/SendRS485Msg.h>
#include <sonia_common/ManagePowerSupplyBus.h>
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
        
        void PublishPowerMsg(const sonia_common::SendRS485Msg::ConstPtr &publishData);

        void PublishPowerData();

        void PowerDataCallBack(const sonia_common::SendRS485Msg::ConstPtr &receiveData);

        void ActivateAllPsCallBack(const sonia_common::ActivateAllPS::ConstPtr &receiveData);

        bool powerActivation(sonia_common::ManagePowerSupplyBus::Request &req,
                         sonia_common::ManagePowerSupplyBus::Response &res);

        void powerActivation(uint8_t slave, uint8_t cmd, uint8_t state);

        void pollPower(uint8_t slave);

        void pollCmd(uint8_t slave, uint8_t cmd);

        uint8_t swapSlave[4] = {sonia_common::SendRS485Msg::SLAVE_powersupply0, sonia_common::SendRS485Msg::SLAVE_powersupply1,
                                       sonia_common::SendRS485Msg::SLAVE_powersupply2, sonia_common::SendRS485Msg::SLAVE_powersupply3};
        uint8_t swapCmdAct[3] = {sonia_common::SendRS485Msg::CMD_PS_ACT_12V, sonia_common::SendRS485Msg::CMD_PS_ACT_16V_1,
                                     sonia_common::SendRS485Msg::CMD_PS_ACT_16V_2};
        uint8_t swapCmd[11] = {sonia_common::SendRS485Msg::CMD_PS_V16_1, sonia_common::SendRS485Msg::CMD_PS_V16_2,
                                sonia_common::SendRS485Msg::CMD_PS_V12, sonia_common::SendRS485Msg::CMD_PS_C16_1, sonia_common::SendRS485Msg::CMD_PS_C16_2,
                                sonia_common::SendRS485Msg::CMD_PS_C12, sonia_common::SendRS485Msg::CMD_PS_temperature, sonia_common::SendRS485Msg::CMD_PS_VBatt,
                                sonia_common::SendRS485Msg::CMD_PS_ACT_12V, sonia_common::SendRS485Msg::CMD_PS_ACT_16V_1, sonia_common::SendRS485Msg::CMD_PS_ACT_16V_2};

        ros::NodeHandlePtr nh_;
        ros::Publisher power_publisher_;
        ros::Publisher power_publisherRx_;
        //ros::Publisher power_publisherInfo_;
        ros::Subscriber power_subscriberTx_;
        ros::Subscriber activate_all_ps_;
        ros::ServiceServer power_activation_;

        const uint8_t next = 3;
        const uint16_t convert = 1000;

        union powerData {
            uint8_t Bytes[4];
            float info;
        };

        uint16_t nbTime[4];
    };
}  // namespace provider_power


#endif //PROVIDER_POWER_PROVIDER_POWER_NODE_H