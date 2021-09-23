/**
 * \file	provider_power_node.cc
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

#include "provider_power_node.h"

namespace provider_power {
//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
    ProviderPowerNode::ProviderPowerNode(ros::NodeHandlePtr &nh)
            : nh_(nh) 
    {

        power_publisher_ = nh_->advertise<sonia_common::PowerMsg>("/provider_power/power", 100);

        power_publisherRx_ = nh_->advertise<sonia_common::SendRS485Msg>("/interface_rs485/dataRx", 100);

        power_subscriberTx_ = nh_->subscribe("/interface_rs485/dataTx", 100, &ProviderPowerNode::PowerDataCallBack, this);

        activate_all_ps_ = nh_->subscribe("/provider_power/activate_all_ps", 100, &ProviderPowerNode::ActivateAllPsCallBack, this);
    }

//------------------------------------------------------------------------------
//
    ProviderPowerNode::~ProviderPowerNode() {

        power_subscriberTx_.shutdown();

    }

//==============================================================================
// M E T H O D   S E C T I O N

    void ProviderPowerNode::Spin(){
        ros::Rate r(1); // 5 hz

        while(ros::ok())
        {
            ros::spinOnce();
            ObtainPowerData();
            r.sleep();
        }
    }

    void ProviderPowerNode::ObtainPowerData() 
    {
        for(int i = 0; i < 3; ++i)
        {
            pollCmd(sonia_common::SendRS485Msg::SLAVE_PSU0, swapCmd[i]);
            ros::Duration(0.3).sleep();
        }
    }

    void ProviderPowerNode::PowerDataCallBack(const sonia_common::SendRS485Msg::ConstPtr &receiveData) 
    {
        sonia_common::PowerMsg msg;

        if (receiveData->slave == sonia_common::SendRS485Msg::SLAVE_PSU0) {
            powerData value;
            uint8_t size_array = receiveData->data.size();

            msg.array.data.clear();

            msg.slave = receiveData->slave;
            msg.cmd = receiveData->cmd;

            if(msg.cmd == sonia_common::SendRS485Msg::CMD_READ_MOTOR || msg.cmd == sonia_common::SendRS485Msg::CMD_ACT_MOTOR)
            {

                for(uint8_t i = 0; i < size_array; ++i)
                {
                    msg.array.data.push_back(receiveData->data[i]);
                }
            }
            else
            {
                for(uint8_t i = 0; i < size_array/4; ++i) // shifting of 4 for each data
                {
                    value.Bytes[0] = receiveData->data[4*i];
                    value.Bytes[1] = receiveData->data[4*i+1];
                    value.Bytes[2] = receiveData->data[4*i+2];
                    value.Bytes[3] = receiveData->data[4*i+3];

                    msg.array.data.push_back(value.info);
                }
                msg.array.layout.dim.push_back(std_msgs::MultiArrayDimension());

                if(msg.cmd == sonia_common::SendRS485Msg::CMD_CURRENT)
                {
                    msg.array.layout.dim[0].label = currentString;
                }
                else
                {
                    msg.array.layout.dim[0].label = voltageString;
                }
            }

            power_publisher_.publish(msg);
        }
    }
    
    void ProviderPowerNode::ActivateAllPsCallBack(const sonia_common::ActivateAllPS::ConstPtr &receiveData)
    {
        sonia_common::SendRS485Msg enablePower;

        enablePower.slave = sonia_common::SendRS485Msg::SLAVE_PSU0;
        enablePower.cmd = sonia_common::SendRS485Msg::CMD_ACT_MOTOR;

        for(uint8_t i =0; i < nb_motor; ++i)
        {
            enablePower.data.push_back(receiveData->data);
        }

        power_publisherRx_.publish(enablePower);
    }

    void ProviderPowerNode::pollCmd(uint8_t slave, uint8_t cmd) 
    {
        sonia_common::SendRS485Msg messageData;

        messageData.slave = slave;
        messageData.cmd = cmd;
        messageData.data.push_back(0x00);

        power_publisherRx_.publish(messageData);
    }
}
