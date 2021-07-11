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
            : nh_(nh) {

        power_publisher_ =
                nh_->advertise<sonia_common::PowerMsg>("/provider_power/power", 10);

        power_publisherRx_ =
                nh_->advertise<sonia_common::SendRS485Msg>("/interface_rs485/dataRx", 10);

        power_subscriberTx_ =
                nh_->subscribe("/interface_rs485/dataTx", 100, &ProviderPowerNode::PowerDataCallBack, this);

        activate_all_ps_ =
                nh_->subscribe("/provider_power/activate_all_ps", 100, &ProviderPowerNode::ActivateAllPsCallBack, this);

        power_activation_ = nh_->advertiseService("/provider_power/manage_power_supply_bus",
                                                     &ProviderPowerNode::powerActivation, this);
    }

//------------------------------------------------------------------------------
//
    ProviderPowerNode::~ProviderPowerNode() {

        power_subscriberTx_.shutdown();

    }

//==============================================================================
// M E T H O D   S E C T I O N

    void ProviderPowerNode::Spin(){
        ros::Rate r(2); // 2 hz

        while(ros::ok())
        {
            ros::spinOnce();
            PublishPowerData();
        }
        r.sleep();
    }

    void ProviderPowerNode::PublishPowerMsg(const sonia_common::SendRS485Msg::ConstPtr &publishData) {

        sonia_common::PowerMsg msg;

        powerData data;

        data.Bytes[0] = publishData->data[0];
        data.Bytes[1] = publishData->data[1];
        data.Bytes[2] = publishData->data[2];
        data.Bytes[3] = publishData->data[3];

        msg.slave = publishData->slave;
        //salve_received = msg.slave;
        msg.cmd = publishData->cmd;
        //cmd_received = msg.cmd;
        msg.data = data.info;

        if (publishData->cmd >= sonia_common::SendRS485Msg::CMD_PS_CHECK_12V and publishData->cmd <= sonia_common::SendRS485Msg::CMD_PS_CHECK_16V_2){

            msg.data = publishData->data[0];

        }

        power_publisher_.publish(msg);

        cv.notify_one();

        //salve_received = 0;
        //cmd_received = 0;

    }


    void ProviderPowerNode::PublishPowerData() {

        pollPower(sonia_common::SendRS485Msg::SLAVE_powersupply0);
        pollPower(sonia_common::SendRS485Msg::SLAVE_powersupply1);
        pollPower(sonia_common::SendRS485Msg::SLAVE_powersupply2);
        pollPower(sonia_common::SendRS485Msg::SLAVE_powersupply3);

    }

    void ProviderPowerNode::PowerDataCallBack(const sonia_common::SendRS485Msg::ConstPtr &receiveData) {

        if (receiveData->slave == receiveData->SLAVE_powersupply0 ||
            receiveData->slave == receiveData->SLAVE_powersupply1 ||
            receiveData->slave == receiveData->SLAVE_powersupply2 ||
            receiveData->slave == receiveData->SLAVE_powersupply3) {

            ProviderPowerNode::PublishPowerMsg(receiveData);
        }
    }

    bool ProviderPowerNode::powerActivation(sonia_common::ManagePowerSupplyBus::Request &req,
                                        sonia_common::ManagePowerSupplyBus::Response &res) {

        sonia_common::SendRS485Msg enablePower;

        enablePower.slave = swapSlave[req.slave];
        enablePower.cmd = swapCmd[req.bus];
        enablePower.data.push_back(req.state);

        power_publisherRx_.publish(enablePower);

        return true;

    }
    void ProviderPowerNode::powerActivation(uint8_t slave, uint8_t cmd, uint8_t state)
    {
        sonia_common::SendRS485Msg enablePower;

        enablePower.slave = slave;
        enablePower.cmd = cmd;
        enablePower.data.push_back(state);

        power_publisherRx_.publish(enablePower);
    }
    
    void ProviderPowerNode::ActivateAllPsCallBack(const sonia_common::ActivateAllPS::ConstPtr &receiveData)
    {
        int i,j;

        for(i = 1; i < 3; ++i)
        {
            for(j = 0; j < 4 ; ++j)
            {
                powerActivation(swapSlave[j], swapCmdAct[i], receiveData->data);
            }
        }
    }

    void ProviderPowerNode::pollPower(uint8_t slave) {

        for(int i = 0; i < 11; ++i)
        {
            //do {
                std::unique_lock<std::mutex> lck(mtx);
                pollCmd(slave, swapCmd[i]);
                cv.wait(lck);
            //} while(slave != salve_received || swapCmd[i] != cmd_received); // Verify that the cmd has been received before sending a new one

        }
    }

    void ProviderPowerNode::pollCmd(uint8_t slave, uint8_t cmd) {

        sonia_common::SendRS485Msg messageData;

        messageData.slave = slave;
        messageData.cmd = cmd;
        messageData.data.push_back(0x00);

        power_publisherRx_.publish(messageData);

    }
}
