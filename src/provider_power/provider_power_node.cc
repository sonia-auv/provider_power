/**
 * \file	sonar_node.cc
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

#include "provider_power_node.h"

namespace provider_power {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ProviderPowerNode::ProviderPowerNode(ros::NodeHandlePtr &nh)
      : nh_(nh){

  power_publisher_ =
            nh_->advertise<provider_power::powerMsg>("/provider_power/power", 10);

  power_publisherRx_ =
            nh_->advertise<interface_rs485::SendRS485Msg>("/interface_rs485/dataRx", 10);

  power_subscriberTx_ =
                nh_->subscribe("/interface_rs485/dataTx", 100, &ProviderPowerNode::PowerDataCallBack, this);

  power_serviceServer_  = nh_->advertiseService("/provider_power/manage_power_supply_bus",
                                                &ProviderPowerNode::powerServer, this);

}

//------------------------------------------------------------------------------
//
ProviderPowerNode::~ProviderPowerNode() {

        power_subscriberTx_.shutdown();

}

//==============================================================================
// M E T H O D   S E C T I O N

void ProviderPowerNode::PublishPowerMsg(const interface_rs485::SendRS485Msg::ConstPtr& publishData){

    provider_power::powerMsg msg;

    uint16_t DataMsg;
    powerData data;

    data.Bytes[1] = publishData->data[0];
    data.Bytes[0] = publishData->data[1];

    DataMsg = data.fraction;

    msg.slave = publishData->slave;
    msg.cmd = publishData->cmd;
    msg.Data = DataMsg;

    power_publisher_.publish(msg);

}


void ProviderPowerNode::PublishPowerData(){

    interface_rs485::SendRS485Msg pollingSlave;


    pollPower(pollingSlave.SLAVE_powersupply0);
    pollPower(pollingSlave.SLAVE_powersupply1);


}

void ProviderPowerNode::PowerDataCallBack(const interface_rs485::SendRS485Msg::ConstPtr& receiveData){

        if(receiveData->slave == receiveData->SLAVE_powersupply0 or receiveData->slave == receiveData->SLAVE_powersupply1){

            ProviderPowerNode::PublishPowerMsg(receiveData);

        }

}

bool ProviderPowerNode::powerServer(provider_power::ManagePowerSupplyBus::Request  &req,
                                    provider_power::ManagePowerSupplyBus::Response &res){

    interface_rs485::SendRS485Msg enablePower;

    static uint8_t swapSlave[4] = {enablePower.SLAVE_powersupply0, enablePower.SLAVE_powersupply1
                            , enablePower.SLAVE_powersupply2 , enablePower.SLAVE_powersupply3};
    static uint8_t swapCmd[3]   = {enablePower.CMD_PS_ACT_12V, enablePower.CMD_PS_ACT_16V_1,
                                   enablePower.CMD_PS_ACT_16V_2};

    enablePower.slave = swapSlave[req.slave];
    enablePower.cmd = swapCmd[req.bus];
    enablePower.data.push_back(req.state);

    power_publisherRx_.publish(enablePower);

    return true;

}

void ProviderPowerNode::pollPower(uint8_t slave){

    interface_rs485::SendRS485Msg pollingCmd;

    pollCmd(slave, pollingCmd.CMD_PS_V16_1);
    pollCmd(slave, pollingCmd.CMD_PS_V16_2);
    pollCmd(slave, pollingCmd.CMD_PS_V12);
    pollCmd(slave, pollingCmd.CMD_PS_C16_1);
    pollCmd(slave, pollingCmd.CMD_PS_C16_2);
    pollCmd(slave, pollingCmd.CMD_PS_C12);
    pollCmd(slave, pollingCmd.CMD_PS_temperature);
    pollCmd(slave, pollingCmd.CMD_PS_VBatt);

}

void ProviderPowerNode::pollCmd(uint8_t slave, uint8_t cmd){

    interface_rs485::SendRS485Msg messageData;

    messageData.slave = slave;
    messageData.cmd =  cmd;
    messageData.data.push_back(0x00);

    power_publisherRx_.publish(messageData);

}


}
