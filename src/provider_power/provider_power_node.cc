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
                nh_->advertise<provider_power::powerMsg>("/provider_power/power", 10);

        power_publisherRx_ =
                nh_->advertise<interface_rs485::SendRS485Msg>("/interface_rs485/dataRx", 10);

        power_publisherInfo_ =
                nh_->advertise<provider_power::powerInfo>("/provider_power/powerInfo", 10);

        power_subscriberTx_ =
                nh_->subscribe("/interface_rs485/dataTx", 100, &ProviderPowerNode::PowerDataCallBack, this);

        activate_all_ps_ =
                nh_->subscribe("/provider_power/activate_all_ps", 100, &ProviderPowerNode::ActivateAllPsCallBack, this);

        power_activation_ = nh_->advertiseService("/provider_power/manage_power_supply_bus",
                                                     &ProviderPowerNode::powerActivation, this);

        ProviderPowerNode::initialize();

        //timerForWatt_ = nh_->createTimer(ros::Duration(1.0), &ProviderPowerNode::wattCallBack, true);

    }

//------------------------------------------------------------------------------
//
    ProviderPowerNode::~ProviderPowerNode() {

        power_subscriberTx_.shutdown();

    }

//==============================================================================
// M E T H O D   S E C T I O N

    void ProviderPowerNode::PublishPowerMsg(const interface_rs485::SendRS485Msg::ConstPtr &publishData) {

        provider_power::powerMsg msg;

        powerData data;

        data.Bytes[0] = publishData->data[0];
        data.Bytes[1] = publishData->data[1];
        data.Bytes[2] = publishData->data[2];
        data.Bytes[3] = publishData->data[3];



        msg.slave = publishData->slave;
        msg.slave -= interface_rs485::SendRS485Msg::SLAVE_powersupply0;
        msg.cmd = publishData->cmd;
        msg.data = data.info;

        if (msg.data >= 0 && msg.data < 1){

            msg.data = 0;

        }

        if (publishData->cmd >= interface_rs485::SendRS485Msg::CMD_PS_CHECK_12V and publishData->cmd <= interface_rs485::SendRS485Msg::CMD_PS_CHECK_16V_2){

            msg.data = publishData->data[0];

        }

        power_publisher_.publish(msg);

    }


    void ProviderPowerNode::PublishPowerData() {

        pollPower(interface_rs485::SendRS485Msg::SLAVE_powersupply0);
        pollPower(interface_rs485::SendRS485Msg::SLAVE_powersupply1);
        pollPower(interface_rs485::SendRS485Msg::SLAVE_powersupply2);
        pollPower(interface_rs485::SendRS485Msg::SLAVE_powersupply3);


    }

    void ProviderPowerNode::PowerDataCallBack(const interface_rs485::SendRS485Msg::ConstPtr &receiveData) {

        if (receiveData->slave == receiveData->SLAVE_powersupply0 ||
            receiveData->slave == receiveData->SLAVE_powersupply1 ||
            receiveData->slave == receiveData->SLAVE_powersupply2 ||
            receiveData->slave == receiveData->SLAVE_powersupply3) {

            ProviderPowerNode::PublishPowerMsg(receiveData);

        }

    }

    bool ProviderPowerNode::powerActivation(provider_power::ManagePowerSupplyBus::Request &req,
                                        provider_power::ManagePowerSupplyBus::Response &res) {

        interface_rs485::SendRS485Msg enablePower;

        enablePower.slave = swapSlave[req.slave];
        enablePower.cmd = swapCmd[req.bus];
        enablePower.data.push_back(req.state);

        power_publisherRx_.publish(enablePower);

        return true;

    }

    void  ProviderPowerNode::powerCheckActivation() {
        ros::Rate rate(10);
        uint8_t i = interface_rs485::SendRS485Msg::SLAVE_powersupply0;

        while (i <= interface_rs485::SendRS485Msg::SLAVE_powersupply3){

            pollCmd(i, interface_rs485::SendRS485Msg::CMD_PS_CHECK_12V);
            rate.sleep();
            pollCmd(i, interface_rs485::SendRS485Msg::CMD_PS_CHECK_16V_1);
            rate.sleep();
            pollCmd(i, interface_rs485::SendRS485Msg::CMD_PS_CHECK_16V_2);
            rate.sleep();

            i++;

        }

    }

    void ProviderPowerNode::ActivateAllPsCallBack(const provider_power::activateAllPS::ConstPtr &receiveData){

        interface_rs485::SendRS485Msg enablePower;

        enablePower.slave = swapSlave[receiveData->slave];
        enablePower.cmd = swapCmd[receiveData->bus];
        enablePower.data.push_back(receiveData->data);

        power_publisherRx_.publish(enablePower);

    }

    void ProviderPowerNode::pollPower(uint8_t slave) {
        ros::Rate rate(8);

        pollCmd(slave, interface_rs485::SendRS485Msg::CMD_PS_V16_1);
        rate.sleep();
        pollCmd(slave, interface_rs485::SendRS485Msg::CMD_PS_V16_2);
        rate.sleep();
        pollCmd(slave, interface_rs485::SendRS485Msg::CMD_PS_V12);
        rate.sleep();
        pollCmd(slave, interface_rs485::SendRS485Msg::CMD_PS_C16_1);
        rate.sleep();
        pollCmd(slave, interface_rs485::SendRS485Msg::CMD_PS_C16_2);
        rate.sleep();
        pollCmd(slave, interface_rs485::SendRS485Msg::CMD_PS_C12);
        rate.sleep();
        pollCmd(slave, interface_rs485::SendRS485Msg::CMD_PS_temperature);
        rate.sleep();
        pollCmd(slave, interface_rs485::SendRS485Msg::CMD_PS_VBatt);
        rate.sleep();
        powerCheckActivation();

    }

    void ProviderPowerNode::pollCmd(uint8_t slave, uint8_t cmd) {

        interface_rs485::SendRS485Msg messageData;

        messageData.slave = slave;
        messageData.cmd = cmd;
        messageData.data.push_back(0x00);

        power_publisherRx_.publish(messageData);

    }

    void ProviderPowerNode::wattCalculation(const uint8_t slave, const uint8_t cmd) {

        provider_power::powerInfo msg;

        msg.slave = slave;
        msg.bus = cmd;

        float voltage = 0;
        float amperage = 0;

        voltage = powerInformation[slave][cmd];
        amperage = powerInformation[slave][cmd + next];

        if (voltage > 0 && amperage > 0){
            nbTime[slave]++;

            watt5min[slave][cmd] = voltage * amperage;
            msg.data5min = watt5min[slave][cmd];
            watttotal[slave][cmd] += watt5min[slave][cmd];

        }

        if (nbTime[slave] == 12){
            nbTime[slave] = 0;

            watt1h[slave][cmd] = watttotal[slave][cmd];
            msg.data1h = watt1h[slave][cmd];

        }

        power_publisherInfo_.publish(msg);


    }

    void ProviderPowerNode::initialize() {
        int i,j;

        for (i=0; i < 4; i++){

            nbTime[i] = 0;

            for(j=0; j < 3; j++){

                watt5min[i][j] = 0;
                watttotal[i][j] = 0;
                watt1h[i][j] = 0;
            }
        }

        for (i=0; i < 4; i++){

            for(j=0; j < 6; j++){

                powerInformation[i][j] = 0.0;
            }
        }

    }

}
