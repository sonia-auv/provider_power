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

}

//------------------------------------------------------------------------------
//
ProviderPowerNode::~ProviderPowerNode() {

}

//==============================================================================
// M E T H O D   S E C T I O N

void ProviderPowerNode::PublishPowerMsg(){

    provider_power::powerMsg msg;

    msg.cur16Pin1card1 = 10.1;

    power_publisher_.publish(msg);


}


void ProviderPowerNode::PublishPowerData(){

    interface_rs485::SendRS485Msg messageData;

    messageData.slave = messageData.SLAVE_powersupply0;
    messageData.cmd =  messageData.CMD_PS_V16_1;
    messageData.data.push_back(0x00);

    power_publisherRx_.publish(messageData);


}


}
