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
        // Publishers
        voltage_publisher_ = nh_->advertise<std_msgs::Float64MultiArray>("/provider_power/voltage", 100);
        current_publisher_ = nh_->advertise<std_msgs::Float64MultiArray>("/provider_power/current", 100);
        motor_publisher_ = nh_->advertise<std_msgs::UInt8MultiArray>("/provider_power/motor_feedback", 100);
        rs485_publisher_ = nh_->advertise<sonia_common::SendRS485Msg>("/interface_rs485/dataRx", 100);

        // Subscribers
        rs485_subscriber_ = nh_->subscribe("/interface_rs485/dataTx", 100, &ProviderPowerNode::PowerDataCallBack, this);
        all_activation_subscriber_ = nh_->subscribe("/provider_power/activate_all_motor", 100, &ProviderPowerNode::AllMotorActivationCallBack, this);
        activation_subscriber_ = nh_->subscribe("/provider_power/activate_motor", 100, &ProviderPowerNode::MotorActivationCallBack, this);

        // Threads
        reader = std::thread(std::bind(&ProviderPowerNode::readData, this));
        writer = std::thread(std::bind(&ProviderPowerNode::writeData, this));
    }

//------------------------------------------------------------------------------
//
    ProviderPowerNode::~ProviderPowerNode() {}

//==============================================================================
// M E T H O D   S E C T I O N

    void ProviderPowerNode::Spin(){
        ros::Rate r(5); // 5 hz

        while(ros::ok())
        {
            ros::spinOnce();
            r.sleep();
        }
    }

    void ProviderPowerNode::PowerDataCallBack(const sonia_common::SendRS485Msg::ConstPtr &receivedData) 
    {
        if (receivedData->slave >= sonia_common::SendRS485Msg::SLAVE_PSU0 && receivedData->slave <= sonia_common::SendRS485Msg::SLAVE_PSU3) {

            ROS_DEBUG("receive a rs485 data")
            writerQueue.push_back(receivedData)


            switch (receivedData->cmd)
            {
            case sonia_common::SendRS485Msg::CMD_VOLTAGE:
                VoltageCMD(receivedData->data, nb_motor/4 + nb_battery);
                break;
            case sonia_common::SendRS485Msg::CMD_CURRENT:
                CurrentCMD(receivedData->data, nb_motor/4 + nb_battery/2);
                break;
            case sonia_common::SendRS485Msg::CMD_READ_MOTOR:
                ReadMotorCMD(receivedData->data, nb_motor/4);
                break;
            default:
                ROS_WARN_STREAM("Unknow CMD to provider_power");
                break;
            }
        }
        
        if (receivedData->slave == sonia_common::SendRS485Msg::SLAVE_PWR_MANAGEMENT) {
            switch (receivedData->cmd)
            {
            case sonia_common::SendRS485Msg::CMD_VOLTAGE:
                VoltageCMD(receivedData->data, nb_motor + nb_battery);
                break;
            case sonia_common::SendRS485Msg::CMD_CURRENT:
                CurrentCMD(receivedData->data, nb_motor + nb_battery);
                break;
            case sonia_common::SendRS485Msg::CMD_READ_MOTOR:
                ReadMotorCMD(receivedData->data, nb_motor);
                break;
            default:
                ROS_WARN_STREAM("Unknow CMD to provider_power");
                break;
            }
        }
    }
    
    void ProviderPowerNode::AllMotorActivationCallBack(const std_msgs::Bool::ConstPtr &activation)
    {
        std::vector<uint8_t> act(nb_motor);
        std::fill(act.begin(), act.end(), activation->data);
        MotorActivation(act);
    }

    void ProviderPowerNode::MotorActivationCallBack(const std_msgs::UInt8MultiArray::ConstPtr &activation)
    {
        if(activation->data.size() != nb_motor)
        {
            ROS_WARN_STREAM("Invalid size requested. Dropping request for MOTOR ACTIVATION");
            return;
        }
        MotorActivation(activation->data);
    }

    void ProviderPowerNode::VoltageCMD(const std::vector<uint8_t> data, const uint8_t size)
    {
        std_msgs::Float64MultiArray msg;

        if(INA22X_DataInterpretation(data, msg.data, size) < 0) 
        {
            ROS_WARN_STREAM("ERROR in the message. Dropping VOLTAGE packet");
            return;
        }
        voltage_publisher_.publish(msg);
    }

    void ProviderPowerNode::CurrentCMD(const std::vector<uint8_t> data, const uint8_t size)
    {
        std_msgs::Float64MultiArray msg;

        if(INA22X_DataInterpretation(data, msg.data, size) < 0) 
        {
            ROS_WARN_STREAM("ERROR in the message. Dropping CURRENT packet");
            return;
        }
        current_publisher_.publish(msg);
    }

    void ProviderPowerNode::ReadMotorCMD(const std::vector<uint8_t> data, const uint8_t size)
    {
        std_msgs::UInt8MultiArray msg;

        if(data.size() != size)
        {
            ROS_WARN_STREAM("Error in the message. Dropping READ MOTOR packet");
            return;
        }
        msg.data = data;
        motor_publisher_.publish(msg);
    }

    int ProviderPowerNode::INA22X_DataInterpretation(const std::vector<uint8_t> &req, std::vector<double> &res, uint8_t size_request)
    {
        uint8_t size_req = req.size();
        if(size_req / 4 != size_request) return -1;

        powerData value;

        for(uint8_t i = 0; i < size_request; ++i) // shifting of 4 for each data
        {
            value.Bytes[0] = req[4*i];
            value.Bytes[1] = req[4*i+1];
            value.Bytes[2] = req[4*i+2];
            value.Bytes[3] = req[4*i+3];
            res.push_back(value.info);
        }
        return 0;
    }

    void ProviderPowerNode::MotorActivation(const std::vector<uint8_t> data)
    {
        sonia_common::SendRS485Msg motor_request;

        motor_request.slave = sonia_common::SendRS485Msg::SLAVE_PSU0;
        motor_request.cmd = sonia_common::SendRS485Msg::CMD_ACT_MOTOR;
        motor_request.data = data;
        rs485_publisher_.publish(motor_request);
    }
}
