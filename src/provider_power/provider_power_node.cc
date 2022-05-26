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

    void ProviderPowerNode::PowerDataCallBack(const sonia_common::SendRS485Msg::ConstPtr &receiveData) 
    {
        if (receiveData->slave >= sonia_common::SendRS485Msg::SLAVE_PSU0 && receiveData->slave <= sonia_common::SendRS485Msg::SLAVE_PSU3) {

            switch (receiveData->cmd) // Size is there to add support to AUV7 when the time will come
            {
            case sonia_common::SendRS485Msg::CMD_VOLTAGE:
                VoltageCMD(receiveData->data, nb_motor + nb_battery);
                break;
            case sonia_common::SendRS485Msg::CMD_CURRENT:
                CurrentCMD(receiveData->data, nb_motor + nb_battery);
                break;
            case sonia_common::SendRS485Msg::CMD_READ_MOTOR:
                ReadMotorCMD(receiveData->data, nb_motor);
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

        if(auv == "AUV8")
        {
            motor_request.slave = sonia_common::SendRS485Msg::SLAVE_PWR_MANAGEMENT;
            rs485_publisher_.publish(motor_request);
        } 
        else 
        {
            for (int i = 0; i <= 3; i++)
            {
                motor_request.slave = i;
                rs485_publisher_.publish(motor_request);
            }
        }
    }

    void ProviderPowerNode::writeVoltageData()
    {
        while(!ros::isShuttingDown())
        {
            ros::Duration(0.1).sleep();
            while(!parsedQueueVoltageSlave0.empty() && !parsedQueueVoltageSlave1.empty() && !parsedQueueVoltageSlave2.empty() && !parsedQueueVoltageSlave3.empty())
            {
                std_msgs::Float64MultiArray msg_16V;
                std_msgs::Float64MultiArray msg_12V;

                std::vector<double> msg_slave0 = parsedQueueVoltageSlave0.get_n_pop_front();
                std::vector<double> msg_slave1 = parsedQueueVoltageSlave1.get_n_pop_front();
                std::vector<double> msg_slave2 = parsedQueueVoltageSlave2.get_n_pop_front();
                std::vector<double> msg_slave3 = parsedQueueVoltageSlave3.get_n_pop_front();

                // Data from motors
                msg_16V.data.push_back(msg_slave0[0]);
                msg_16V.data.push_back(msg_slave1[0]);
                msg_16V.data.push_back(msg_slave2[0]);
                msg_16V.data.push_back(msg_slave3[0]);
                msg_16V.data.push_back(msg_slave0[1]);
                msg_16V.data.push_back(msg_slave1[1]);
                msg_16V.data.push_back(msg_slave2[1]);
                msg_16V.data.push_back(msg_slave3[1]);

                // Data from batteries
                msg_16V.data.push_back((msg_slave0[3]+msg_slave1[3])/2);
                msg_16V.data.push_back((msg_slave2[3]+msg_slave3[3])/2);

                // 12 V data
                msg_12V.data.push_back(msg_slave0[2]);
                msg_12V.data.push_back(msg_slave1[2]);
                msg_12V.data.push_back(msg_slave2[2]);
                msg_12V.data.push_back(msg_slave3[2]);

                voltage16V_publisher_.publish(msg_16V);
                voltage12V_publisher_.publish(msg_12V);

            }
        }
    }
}
