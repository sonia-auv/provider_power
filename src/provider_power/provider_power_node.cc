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
        writerVoltage = std::thread(std::bind(&ProviderPowerNode::writeVoltageData, this));
        writerCurrent = std::thread(std::bind(&ProviderPowerNode::writeCurrentData, this));
        writerMotor = std::thread(std::bind(&ProviderPowerNode::writeMotorData, this));

        // Check for env variable existence
        auv = std::getenv("AUV");
        if (auv == 0x0)
        {
            ROS_ERROR_STREAM("No environment variable found, using the default from AUV7");
            auv = "AUV7";
        }
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

            //std_msgs::Float64MultiArray msg;
            std::vector<double> msg;

            switch (receivedData->cmd)
            {
                case sonia_common::SendRS485Msg::CMD_VOLTAGE:
                    switch (receivedData->slave)
                    {
                        case sonia_common::SendRS485Msg::SLAVE_PSU0:
                            ROS_INFO("voltage slave 0");
                            if(INA22X_DataInterpretation(receivedData->data, msg, receivedData->data.size()) < 0) 
                            {
                                ROS_WARN_STREAM("ERROR in the message. Dropping VOLTAGE packet");
                                return;
                            }
                            else
                            {
                                parsedQueueVoltageSlave0.push_back(msg);
                            }
                            break;
                        case sonia_common::SendRS485Msg::SLAVE_PSU1:
                            ROS_INFO("voltage slave 1");
                            if(INA22X_DataInterpretation(receivedData->data, msg, receivedData->data.size()) < 0) 
                            {
                                ROS_WARN_STREAM("ERROR in the message. Dropping VOLTAGE packet");
                                return;
                            }
                            else
                            {
                                parsedQueueVoltageSlave1.push_back(msg);
                            }
                            break;
                        case sonia_common::SendRS485Msg::SLAVE_PSU2:
                            ROS_INFO("voltage slave 2");
                            if(INA22X_DataInterpretation(receivedData->data, msg, receivedData->data.size()) < 0) 
                            {
                                ROS_WARN_STREAM("ERROR in the message. Dropping VOLTAGE packet");
                                return;
                            }
                            else
                            {
                                parsedQueueVoltageSlave2.push_back(msg);
                            }
                            break;
                        case sonia_common::SendRS485Msg::SLAVE_PSU3:
                            ROS_INFO("voltage slave 3");
                            if(INA22X_DataInterpretation(receivedData->data, msg, receivedData->data.size()) < 0) 
                            {
                                ROS_WARN_STREAM("ERROR in the message. Dropping VOLTAGE packet");
                                return;
                            }
                            else
                            {
                                parsedQueueVoltageSlave3.push_back(msg);
                            }
                            break;
                        default:
                            ROS_WARN_STREAM("Unknown SLAVE to provider_power");
                            break;
                    }
                    break;
                case sonia_common::SendRS485Msg::CMD_CURRENT:
                    ROS_INFO("receive a rs485 data from SLAVE 1");
                    //readerQueueSlave1.push_back(receivedData);
                    break;
                case sonia_common::SendRS485Msg::CMD_READ_MOTOR:
                    ROS_INFO("receive a rs485 data from SLAVE 2");
                    //readerQueueSlave2.push_back(receivedData);
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
        int size;
        if (auv == "AUV8")
        { 
            size = nb_motor;
        } 
        else 
        {
            size = nb_motor/4;
        }
        std::vector<uint8_t> act(size);
        std::fill(act.begin(), act.end(), activation->data);
        MotorActivation(act);
    }

    void ProviderPowerNode::MotorActivationCallBack(const std_msgs::UInt8MultiArray::ConstPtr &activation)
    {
        if((activation->data.size() != nb_motor))
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
        if(size_req % 4 != 0) return -1;

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
                motor_request.slave = i; // Sorry
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
                // std::vector<uint8_t> act(size);

                // std::vector<double> 

                std_msgs::Float64MultiArray msg_16V;
                //std_msgs::Float64MultiArray msg_12V;


                std::vector<double> msg_slave0 = parsedQueueVoltageSlave0.get_n_pop_front();
                std::vector<double> msg_slave1 = parsedQueueVoltageSlave1.get_n_pop_front();
                std::vector<double> msg_slave2 = parsedQueueVoltageSlave2.get_n_pop_front();
                std::vector<double> msg_slave3 = parsedQueueVoltageSlave3.get_n_pop_front();

                //msg_16V.data = [msg_slave0(0),msg_slave1(0),msg_slave2(0),msg_slave3(0),msg_slave0(1),msg_slave1(1),msg_slave2(1),msg_slave3(1),msg_slave0(2),msg_slave1(2)];


                //voltage_publisher_.publish(msg_16V);

                //parsedData.msg.slave = msg_ptr->slave;
                //parsedData.msg.cmd = msg_ptr->cmd;

            }
        }


    }

    void ProviderPowerNode::writeCurrentData()
    {

    }

    void ProviderPowerNode::writeMotorData()
    {

    }

}
