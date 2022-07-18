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

#define TIME_BETWEEN_FLUSH 30 // seconds

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
        voltage16V_publisher_ = nh_->advertise<std_msgs::Float64MultiArray>("/provider_power/voltage", 100);
        voltage12V_publisher_ = nh_->advertise<std_msgs::Float64MultiArray>("/provider_power/voltage12V", 100);
        current_publisher_ = nh_->advertise<std_msgs::Float64MultiArray>("/provider_power/current", 100);
        temperature_publisher_ = nh_->advertise<std_msgs::Float64MultiArray>("/provider_power/temperature", 100);
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
        if (strcmp(std::getenv("AUV"), "AUV8") == 0)
        {
            auv = "AUV8";
        }
        else if (strcmp(std::getenv("AUV"), "AUV7") == 0)
        {
            auv = "AUV7";
        }
        else 
        {
            ROS_ERROR_STREAM("No environnement variable found, using the default from AUV8");
            auv = "AUV8";
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

            std::vector<double> msg;

            switch (receivedData->cmd)
            {
                case sonia_common::SendRS485Msg::CMD_VOLTAGE:
                    switch (receivedData->slave)
                    {
                        case sonia_common::SendRS485Msg::SLAVE_PSU0:
                            if(INA22X_DataInterpretation(receivedData->data, msg, receivedData->data.size()/4) < 0) 
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
                            if(INA22X_DataInterpretation(receivedData->data, msg, receivedData->data.size()/4) < 0) 
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
                            if(INA22X_DataInterpretation(receivedData->data, msg, receivedData->data.size()/4) < 0) 
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
                            if(INA22X_DataInterpretation(receivedData->data, msg, receivedData->data.size()/4) < 0) 
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
                    switch (receivedData->slave)
                    {
                        case sonia_common::SendRS485Msg::SLAVE_PSU0:
                            if(INA22X_DataInterpretation(receivedData->data, msg, receivedData->data.size()/4) < 0) 
                            {
                                ROS_WARN_STREAM("ERROR in the message. Dropping CURRENT packet");
                                return;
                            }
                            else
                            {
                                parsedQueueCurrentSlave0.push_back(msg);
                            }
                            break;
                        case sonia_common::SendRS485Msg::SLAVE_PSU1:
                            if(INA22X_DataInterpretation(receivedData->data, msg, receivedData->data.size()/4) < 0) 
                            {
                                ROS_WARN_STREAM("ERROR in the message. Dropping CURRENT packet");
                                return;
                            }
                            else
                            {
                                parsedQueueCurrentSlave1.push_back(msg);
                            }
                            break;
                        case sonia_common::SendRS485Msg::SLAVE_PSU2:
                            if(INA22X_DataInterpretation(receivedData->data, msg, receivedData->data.size()/4) < 0) 
                            {
                                ROS_WARN_STREAM("ERROR in the message. Dropping CURRENT packet");
                                return;
                            }
                            else
                            {
                                parsedQueueCurrentSlave2.push_back(msg);
                            }
                            break;
                        case sonia_common::SendRS485Msg::SLAVE_PSU3:
                            if(INA22X_DataInterpretation(receivedData->data, msg, receivedData->data.size()/4) < 0) 
                            {
                                ROS_WARN_STREAM("ERROR in the message. Dropping CURRENT packet");
                                return;
                            }
                            else
                            {
                                parsedQueueCurrentSlave3.push_back(msg);
                            }
                            break;
                        default:
                            ROS_WARN_STREAM("Unknown SLAVE to provider_power");
                            break;
                    }
                    break;
                case sonia_common::SendRS485Msg::CMD_READ_MOTOR:
                    switch (receivedData->slave)
                    {
                        case sonia_common::SendRS485Msg::SLAVE_PSU0:
                            readQueueMotorSlave0.push_back(receivedData->data);
                            break;
                        case sonia_common::SendRS485Msg::SLAVE_PSU1:
                            readQueueMotorSlave1.push_back(receivedData->data);
                            break;
                        case sonia_common::SendRS485Msg::SLAVE_PSU2:
                            readQueueMotorSlave2.push_back(receivedData->data);
                            break;
                        case sonia_common::SendRS485Msg::SLAVE_PSU3:
                            readQueueMotorSlave3.push_back(receivedData->data);
                            break;
                        default:
                            ROS_WARN_STREAM("Unknown SLAVE to provider_power");
                            break;
                    }
                    break;
                case sonia_common::SendRS485Msg::CMD_KEEP_ALIVE:
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
            case sonia_common::SendRS485Msg::CMD_TEMPERATURE:
                TemperatureCMD(receivedData->data, nb_motor + nb_battery); 
                break;
            case sonia_common::SendRS485Msg::CMD_READ_MOTOR:
                ReadMotorCMD(receivedData->data, nb_motor);
                break;
            case sonia_common::SendRS485Msg::CMD_KEEP_ALIVE:
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
        voltage16V_publisher_.publish(msg);
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

    void ProviderPowerNode::TemperatureCMD(const std::vector<uint8_t> data, const uint8_t size)
    {
        std_msgs::Float64MultiArray msg;

        if(INA22X_DataInterpretation(data, msg.data, size) < 0) 
        {
            ROS_WARN_STREAM("ERROR in the message. Dropping TEMPERATURE packet");
            return;
        }
        temperature_publisher_.publish(msg);
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
                motor_request.slave = i;
                rs485_publisher_.publish(motor_request);
            }
        }
    }
    

    void ProviderPowerNode::writeVoltageData()
    {
        double time_start = ros::Time::now().toSec();

        while(!ros::isShuttingDown())
        {
            if(!parsedQueueVoltageSlave0.empty() && !parsedQueueVoltageSlave1.empty() && !parsedQueueVoltageSlave2.empty() && !parsedQueueVoltageSlave3.empty())
            {
                std_msgs::Float64MultiArray msg_16V;
                std_msgs::Float64MultiArray msg_12V;

                std::vector<double> msg_slave0 = parsedQueueVoltageSlave0.get_n_pop_front();
                std::vector<double> msg_slave1 = parsedQueueVoltageSlave1.get_n_pop_front();
                std::vector<double> msg_slave2 = parsedQueueVoltageSlave2.get_n_pop_front();
                std::vector<double> msg_slave3 = parsedQueueVoltageSlave3.get_n_pop_front();

                // Motors Voltage
                msg_16V.data.push_back(msg_slave0[0]);
                msg_16V.data.push_back(msg_slave1[0]);
                msg_16V.data.push_back(msg_slave2[0]);
                msg_16V.data.push_back(msg_slave3[0]);
                msg_16V.data.push_back(msg_slave0[1]);
                msg_16V.data.push_back(msg_slave1[1]);
                msg_16V.data.push_back(msg_slave2[1]);
                msg_16V.data.push_back(msg_slave3[1]);

                // Batteries Voltage
                msg_16V.data.push_back((msg_slave0[3]+msg_slave1[3])/2);
                msg_16V.data.push_back((msg_slave2[3]+msg_slave3[3])/2);

                // 12V Voltage
                msg_12V.data.push_back(msg_slave0[2]);
                msg_12V.data.push_back(msg_slave1[2]);
                msg_12V.data.push_back(msg_slave2[2]);
                msg_12V.data.push_back(msg_slave3[2]);

                voltage16V_publisher_.publish(msg_16V);
                voltage12V_publisher_.publish(msg_12V);
            }
            else
            {
                ros::Duration(0.1).sleep();
            }

            if(time_start - ros::Time::now().toSec() >= TIME_BETWEEN_FLUSH)
            {
                parsedQueueVoltageSlave0.clear();
                parsedQueueVoltageSlave1.clear();
                parsedQueueVoltageSlave2.clear();
                parsedQueueVoltageSlave3.clear();
                time_start = ros::Time::now().toSec();
            }
        }
    }

    void ProviderPowerNode::writeCurrentData()
    {
        double time_start = ros::Time::now().toSec();

        while(!ros::isShuttingDown())
        {   
            if(!parsedQueueCurrentSlave0.empty() && !parsedQueueCurrentSlave1.empty() && !parsedQueueCurrentSlave2.empty() && !parsedQueueCurrentSlave3.empty())
            {
                std_msgs::Float64MultiArray msg;

                std::vector<double> msg_slave0 = parsedQueueCurrentSlave0.get_n_pop_front();
                std::vector<double> msg_slave1 = parsedQueueCurrentSlave1.get_n_pop_front();
                std::vector<double> msg_slave2 = parsedQueueCurrentSlave2.get_n_pop_front();
                std::vector<double> msg_slave3 = parsedQueueCurrentSlave3.get_n_pop_front();

                for (int i = 0; i <= 2; i++)
                {
                    msg.data.push_back(msg_slave0[i]);
                    msg.data.push_back(msg_slave1[i]);
                    msg.data.push_back(msg_slave2[i]);
                    msg.data.push_back(msg_slave3[i]);
                }

                current_publisher_.publish(msg);
            }
            else
            {
                ros::Duration(0.1).sleep();
            }

            if(time_start - ros::Time::now().toSec() >= TIME_BETWEEN_FLUSH)
            {
                parsedQueueCurrentSlave0.clear();
                parsedQueueCurrentSlave1.clear();
                parsedQueueCurrentSlave2.clear();
                parsedQueueCurrentSlave3.clear();
                time_start = ros::Time::now().toSec();
            }
        }
    }

    void ProviderPowerNode::writeMotorData()
    {
        double time_start = ros::Time::now().toSec();

        while(!ros::isShuttingDown())
        {
            if(!readQueueMotorSlave0.empty() && !readQueueMotorSlave1.empty() && !readQueueMotorSlave2.empty() && !readQueueMotorSlave3.empty())
            {
                std_msgs::UInt8MultiArray msg;

                std::vector<uint8_t> msg_slave0 = readQueueMotorSlave0.get_n_pop_front();
                std::vector<uint8_t> msg_slave1 = readQueueMotorSlave1.get_n_pop_front();
                std::vector<uint8_t> msg_slave2 = readQueueMotorSlave2.get_n_pop_front();
                std::vector<uint8_t> msg_slave3 = readQueueMotorSlave3.get_n_pop_front();

                for (int i = 0; i < 2; i++)
                {
                    msg.data.push_back(msg_slave0[i]);
                    msg.data.push_back(msg_slave1[i]);
                    msg.data.push_back(msg_slave2[i]);
                    msg.data.push_back(msg_slave3[i]);
                }

                motor_publisher_.publish(msg);
            }
            else
            {
                ros::Duration(0.1).sleep();
            }

            if(time_start - ros::Time::now().toSec() >= TIME_BETWEEN_FLUSH)
            {
                readQueueMotorSlave0.clear();
                readQueueMotorSlave1.clear();
                readQueueMotorSlave2.clear();
                readQueueMotorSlave3.clear();
                time_start = ros::Time::now().toSec();
            }
        }
    }
}
