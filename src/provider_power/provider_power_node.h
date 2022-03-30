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
#include <sonia_common/SendRS485Msg.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>
#include <thread>
#include <sharedQueue.h>

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

        void PowerDataCallBack(const sonia_common::SendRS485Msg::ConstPtr &receivedData);

        void AllMotorActivationCallBack(const std_msgs::Bool::ConstPtr &activation);

        void MotorActivationCallBack(const std_msgs::UInt8MultiArray::ConstPtr &activation);

        void VoltageCMD(const std::vector<uint8_t> data, const uint8_t size);

        void CurrentCMD(const std::vector<uint8_t> data, const uint8_t size);

        void ReadMotorCMD(const std::vector<uint8_t> data, const uint8_t size);

        int INA22X_DataInterpretation(const std::vector<uint8_t> &req, std::vector<double> &res, uint8_t size_request);

        void MotorActivation(const std::vector<uint8_t> data);

        void writeData();

        ros::NodeHandlePtr nh_;
        ros::Publisher voltage_publisher_;
        ros::Publisher current_publisher_;
        ros::Publisher motor_publisher_;
        ros::Publisher rs485_publisher_;
        ros::Subscriber rs485_subscriber_;
        ros::Subscriber all_activation_subscriber_;
        ros::Subscriber activation_subscriber_;

        std::thread reader;
        std::thread writter;

        SharedQueue<sonia_common::SendRS485Msg::ConstPtr> writerQueue;

        union powerData {
            uint8_t Bytes[4];
            float_t info;
        };

        const uint8_t nb_motor = 8;
        const uint8_t nb_battery = 2;
    };
}  // namespace provider_power


#endif //PROVIDER_POWER_PROVIDER_POWER_NODE_H