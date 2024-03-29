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
#include <mutex>

namespace provider_power {

    class ProviderPowerNode {
    public:
        const double RATE_HZ = 5.0;
        const double RATE_HZ_MESSAGE = 2.0;

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

        void TemperatureCMD(const std::vector<uint8_t> data, const uint8_t size);

        int INA22X_DataInterpretation(const std::vector<uint8_t> &req, std::vector<double> &res, uint8_t size_request);

        void MotorActivation(const std::vector<uint8_t> data);

        void writeVoltageData();
        void writeCurrentData();
        void writeMotorData();

        ros::NodeHandlePtr nh_;
        ros::Publisher voltage16V_publisher_;
        ros::Publisher voltage12V_publisher_;
        ros::Publisher current_publisher_;
        ros::Publisher temperature_publisher_;
        ros::Publisher motor_publisher_;
        ros::Publisher rs485_publisher_;
        ros::Subscriber rs485_subscriber_;
        ros::Subscriber all_activation_subscriber_;
        ros::Subscriber activation_subscriber_;

        std::thread writerVoltage;
        std::thread writerCurrent;
        std::thread writerMotor;

        std::mutex voltageMutex;
        std::mutex currentMutex;
        std::mutex motorMutex;

        std::vector<double> parsedQueueVoltageSlave0;
        std::vector<double> parsedQueueVoltageSlave1;
        std::vector<double> parsedQueueVoltageSlave2;
        std::vector<double> parsedQueueVoltageSlave3;
        std::vector<double> parsedQueueCurrentSlave0;
        std::vector<double> parsedQueueCurrentSlave1;
        std::vector<double> parsedQueueCurrentSlave2;
        std::vector<double> parsedQueueCurrentSlave3;
        std::vector<uint8_t> readQueueMotorSlave0;
        std::vector<uint8_t> readQueueMotorSlave1;
        std::vector<uint8_t> readQueueMotorSlave2;
        std::vector<uint8_t> readQueueMotorSlave3;

        union powerData {
            uint8_t Bytes[4];
            float_t info;
        };

        const uint8_t nb_motor = 8;
        const uint8_t nb_battery = 2;
        const char* auv;
    };
}  // namespace provider_power


#endif //PROVIDER_POWER_PROVIDER_POWER_NODE_H