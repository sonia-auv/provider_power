//
// Created by sonia on 4/4/17.
//

#include "gtest/gtest.h"
#include <ros/ros.h>
#include "provider_power/provider_power_node.h"

char **argv_g;
int argc_g ;
union powerData {
    uint8_t Bytes[2];
    uint16_t data;
};

ros::NodeHandle nh;

class : public ::testing::Test{

private:


    const uint16_t Vtest1 = 0;      //Test minimal
    const uint16_t Vtest2 = 16000;  //Test nominal
    const uint16_t Vtest3 = 65535;  //Test maximal
    const uint16_t Ctest1 = 0;      //Test minimal
    const uint16_t Ctest2 = 15567;  //Test nominal
    const uint16_t Ctest3 = 65535;  //Test maximal





public:

    powerData dataVTest1, dataVTest2, dataVTest3, dataCTest1, dataCTest2, dataCTest3;

    ros::Publisher power_subscriberTx_ = nh.advertise<interface_rs485::SendRS485Msg>("/interface_rs485/dataRx", 10);

    interface_rs485::SendRS485Msg msg;

    struct dataTest{

        const uint8_t test1;
        const uint8_t test2;
        const uint8_t test3;

    };

    struct mockPS{

        dataTest

    };



};


// Declare a test
TEST(TestSuite, testCase1) {

    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    provider_power::ProviderPowerNode ppn(nh);
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<interface_rs485::SendRS485Msg>("/interface_rs485/dataTx", 10);

    const interface_rs485::SendRS485Msg::ConstPtr msg1;

    interface_rs485::SendRS485Msg msg;

    powerData powerIn;

    msg.slave = interface_rs485::SendRS485Msg::SLAVE_powersupply0;
    msg.cmd = interface_rs485::SendRS485Msg::CMD_PS_V16_1;

    powerIn.data = 16000;

    msg.data.push_back(powerIn.Bytes[1]);
    msg.data.push_back(powerIn.Bytes[0]);

    //pub.publish(msg);

    msg.cmd = interface_rs485::SendRS485Msg::CMD_PS_C16_1;


    //*msg1 = msg;
    pub.publish(msg);


    //sleep(2);

   // ppn.PublishPowerMsg(msg1);

    uint16_t e = ppn.powerInformation[0][0];

    uint16_t expected = ppn.watt5min[0][0];

    ASSERT_EQ(0,expected);


}

// Declare another test
TEST(TestSuite, testCase2)
{

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);
    testing::InitGoogleMock(&argc, argv);
    argc_g = argc;
    argv_g = argv;
    ros::init(argc_g, argv_g, "provider_power");

    return RUN_ALL_TESTS();
}