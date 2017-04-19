//
// Created by sonia on 4/4/17.
//

#include "gtest/gtest.h"
#include <ros/ros.h>
#include "provider_power/provider_power_node.h"

char **argv_g;
int argc_g;

union powerData {
    uint8_t Bytes[2];
    uint16_t data;
};


class mockingPS{
public:
    //============================================================================
    // P U B L I C   C / D T O R S

    mockingPS(ros::NodeHandlePtr &nh);

    ~mockingPS();


    powerData dataVTest1, dataVTest2, dataVTest3, dataCTest1, dataCTest2, dataCTest3;

    //ros::Publisher power_publisherTx_ = nh.advertise<interface_rs485::SendRS485Msg>("/interface_rs485/dataTx", 10);

    interface_rs485::SendRS485Msg msgTestV;
    interface_rs485::SendRS485Msg msgTestC;


    const uint16_t Vtest1 = 0;      //Test minimal
    const uint16_t Vtest2 = 12234;  //Test nominal
    const uint16_t Vtest3 = 65535;  //Test maximal
    const uint16_t Ctest1 = 0;      //Test minimal
    const uint16_t Ctest2 = 155;   //Test nominal
    const uint16_t Ctest3 = 65535;  //Test maximal

    void initializeMockTest1();
    void initializeMockTest2();
    void initializeMockTest3();

    ros::Publisher power_publisherTx_;


private:

    ros::NodeHandlePtr nh_;

};


mockingPS::mockingPS(ros::NodeHandlePtr &nh)
        : nh_(nh) {

    power_publisherTx_ = nh_->advertise<interface_rs485::SendRS485Msg>("/interface_rs485/dataTx", 10);


}

mockingPS::~mockingPS(){


}

void mockingPS::initializeMockTest1(){

    msgTestV.slave = interface_rs485::SendRS485Msg::SLAVE_powersupply0;
    msgTestC.slave = interface_rs485::SendRS485Msg::SLAVE_powersupply0;

    msgTestV.cmd = interface_rs485::SendRS485Msg::CMD_PS_V16_1;
    msgTestC.cmd = interface_rs485::SendRS485Msg::CMD_PS_C16_1;

    dataVTest1.data = Vtest1;
    dataCTest1.data = Ctest1;


    msgTestV.data.push_back(dataVTest1.Bytes[1]);
    msgTestV.data.push_back(dataVTest1.Bytes[0]);

    msgTestC.data.push_back(dataCTest1.Bytes[1]);
    msgTestC.data.push_back(dataCTest1.Bytes[0]);





}

void mockingPS::initializeMockTest2(){

    msgTestV.slave = interface_rs485::SendRS485Msg::SLAVE_powersupply1;
    msgTestC.slave = interface_rs485::SendRS485Msg::SLAVE_powersupply1;

    msgTestV.cmd = interface_rs485::SendRS485Msg::CMD_PS_V12;
    msgTestC.cmd = interface_rs485::SendRS485Msg::CMD_PS_C12;

    dataVTest2.data = Vtest2;
    dataCTest2.data = Ctest2;

    msgTestV.data.push_back(dataVTest2.Bytes[1]);
    msgTestV.data.push_back(dataVTest2.Bytes[0]);

    msgTestC.data.push_back(dataCTest2.Bytes[1]);
    msgTestC.data.push_back(dataCTest2.Bytes[0]);

}

void mockingPS::initializeMockTest3(){

    msgTestV.slave = interface_rs485::SendRS485Msg::SLAVE_powersupply3;
    msgTestC.slave = interface_rs485::SendRS485Msg::SLAVE_powersupply3;

    msgTestV.cmd = interface_rs485::SendRS485Msg::CMD_PS_V16_2;
    msgTestC.cmd = interface_rs485::SendRS485Msg::CMD_PS_C16_2;

    dataVTest3.data = Vtest3;
    dataCTest3.data = Ctest3;

    msgTestV.data.push_back(dataVTest3.Bytes[1]);
    msgTestV.data.push_back(dataVTest3.Bytes[0]);

    msgTestC.data.push_back(dataCTest3.Bytes[1]);
    msgTestC.data.push_back(dataCTest3.Bytes[0]);

}


// Declare a test
TEST(TestSuite, testCase1) {

    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    mockingPS mps(nh);
    mps.initializeMockTest1();

    mps.power_publisherTx_.publish(mps.msgTestV);
    sleep(2);
    mps.power_publisherTx_.publish(mps.msgTestC);

}

// Declare another test
TEST(TestSuite, testCase2){

    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    mockingPS mps(nh);
    mps.initializeMockTest2();

    mps.power_publisherTx_.publish(mps.msgTestV);
    sleep(2);
    mps.power_publisherTx_.publish(mps.msgTestC);

}

TEST(TestSuite, testCase3){

    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    mockingPS mps(nh);
    mps.initializeMockTest3();

    mps.power_publisherTx_.publish(mps.msgTestV);
    sleep(2);
    mps.power_publisherTx_.publish(mps.msgTestC);

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {

    ros::init(argc_g, argv_g, "provider_power");
    testing::InitGoogleTest(&argc, argv);


    return RUN_ALL_TESTS();
}