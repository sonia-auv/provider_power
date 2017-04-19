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

uint8_t done;

class mockingPS{
public:
    //============================================================================
    // P U B L I C   C / D T O R S

    mockingPS(ros::NodeHandlePtr &nh);

    ~mockingPS();


    powerData dataVTest1, dataVTest2, dataVTest3, dataCTest1, dataCTest2, dataCTest3, dataVTest, dataCTest;

    //ros::Publisher power_publisherTx_ = nh.advertise<interface_rs485::SendRS485Msg>("/interface_rs485/dataTx", 10);

    interface_rs485::SendRS485Msg msgTestV;
    interface_rs485::SendRS485Msg msgTestC;


    const float Vtest1 = 0;      //Test minimal
    const float Vtest2 = 12234;  //Test nominal
    const float Vtest3 = 65535;  //Test maximal
    const float Ctest1 = 0;      //Test minimal
    const float Ctest2 = 1550;   //Test nominal
    const float Ctest3 = 65535;  //Test maximal

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

    dataVTest.data = Vtest1;
    dataCTest.data = Ctest1;


    msgTestV.data.push_back(dataVTest.Bytes[1]);
    msgTestV.data.push_back(dataVTest.Bytes[0]);

    msgTestC.data.push_back(dataCTest.Bytes[1]);
    msgTestC.data.push_back(dataCTest.Bytes[0]);

}

void mockingPS::initializeMockTest2(){

    msgTestV.slave = interface_rs485::SendRS485Msg::SLAVE_powersupply1;
    msgTestC.slave = interface_rs485::SendRS485Msg::SLAVE_powersupply1;

    msgTestV.cmd = interface_rs485::SendRS485Msg::CMD_PS_V12;
    msgTestC.cmd = interface_rs485::SendRS485Msg::CMD_PS_C12;

    dataVTest.data = Vtest2;
    dataCTest.data = Ctest2;

    msgTestV.data.push_back(dataVTest.Bytes[1]);
    msgTestV.data.push_back(dataVTest.Bytes[0]);

    msgTestC.data.push_back(dataCTest.Bytes[1]);
    msgTestC.data.push_back(dataCTest.Bytes[0]);

}

void mockingPS::initializeMockTest3(){

    msgTestV.slave = interface_rs485::SendRS485Msg::SLAVE_powersupply3;
    msgTestC.slave = interface_rs485::SendRS485Msg::SLAVE_powersupply3;

    msgTestV.cmd = interface_rs485::SendRS485Msg::CMD_PS_V16_2;
    msgTestC.cmd = interface_rs485::SendRS485Msg::CMD_PS_C16_2;

    dataVTest.data = Vtest3;
    dataCTest.data = Ctest3;

    msgTestV.data.push_back(dataVTest.Bytes[1]);
    msgTestV.data.push_back(dataVTest.Bytes[0]);

    msgTestC.data.push_back(dataCTest.Bytes[1]);
    msgTestC.data.push_back(dataCTest.Bytes[0]);

}


// Declare a test
TEST(TestSuite, testCase1) {

    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    mockingPS mps(nh);
    mps.initializeMockTest1();

    provider_power::ProviderPowerNode ppn(nh);


    done=0;
    do{

        mps.power_publisherTx_.publish(mps.msgTestV);
        sleep(1);
        mps.power_publisherTx_.publish(mps.msgTestC);
        sleep(1);

        done++;

    }while(done <= 4);


    ppn.wattCalculation(0,0);

    uint16_t data = ppn.watt5min[0][0];

    ASSERT_EQ(data,0);

}

// Declare another test
TEST(TestSuite, testCase2){

    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    mockingPS mps(nh);
    mps.initializeMockTest2();

    provider_power::ProviderPowerNode ppn(nh);

    done=0;
    do{

        mps.power_publisherTx_.publish(mps.msgTestV);
        sleep(1);
        mps.power_publisherTx_.publish(mps.msgTestC);
        sleep(1);

        done++;

    }while(done <= 2);

    ppn.wattCalculation(0,0);

    uint16_t data = ppn.watt5min[1][2];

    ASSERT_EQ(data,0);

}

TEST(TestSuite, testCase3){

    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    mockingPS mps(nh);
    mps.initializeMockTest3();

    provider_power::ProviderPowerNode ppn(nh);

    done=0;
    do{

        mps.power_publisherTx_.publish(mps.msgTestV);
        sleep(1);
        mps.power_publisherTx_.publish(mps.msgTestC);
        sleep(1);

        done++;

    }while(done <= 4);

    ppn.wattCalculation(0,0);

    uint16_t data = ppn.watt5min[0][0];

    ASSERT_EQ(data,0);

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {

    ros::init(argc_g, argv_g, "provider_power");
    testing::InitGoogleTest(&argc, argv);


    return RUN_ALL_TESTS();
}