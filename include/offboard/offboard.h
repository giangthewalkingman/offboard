#ifndef OFFBOARD_H_
#define OFFBOARD_H_

#include<ros/ros.h>
#include<ros/time.h>
#include<iostream>
#include<cmath>
#include<cstdio>
#include<string>
#include<vector>
#include<unistd.h>
#include<ncurses.h>
#include<curses.h>
#include<wiringPi.h>
#include<wiringPiI2C.h>
#include<std_msgs/Bool.h>

#define DEVICE_ID 0x08
#define FORWARD 15
#define BACKWARD 0
#define TURNLEFT 10
#define TURNRIGHT 5
#define STOP 17

// direction rule: rf-lf-rb-lb, 1=forward, 0=backward
// 0000 = 0     -> backward
// 0001
// 0010
// 0100
// 1000
// 0011
// 0110
// 1100
// 1001
// 1010 = 10    -> turn left
// 0101 = 5     -> turn right
// 0111
// 1110
// 1101
// 1011
// 1111 = 15    -> forward

class OffboardControl
{
    public:
    OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool input_setpoint);
    ~OffboardControl();
    private:
    ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

    ros::Subscriber arm_mode_sub;

    std_msgs::Bool arm_mode_;

    int fd = 0;

    // right front - left front - right back - left back
    int16_t pwmValue[2];
    int16_t PWM_INCREMENT = 5;
    ros::Time operation_time_1, operation_time_2;

    void armModeCallback(const std_msgs::Bool::ConstPtr &msg);

    void offboard();
    void landing();
    void teleopControl();
    void i2cSetup();
    void waitForArming(double hz);
    void initNcurses();
    void cleanupNcurses();
    void printPWM(int16_t pwmValues[]);
    void sendI2CMsg(uint8_t throttle_pwm, uint8_t steering_pwm);
};

#endif