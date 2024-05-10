#ifndef OFFBOARD_H_
#define OFFBOARD_H_

#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include <unistd.h>
#include <ncurses.h>
#include <curses.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense> 
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#define DEVICE_ID 0x08

class OffboardControl
{
    public:
    OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool input_setpoint);
    ~OffboardControl();
    private:
    ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

    ros::Subscriber arm_mode_sub;
    ros::Subscriber odom_sub; // odometry subscriber

    ros::Publisher odom_error_pub; //publish odom error before arm

    std_msgs::Bool arm_mode_;
    nav_msgs::Odometry current_odom_;
    ros::Time operation_time_1, operation_time_2;
    Eigen::Vector3d robot_pos_;

    int fd = 0;

    // right front - left front - right back - left back
    int16_t pwmValue[2];
    int16_t PWM_INCREMENT = 5;
    bool odom_received_ = false; // check received odometry or not
    bool i2c_flag_ = false; 
    bool rc_flag_ = false;
    // uint8_t flags = 0;
    double yaw_ = 0;
    const double length = 20.0;
    double steering_noise = 0.0;
    double distance_noise = 0.0;
    double steering_drift = 0.0;

    void armModeCallback(const std_msgs::Bool::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry &odomMsg);

    void offboard();
    void landing();
    void teleopControl();
    void i2cSetup();
    void waitForArming(double hz);
    void initNcurses();
    void cleanupNcurses();
    void printPWM(int16_t pwmValues[]);
    // rc_flag_ << 1; i2c_flag_ << 0;  
    void sendI2CMsg(uint8_t throttle_pwm, uint8_t steering_pwm, uint8_t flags);
    void PidTest();
};

inline Eigen::Vector3d toEigen(const geometry_msgs::Point &p) {
  Eigen::Vector3d ev3(p.x, p.y, p.z);
  return ev3;
}

inline Eigen::Vector3d toEigen(const geometry_msgs::PoseStamped &p) {
  return toEigen(p.pose.position);
}

inline Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion &p) {
  Eigen::Quaterniond q4(p.w ,p.x, p.y, p.z);
    return q4;
}

inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3) {
  Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
  return ev3;
}

#endif