#ifndef ROS_32_H
#define ROS_32_H

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <std_msgs/String.h>
#include <boost/asio/serial_port.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>
#include <sstream>
#include <functional>
#include <custom_msgs/carplace.h>
#include <custom_msgs/xyz.h>


using namespace std;
extern float msg_place[];
extern void serialInit();//初始化串口
extern void send12byte(float* msg);//发送12byte数据
extern int receive12byte(unsigned char* msg);//接收12byte数据
extern unsigned char getCrc(const unsigned char *data, short length, unsigned char polynomial, unsigned char initial_value); //计算CRC8校验码
extern unsigned char getParity(const unsigned char *data, short length); //计算奇偶校验码
extern void depart_place(const nav_msgs::Odometry::ConstPtr &msg);//拆分ros消息
extern void trans2robot(float* msg);//slam坐标转换机器人坐标
extern void send8byte(const custom_msgs::carplace &msg);//发送8byte数据
extern void send24byte();//发送24byte数据
extern void basket_place(const custom_msgs::xyz &msg);//拆分ros消息
extern void pushData(float* msg);//压入data_buf前三位
extern void pushData2(float* msg);//压入data_buf后三位

#endif // ROS_32_H