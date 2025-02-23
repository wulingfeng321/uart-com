#include "../include/ros_32.h"
using namespace std;

//主函数
int main(int argc, char** argv){
    ROS_INFO("uart_node is running");
    //初始化节点
    ros::init(argc, argv, "uart_node");
    //创建节点句柄
    ros::NodeHandle uart;

//发送数据
    //初始化串口
    serialInit(); 
    //发送测试数据
    //sendTest();
    ros::Rate loop_rate(50); //指定循环频率60 
    ROS_INFO("uart_node is running now");
    while(ros::ok()){
        //ROS_INFO("uart_node is running");
        //订阅者创建
		//订阅place_info话题，包含机器人当前坐标，姿态信息
        //将ros消息转换为串口数据
        ros::Subscriber sub = uart.subscribe("/Odometry", 10, depart_place);
        ros::Subscriber sub2= uart.subscribe("/car_place",10, send8byte);
        //等待回调函数执行
		loop_rate.sleep();
        ros::spin();
		//等待一秒
		//ros::Duration(1.0).sleep();
	}
    return 0;
}