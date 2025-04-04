#include "../include/ros_32.h"

using namespace std;
float msg_place[3];

//串口发送接收相关常量、变量、共用体对象
const unsigned char ender[2] = {0x0d, 0x0a};//结束符 0x0d:回车 0x0a:换行
const unsigned char header[2] = {0x55, 0xaa};//帧头 0x55:帧头 0xaa:帧头
std::string serial_name="/dev/ttyTHS0"; //串口名称
//创建串口对象
boost::asio::io_context iosev;//创建io_service对象
//创建串口对象
boost::asio::serial_port sp(iosev, serial_name);//创建串口对象   

//crc校验常量
const unsigned char CRC8_POLYNOMIAL = 0x07; //CRC8多项式
const unsigned char CRC8_INITIAL_VALUE = 0x00; //CRC8初始值
    
// 串口初始化
extern void serialInit(){
    //设置串口参数
    sp.set_option(boost::asio::serial_port_base::baud_rate(115200));
    sp.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    sp.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    sp.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    sp.set_option(boost::asio::serial_port_base::character_size(8));  
}

//发送函数

//发送12byte数据信息，可容纳6个float数据,精度为precision1位小数和precision2位小数
//数据格式为：帧头  数据长度  数据  校验和  结束符
void send12byte(float* msg){

    //创建共用体，将float数据转换为字节数组
    union{
        float f[3];
        unsigned char c[12];
    }data;
    for(int i = 0; i < 3; i++){
        data.f[i] = msg[i];
    }

    unsigned char buf[18]= {0};                                
    int i = 0;
    short len = 0;//数据长度，范围0-255
    //消息头
    buf[i++] = header[0];
    buf[i++] = header[1];
    //数据长度
    len = 12;
    buf[i++] = 0x0c;//长度为12
    //数据赋值
    for(int j = 0; j < 12; j++){
        buf[i++] = data.c[j];
    }
    //校验和
    unsigned char crc = getCrc(data.c, len, CRC8_POLYNOMIAL, CRC8_INITIAL_VALUE);
    buf[i++] = crc;
    //消息尾
    buf[i++] = ender[0];
    buf[i++] = ender[1];
    //发送数据
    boost::asio::write(sp, boost::asio::buffer(buf));
    ROS_INFO("send data %.2f %.2f %.2f complete:",data.f[0],data.f[1],data.f[2]);
}


//发送8byte数据信息，msg为接收到的自定消息类型
void send8byte(const custom_msgs::carplace &msg){
    //创建共用体，将float数据转换为字节数组
    union{
        float f[2];
        unsigned char c[8];
    }data;
    data.f[0] = msg.distance;
    data.f[1] = msg.yaw;

    unsigned char buf[14]= {0};                                
    int i = 0;
    short len = 0;//数据长度，范围0-255
    //消息头
    buf[i++] = header[0];
    buf[i++] = header[1];
    //数据长度
    len = 8;
    buf[i++] = 0x08;//长度为8
    //数据赋值
    for(int j = 0; j < 8; j++){
        buf[i++] = data.c[j];
    }
    //校验和
    unsigned char crc = getCrc(data.c, len, CRC8_POLYNOMIAL, CRC8_INITIAL_VALUE);
    buf[i++] = crc;
    //消息尾
    buf[i++] = ender[0];
    buf[i++] = ender[1];
    //发送数据
    boost::asio::write(sp, boost::asio::buffer(buf));
    ROS_INFO("send data %.2f %.2f complete:",data.f[0],data.f[1]);
}

//接收函数

//接收12byte数据信息，msg为接收到的字节数组
//数据格式为：帧头  数据长度  数据  校验和  结束符
unsigned char crcr;
int receive12byte(unsigned char* msg){
    unsigned char buf[20]= {0};
    int i = 0;
    int len = 0;
    //接收数据
    boost::asio::read(sp, boost::asio::buffer(buf, 20));
    //数据长度
    len = buf[2];
    //数据
    int k = 3;
    msg[k-3] = buf[k++];
    int 
    //校验和
    crcr = getCrc(buf, 15, CRC8_POLYNOMIAL, CRC8_INITIAL_VALUE);
    if(crcr!= buf[15]){
        ROS_WARN("CRC ERROR!");
        return -1;
    }
    //消息尾
    if(buf[16]!= ender[0] || buf[17]!= ender[1]){
        ROS_WARN("ENDER ERROR!");
        return -1;
    
    }
    ROS_INFO("receive data complete");
    return len;
}

//功能函数
// CRC-8计算函数 data:数据 length:数据长度 polynomial:多项式 initial_value:初始值 polynomial=0x07 initial_value=0x00
unsigned char getCrc(const unsigned char *data, short length, unsigned char polynomial, unsigned char initial_value) {
    unsigned char crc = initial_value; // 初始化CRC值
    for (int i = 0; i < length; i++) {
        crc ^= data[i]; // 当前字节与CRC值异或
        for (int j = 0; j < 8; j++) { // 每字节8位循环
            if (crc & 0x80) { // 检查最高位是否为1
                crc = (crc << 1) ^ polynomial; // 左移并异或生成多项式
            } else {
                crc <<= 1; // 左移，不异或
            }
        }
    }
    return crc; // 返回最终计算的CRC值
}

//关闭串口
void serialClose()
{
    sp.close();
}

//数据处理函数

//机器人ros信息处理
//place_info拆分
void depart_place(const nav_msgs::Odometry::ConstPtr &msg)
{
    ROS_INFO("Received a new odometry message at %d.",msg->header.stamp.sec);

    // 提取位置信息
    const geometry_msgs::Point &position = msg->pose.pose.position;
    const geometry_msgs::Quaternion &orientation = msg->pose.pose.orientation;

    //信息切分
    float a, b, c, d, e, f, g;
    a=position.x;
    b=position.y;
    c=position.z;
    d=orientation.x;
    e=orientation.y;
    f=orientation.z;
    g=orientation.w;

    //提取四元数转换为欧拉角
    tf2::Quaternion q(d, e, f, g);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ROS_INFO("x:%f y:%f z:%f", a, b, c);
    ROS_INFO("roll:%f pitch:%f yaw:%f", roll, pitch, yaw);

    //坐标系变换
    float msg_bef[6] = {a, b, c, roll, pitch, yaw};
    trans2robot(msg_bef);

    //数据赋值
    int i = 0;
    msg_place[i++] = msg_bef[0];
    msg_place[i++] = msg_bef[1];
    msg_place[i++] = msg_bef[5];

    //发送数据
    ROS_INFO("data into send 12byte:");
    send12byte(msg_place);
}

//串口接受数据处理
//void serial_data_process(unsigned char* msg, int len,std::string message) {}

//slam坐标转换机器人坐标
void trans2robot(float* msg){
    //机器人坐标系与slam坐标系关系：x轴正方向相反，y轴正方向相反，z轴正方向相同，均为右手坐标系，且slam坐标系原点位于机器人坐标系的(x,y,z)=(0,0.33662,1.14837)
    float x = msg[0];
    float y = msg[1];
    float z = msg[2];
    float roll = msg[3];
    float pitch = msg[4];
    float yaw = msg[5];
    float x_robot , y_robot, yaw_robot;
    x_robot = x-0.33662*sin(roll);
    y_robot = y-0.33662+0.33662*cos(roll);
    yaw_robot = yaw;
    msg[0] = x_robot;
    msg[1] = y_robot;
    msg[5] = yaw_robot;
}