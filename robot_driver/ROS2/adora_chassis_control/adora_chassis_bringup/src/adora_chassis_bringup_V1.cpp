#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <math.h>
#include <chrono>
#include <thread>
#include <deque>
#include <chrono>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <array>
#include "sys/time.h"

#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <eigen3/Eigen/Geometry>

#include <serial/serial.h> //ROS已经内置了的串口包

#include "ros_dt_msg.h"
#include "ros_dt_control.h"
#include "adora_msgs/msg/control1.hpp"
#include "adora_msgs/msg/dt1.hpp" //要用到 msg 中定义的数据类型
#include "adora_msgs/msg/dt2.hpp"
#include "adora_msgs/msg/error.hpp"

using namespace std;

#define Base_Width 348 // 轴距

serial::Serial ser;   // 声明串口对象
int control_mode = 0; // 1开启线速度、角速度反馈模式  2开启速度反馈模式,在launch文件设置
/*************************************************************************/

static void open20ms(u8 data)
{
    switch (data)
    {
    case 0:
        printf("close20ms");
        break;
    case 1:
        printf("open20ms1");
        break;
    case 2:
        printf("open20ms2");
        break;
    default:
        break;
    }

    dt_Open20MsData.prot.Header = HEADER;
    dt_Open20MsData.prot.Len = 0x0A;
    dt_Open20MsData.prot.Type = 0x02;
    dt_Open20MsData.prot.Cmd = 0x01;
    dt_Open20MsData.prot.Num = 0x01;
    dt_Open20MsData.prot.Data = data;
    dt_Open20MsData.prot.Check = 0;
    for (int i = 0; i < dt_Open20MsData.prot.Len - 2; i++)
    {
        dt_Open20MsData.prot.Check += dt_Open20MsData.data[i];
    }
    ser.write(dt_Open20MsData.data, sizeof(dt_Open20MsData.data));
}

static void openGoCharge(u8 data)
{
    dt_OpenGoCharge.prot.Header = HEADER;
    dt_OpenGoCharge.prot.Len = 0x0A;
    dt_OpenGoCharge.prot.Type = 0x02;
    dt_OpenGoCharge.prot.Cmd = 0x04;
    dt_OpenGoCharge.prot.Num = 1;
    dt_OpenGoCharge.prot.Data = data;
    dt_OpenGoCharge.prot.Check = 0;
    for (int i = 0; i < dt_OpenGoCharge.prot.Len - 2; i++)
    {
        dt_OpenGoCharge.prot.Check += dt_OpenGoCharge.data[i];
    }
    ser.write(dt_OpenGoCharge.data, sizeof(dt_OpenGoCharge.data));
}

static void dtstop(u8 data)
{
    dt_Stop.prot.Header = HEADER;
    dt_Stop.prot.Len = 0x0A;
    dt_Stop.prot.Type = 0x02;
    dt_Stop.prot.Cmd = 0x03;
    dt_Stop.prot.Num = 0x01;
    dt_Stop.prot.Data = data;
    dt_Stop.prot.Check = 0;
    for (int i = 0; i < dt_Stop.prot.Len - 2; i++)
    {
        dt_Stop.prot.Check += dt_Stop.data[i];
    }
    ser.write(dt_Stop.data, sizeof(dt_Stop.data));
}

// 当关闭包时调用，关闭
void mySigIntHandler(int sig)
{
    printf("close the com serial!\n");
    open20ms(0);
    sleep(1);
    // ser.close();
    // ros::shutdown();
    rclcpp::shutdown();
}

void dt_control1(s16 Vx, float Vz)
{
    memset(TXRobotData1.data, 0, sizeof(TXRobotData1.data));

    TXRobotData1.prot.Header = HEADER;
    TXRobotData1.prot.Len = 0x10;
    TXRobotData1.prot.Type = 0x02;
    TXRobotData1.prot.Cmd = 0x02;
    TXRobotData1.prot.Num = 0x04;
    TXRobotData1.prot.Mode = 0;
    TXRobotData1.prot.Vx = Vx;
    TXRobotData1.prot.Vz = Vz;
    TXRobotData1.prot.Check = 0;

    for (u8 i = 0; i < sizeof(TXRobotData1.data) - 2; i++)
    {
        TXRobotData1.prot.Check += TXRobotData1.data[i];
    }

    ser.write(TXRobotData1.data, sizeof(TXRobotData1.data));
}

void dt_control2(s16 lspeed, s16 rspeed)
{
    memset(TXRobotData2.data, 0, sizeof(TXRobotData2.data));

    TXRobotData2.prot.Header = HEADER;
    TXRobotData2.prot.Len = 0x10;
    TXRobotData2.prot.Type = 0x02;
    TXRobotData2.prot.Cmd = 0x02;
    TXRobotData2.prot.Num = 0x04;
    TXRobotData2.prot.Mode = 1;
    TXRobotData2.prot.LSpeed = lspeed;
    TXRobotData2.prot.RSpeed = rspeed;
    TXRobotData2.prot.Check = 0;

    for (u8 i = 0; i < sizeof(TXRobotData2.data) - 2; i++)
    {
        TXRobotData2.prot.Check += TXRobotData2.data[i];
    }

    ser.write(TXRobotData2.data, sizeof(TXRobotData2.data));
}

void dt_control_callback(const geometry_msgs::msg::Twist::ConstPtr &twist_aux)
{
    if (control_mode == 1)
    {
        dt_control1(twist_aux->linear.x * 1000, twist_aux->angular.z);
    }
    else if (control_mode == 2)
    {
        s16 TempLSpeed = 0, TempRSpeed = 0;

        TempLSpeed = twist_aux->linear.x * 1000 - twist_aux->angular.z * Base_Width / 2.0;
        TempRSpeed = twist_aux->linear.x * 1000 + twist_aux->angular.z * Base_Width / 2.0;
        dt_control2(TempLSpeed, TempRSpeed);
    }
}

void dt_go_charge_callback(const std_msgs::msg::UInt8 status)
{
    openGoCharge(status.data);
}

void dt_error_clear()
{
    u8 data[10] = {0xED, 0xDE, 0x0A, 0x02, 0x07, 0x01, 0x00, 0x00, 0xDF, 0x01};
    ser.write(data, 10);
}

void dt_error_clear_callback(const std_msgs::msg::UInt8::ConstPtr &error_msg)
{
    dt_error_clear();
}

void dt_stop_callback(const std_msgs::msg::UInt8 status)
{
    dtstop(status.data);
}

int main(int argc, char **argv)
{
    u16 len = 0;
    u8 data[200];
    u8 buffer[200] = {0};
    std::string usart_port;
    int baud_data;
    int len_time = 0;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("publish_dt_msg");
    // ros::init(argc, argv, "publish_dt_msg", ros::init_options::NoSigintHandler); // 解析参数
    // ros::NodeHandle nh;
    // ros::NodeHandle n("~");

    node->declare_parameter<std::string>("usart_port", usart_port);
    node->declare_parameter<int>("baud_data", baud_data);
    node->declare_parameter<int>("control_mode", control_mode);
    // ros::Subscriber dt_error_sub = nh.subscribe("/collision_release", 100, dt_error_clear_callback); // 防撞解除
    // ros::Subscriber go_charde_sub = nh.subscribe("/go_charge", 100, dt_go_charge_callback);          // 回充 0为关闭，1为开启
    // ros::Subscriber dt_stop_sub = nh.subscribe("/go_stop", 100, dt_stop_callback);                   // 软件急停 0为不急停，1为急停

    rclcpp::Subscription<std_msgs::msg::UInt8::ConstPtr>::SharedPtr dt_error_sub = node->create_subscription<std_msgs::msg::UInt8::ConstPtr>("/collision_release", 10, dt_error_clear_callback);
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr go_charde_sub = node->create_subscription<std_msgs::msg::UInt8>("/go_charge", 10, dt_go_charge_callback);
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr dt_stop_sub = node->create_subscription<std_msgs::msg::UInt8>("/go_stop", 10, dt_stop_callback);

    signal(SIGINT, mySigIntHandler); // 把原来ctrl+c中断函数覆盖掉，把信号槽连接到mySigIntHandler保证关闭节点

    adora_msgs::msg::Dt1 dt1_msg;
    adora_msgs::msg::Control1 control_msg;

    // ros::Publisher pub1 = nh.advertise<ros_dt_msg::dt1>("DT_Robot_agv1", 10); // 创建 publisher 对象
    // ros::Subscriber dt_control_sub = nh.subscribe("/cmd_vel", 100, dt_control_callback);
    rclcpp::Publisher<adora_msgs::msg::Dt1>::SharedPtr pub1 = node->create_publisher<adora_msgs::msg::Dt1>("/DT_Robot_agv1", 20);
    rclcpp::Subscription<geometry_msgs::msg::Twist::ConstPtr>::SharedPtr dt_control_sub = node->create_subscription<geometry_msgs::msg::Twist::ConstPtr>("/cmd_vel", 10, dt_control_callback);

    adora_msgs::msg::Dt2 dt2_msg;
    // ros::Publisher pub2 = nh.advertise<ros_dt_msg::dt2>("DT_Robot_agv2", 10); // 创建 publisher 对象
    rclcpp::Publisher<adora_msgs::msg::Dt2>::SharedPtr pub2 = node->create_publisher<adora_msgs::msg::Dt2>("DT_Robot_agv2", 20);
    try
    {
        // 设置串口属性，并打开串口
        ser.setPort(usart_port);
        ser.setBaudrate(baud_data);
        serial::Timeout to = serial::Timeout::simpleTimeout(50);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        printf("Unable to open port ");
        return -1;
    }

    // 检测串口是否已经打开，并给出提示信息
    if (ser.isOpen())
    {
        // ser.flushInput(); // 清空输入缓存,把多余的无用数据删除
        printf("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    open20ms(control_mode);

    rclcpp::Rate loop_rate(100);

    while (rclcpp::ok())
    {
        len = ser.available();
        if (len >= sizeof(RXRobotData20MS.data))
        {

            ser.read(buffer, len);
            memset(RXRobotData20MS.data, 0, sizeof(RXRobotData20MS.data));
            for (u8 i = 0; i < sizeof(RXRobotData20MS.data); i++)
            {
                RXRobotData20MS.data[i] = buffer[i];
            }
            u16 TempCheck = 0;
            for (u8 i = 0; i < sizeof(RXRobotData20MS.data) - 2; i++)
            {
                TempCheck += RXRobotData20MS.data[i];
            }

            // 头和校验正确
            if (RXRobotData20MS.prot.Header == HEADER && RXRobotData20MS.prot.Check == TempCheck && RXRobotData20MS.prot.Cmd == 0x81)
            {
                len_time = 0;
                for (int i = 0; i < sizeof(RXMode1.data); i++)
                {
                    if (control_mode == 1)
                    {
                        RXMode1.data[i] = RXRobotData20MS.prot.data[i];
                    }
                    else if (control_mode == 2)
                    {
                        RXMode2.data[i] = RXRobotData20MS.prot.data[i];
                    }
                }

                // 消息赋值
                if (control_mode == 1)
                {
                    dt1_msg.vx = RXMode1.prot.Vx;
                    dt1_msg.vz = RXMode1.prot.Vz;
                    dt1_msg.voltage = RXMode1.prot.Voltage;
                    dt1_msg.state = RXMode1.prot.State;
                    pub1->publish(dt1_msg);
                    memset(RXMode1.data, 0, sizeof(RXMode1.data));
                }
                else if (control_mode == 2)
                {
                    dt2_msg.lspeed = RXMode2.prot.LSpeed;
                    dt2_msg.rspeed = RXMode2.prot.RSpeed;
                    dt2_msg.ladden = RXMode2.prot.LAddEN;
                    dt2_msg.radden = RXMode2.prot.RAddEN;
                    dt2_msg.voltage = RXMode2.prot.Voltage;
                    dt2_msg.state = RXMode2.prot.State;
                    pub2->publish(dt2_msg);
                    memset(RXMode2.data, 0, sizeof(RXMode2.data));
                }
            }
            else
            {
                printf("not read accuracy,SUM:%02X,Check:%02X\n\n", TempCheck, RXRobotData20MS.prot.Check);
                len = ser.available();
                // 清空数据残余
                if (len > 0 && len < 200)
                {
                    ser.read(data, len);
                }
                else
                {
                    ser.read(data, 200);
                }
                len_time = 0;
            }
        }
        else
        {
            len_time++;
            if (len_time > 100)
            {
                printf("len_time:%d\n", len_time);
                len_time = 0;
                open20ms(control_mode);
                printf("ros dt open 20cm\n");
            }
        }

        // ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
