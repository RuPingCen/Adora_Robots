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
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <eigen3/Eigen/Geometry>

#include <serial/serial.h> //ROS已经内置了的串口包

// 其中数值 3276800 表示电机转动100圈，电机转动1圈的数值是32768.
uint32_t MIN_MOTOR_POSITION = 0;
uint32_t MAX_MOTOR_POSITION = 36044000; // 勿修改！！！ 32768*1100(圈) = 36044800 对应700mm行程的丝杠，
float Ratio_K_1 = 0.63636364;           // 电机每转动一圈丝杆行进的距离（单位 mm ）
float Ratio_K_2 = 32768;                // 电机转动1圈的编码器反馈的数值是32768

using namespace std;

class SerialLiftingMotor : public rclcpp::Node
{
public:
    SerialLiftingMotor()
        : Node("publish_dt_msg")
    {
        // 参数声明和获取
        dev_ = "/dev/ttyACM0";
        baud = 19200;

        sub_cmdvel_topic = "/adora/lifting_motor/cmd";
        pub_position_topic = "/adora/lifting_motor/states";

        this->declare_parameter<std::string>("dev", dev_);
        this->declare_parameter<int>("baud", baud);
        // declare_parameter<int>("time_out",time_out);
        // declare_parameter<int>("hz",hz);
        this->declare_parameter<double>("delay", delay);
        this->declare_parameter<int>("max_lifting_distance", max_lifting_distance);
        this->declare_parameter<std::string>("robot_name", robot_name);
        this->declare_parameter<std::string>("sub_cmdvel_topic", sub_cmdvel_topic);
        this->declare_parameter<std::string>("pub_position_topic", pub_position_topic);

        this->get_parameter("dev", dev_);
        this->get_parameter("baud", baud);
        this->get_parameter("delay", delay);
        this->get_parameter("max_lifting_distance", max_lifting_distance);
        this->get_parameter("robot_name", robot_name);
        this->get_parameter("sub_cmdvel_topic", sub_cmdvel_topic);
        this->get_parameter("pub_position_topic", pub_position_topic);

        RCLCPP_INFO_STREAM(this->get_logger(), "dev:   " << dev_);
        RCLCPP_INFO_STREAM(this->get_logger(), "baud:   " << baud);

        RCLCPP_INFO_STREAM(this->get_logger(), "sub_cmdvel_topic:   " << sub_cmdvel_topic);
        RCLCPP_INFO_STREAM(this->get_logger(), "pub_position_topic:   " << pub_position_topic);

        // 比较两个字符数组，最多比较 LEN 个字符
        if (robot_name.length() >= 12)
        {
            if (((strncmp(robot_name.c_str(), "ADORA_A2_MAX", 12) == 0) && (max_lifting_distance == 700)) ||
                ((strncmp(robot_name.c_str(), "ADORA_A2_PRO", 12) == 0) && (max_lifting_distance == 1100)))
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "robot_name:    " << robot_name);
                RCLCPP_INFO_STREAM(this->get_logger(), "max_lifting_distance:   " << max_lifting_distance);
            }
            else
            {
                RCLCPP_FATAL_STREAM(this->get_logger(), "robot_name and max_lifting_distance do not match !!!"
                                                            << "\n robot_name " << robot_name
                                                            << "  max_lifting_distance " << max_lifting_distance);
                return;
            }
        }
        else
        {
            RCLCPP_FATAL_STREAM(this->get_logger(), "robot_name and max_lifting_distance do not match !!!"
                                                        << "robot_name " << robot_name
                                                        << "max_lifting_distance" << max_lifting_distance);
            return;
        }

        command_sub = this->create_subscription<std_msgs::msg::UInt32>(sub_cmdvel_topic, 10, std::bind(&SerialLiftingMotor::cmd_vel_callback, this, std::placeholders::_1));

        position_pub = this->create_publisher<std_msgs::msg::UInt32>(pub_position_topic, 20);

        try
        {
            // 设置串口属性，并打开串口
            ser.setPort(dev_);
            ser.setBaudrate(baud);
            serial::Timeout to = serial::Timeout::simpleTimeout(50);
            ser.setTimeout(to);
            ser.open();
        }
        catch (serial::IOException &e)
        {
            printf("Unable to open port ");
            return;
        }

        // 检测串口是否已经打开，并给出提示信息
        if (ser.isOpen())
        {
            // ser.flushInput(); // 清空输入缓存,把多余的无用数据删除
            printf("Serial Port initialized");
        }
        else
        {
            return;
        }
        motor_init();
        // 设置定时调用函数，用于读取串口缓冲区
        timer_ = create_wall_timer(20ms, std::bind(&SerialLiftingMotor::read_uart_buffer, this));
    }

private:
    std::string dev_;
    std::string sub_cmdvel_topic, pub_position_topic;
    int baud;
    int max_lifting_distance = 700;
    std::string robot_name = "ADORA_A2_MAX";
    double delay;
    serial::Serial ser;
    uint16_t uart_len = 0;
    uint8_t uart_buffer[1024] = {0};
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr command_sub;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr position_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    uint8_t auchCRCHi[256] = // CRC 高位字节值表
        {
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
            0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
            0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
            0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
            0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
            0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
            0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
            0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
            0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
            0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
            0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
            0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
            0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};

    uint8_t auchCRCLo[256] = {
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05,
        0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA,
        0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA,
        0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15,
        0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0,
        0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35,
        0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B,
        0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA,
        0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27,
        0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
        0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64,
        0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
        0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE,
        0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7,
        0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
        0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
        0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99,
        0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E,
        0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46,
        0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40};

    union INT32Data // union的作用为实现char数组和int32之间的转换
    {
        int32_t int32_dat;
        unsigned char byte_data[4];
    } tem_INT32Data;

    void cmd_vel_callback(const std_msgs::msg::UInt32 msg)
    {
        int tem_distance = msg.data;

        if (tem_distance > max_lifting_distance)
            tem_distance = max_lifting_distance - 5; // 预留5mm行程

        // 丝杠起点位置： 0
        // 丝杠终点位置： 32768*1100(圈) = 36044800 对应700mm行程的丝杠， 0.63636363
        // 其中32768是电机外部的转轴转动一圈的值。
        // 700mm/0.63636363*32768
        float tem_value = tem_distance / Ratio_K_1 * Ratio_K_2;
        printf("recived distance  (mm): %d  , control value:  %2.f\n", tem_distance, tem_value);
        motor_position_set(tem_value / 1);
    }

    void read_uart_buffer(void)
    {
        uart_len = 0;
        uart_len = ser.available();
        uart_len = ser.read(uart_buffer, 20);
        if (uart_len < 7)
        {
            printf("data len=%d < 7", uart_len);
            return;
        }

        // printf("\n recived  value:  ");
        // for(int i=0;i<uart_len;i++)
        //     printf("0x%X  ", uart_buffer[i]);

        if (uart_buffer[0] == 0x01 && uart_buffer[1] == 0x03 && uart_buffer[2] == 0x04)
        {
            tem_INT32Data.int32_dat = 0;
            tem_INT32Data.byte_data[0] = uart_buffer[4];
            tem_INT32Data.byte_data[1] = uart_buffer[3];
            tem_INT32Data.byte_data[2] = uart_buffer[6];
            tem_INT32Data.byte_data[3] = uart_buffer[5];

            uint32_t tem_distance = (tem_INT32Data.int32_dat / Ratio_K_2 * Ratio_K_1) / 1;
            printf("position: %d  ,distance:(mm) %d \n", tem_INT32Data.int32_dat, tem_distance);

            std_msgs::msg::UInt32 msg;
            msg.data = tem_distance;
            position_pub->publish(msg);
        }

        // for next time
        motor_position_read();
    }
    void motor_init(void)
    {
        uint8_t data_tem_1[8] = {0x01, 0x06, 0x00, 0x00, 0x00, 0x01, 0x48, 0x0A};
        uint8_t data_tem_2[8] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x01, 0x48, 0xCA};
        uint8_t data_tem_3[8] = {0x01, 0x06, 0x00, 0x03, 0x13, 0x88, 0x74, 0x9C};
        uint8_t data_tem_4[8] = {0x01, 0x06, 0x00, 0x02, 0x05, 0xDC, 0x2A, 0xC3};

        ser.write(data_tem_1, 8);
        usleep(50000);
        ser.write(data_tem_2, 8);
        usleep(50000);
        ser.write(data_tem_3, 8);
        usleep(50000);
        ser.write(data_tem_4, 8);
        usleep(50000);
    }
    //  puchMsg 要进行 CRC 校验的消息
    //  usDataLen 消息中字节数
    void calculate_crc16(uint8_t *puchMsg, uint16_t usDataLen,
                         uint8_t &HSB, uint8_t &LSB)
    {
        // unsigned short CRC16(puchMsg, usDataLen)
        unsigned char uchCRCHi = 0xFF; // 高 CRC 字节初始化
        unsigned char uchCRCLo = 0xFF; // 低 CRC 字节初始化
        unsigned char uIndex;          // CRC 循环中的索引
        while (usDataLen--)            // 传输消息缓冲区
        {
            uIndex = uchCRCHi ^ *puchMsg++; // 计算 CRC
            uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
            uchCRCLo = auchCRCLo[uIndex];
        }
        LSB = uchCRCLo;
        HSB = uchCRCHi;
    }
    void motor_position_read(void)
    {
        uint8_t data_tem_1[8] = {0x01, 0x03, 0x00, 0x16, 0x00, 0x02, 0xFF, 0xFF};
        // data_array_1.extend([0x01, 0x03, 0x00, 0x16, 0x00, 0x02])  # read motor global positio
        calculate_crc16(data_tem_1, 6, data_tem_1[6], data_tem_1[7]);
        ser.write(data_tem_1, 8);
    }

    //  电机M8010HG8 额定转速2500RPM (加速比8    0-277RPM)  4NM 的型号
    // 丝杠起点位置： 0
    // 丝杠终点位置： 32768*1100(圈) = 36044800 对应700mm行程的丝杠，
    // 其中32768是电机外部的转轴转动一圈的值。
    void motor_position_set(uint32_t position_value)
    {
        uint8_t data_tem_1[8] = {0x01, 0x7b, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF};
        // set motor position
        if (position_value < MIN_MOTOR_POSITION)
            position_value = MIN_MOTOR_POSITION;
        else if (position_value > MAX_MOTOR_POSITION)
            position_value = MAX_MOTOR_POSITION;

        data_tem_1[2] = (position_value >> 24) & 0xff;
        data_tem_1[3] = (position_value >> 16) & 0xff;
        data_tem_1[4] = (position_value >> 8) & 0xff;
        data_tem_1[5] = position_value & 0xff;
        calculate_crc16(data_tem_1, 6, data_tem_1[6], data_tem_1[7]);
        ser.write(data_tem_1, 8);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto serial_lifting_motor_node = std::make_shared<SerialLiftingMotor>();
    rclcpp::spin(serial_lifting_motor_node);
    rclcpp::shutdown();
    return 0;
}
