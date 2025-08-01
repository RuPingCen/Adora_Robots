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

serial::Serial ser; // 声明串口对象
int control_mode = 1;//1开启线速度、角速度反馈模式  2开启速度反馈模式,在launch文件设置
/*************************************************************************/
 
 
 
using namespace std;
 
class ADoraA1MiniNode : public rclcpp::Node
{
public:
    ADoraA1MiniNode()
        :Node("adora_bringup_node")
    {
        // 参数声明和获取
		this->declare_parameter<std::string>("usart_port",usart_port);
		this->declare_parameter<int>("baud_data",baud_data);
		this->declare_parameter<int>("control_mode", control_mode);
 
        this->get_parameter("usart_port", usart_port);
        this->get_parameter("baud_data", baud_data);
        this->get_parameter("control_mode", control_mode);

		RCLCPP_INFO_STREAM(this->get_logger(),"usart_port:   "<<usart_port);
		RCLCPP_INFO_STREAM(this->get_logger(),"baud_data:   "<<baud_data);
		RCLCPP_INFO_STREAM(this->get_logger(),"control_mode:   "<<control_mode);
 

        signal(SIGINT, ADoraA1MiniNode::mySigIntHandler);      
 

        dt_error_sub = this->create_subscription<std_msgs::msg::UInt8>("collision_release", 
                            10,std::bind(&ADoraA1MiniNode::dt_error_clear_callback,this,std::placeholders::_1));
                            
        go_charde_sub = this->create_subscription<std_msgs::msg::UInt8> ("go_charge",
                             10,std::bind(&ADoraA1MiniNode::dt_go_charge_callback,this,std::placeholders::_1));
    
        dt_stop_sub = this->create_subscription<std_msgs::msg::UInt8>("go_stop",
                             10,std::bind(&ADoraA1MiniNode::dt_stop_callback,this,std::placeholders::_1));
                            

        //ros::Publisher pub1 = nh.advertise<ros_dt_msg::dt1>("DT_Robot_agv1", 10); // 创建 publisher 对象
        //ros::Subscriber dt_control_sub = nh.subscribe("/cmd_vel", 100, dt_control_callback);
        pub1 = this->create_publisher<adora_msgs::msg::Dt1>("DT_Robot_agv1",20); 

        dt_control_sub = this->create_subscription<geometry_msgs::msg::Twist >("cmd_vel",
                            10,std::bind(&ADoraA1MiniNode::dt_control_callback,this,std::placeholders::_1));
    

        pub2 = this->create_publisher<adora_msgs::msg::Dt2>("DT_Robot_agv2",20); 

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
            return ;
        }
    
        // 检测串口是否已经打开，并给出提示信息
        if (ser.isOpen())
        {
            // ser.flushInput(); // 清空输入缓存,把多余的无用数据删除
            printf("Serial Port initialized");
        }
        else
        {
            return ;
        }
        open20ms(control_mode);
    
 
		// 设置定时调用函数，用于读取串口缓冲区
        timer_ = create_wall_timer(10ms, std::bind(&ADoraA1MiniNode::read_uart_buffer, this));
    }

private:
    uint16_t len = 0;
    uint8_t data[200];
    uint8_t buffer[200] = {0};
    std::string usart_port = "/dev/ttyUSB0";
    int baud_data = 115200;
    int len_time = 0;

    adora_msgs::msg::Dt1 dt1_msg;
    adora_msgs::msg::Control1 control_msg;
    adora_msgs::msg::Dt2 dt2_msg;

	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr dt_error_sub;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr go_charde_sub ;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr dt_stop_sub;
    
    rclcpp::Publisher<adora_msgs::msg::Dt1>::SharedPtr pub1;
    rclcpp::Subscription<geometry_msgs::msg::Twist >::SharedPtr dt_control_sub;
    rclcpp::Publisher<adora_msgs::msg::Dt2>::SharedPtr pub2;


    rclcpp::TimerBase::SharedPtr timer_;

     
 
	void read_uart_buffer(void)
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
            for(u8 i=0;i<sizeof(RXRobotData20MS.data)-2;i++)
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
                printf("not read accuracy,SUM:%02X,Check:%02X\n\n",TempCheck,RXRobotData20MS.prot.Check );
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
                printf("len_time:%d\n",len_time);
                len_time = 0;
                open20ms(control_mode);
                printf("ros dt open 20cm\n");               
            }
        }         
	}



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
    void static mySigIntHandler(int sig)
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
        TXRobotData1.prot.Vx = Vx;// (int16)(mm/s)
        TXRobotData1.prot.Vz = Vz;// (float)(rad/s)
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

    void dt_control_callback(const geometry_msgs::msg::Twist &twist_aux)
    {
        if (control_mode == 1)
        {
            // mm/s   rad/s
            
            RCLCPP_INFO_STREAM(this->get_logger(),"recived data vx: "<<twist_aux.linear.x<<"\twz: "<<twist_aux.angular.z);
 
            dt_control1(twist_aux.linear.x* 1000,twist_aux.angular.z);
        }
        else if (control_mode == 2)
        {
            s16 TempLSpeed = 0, TempRSpeed = 0;

            TempLSpeed = twist_aux.linear.x * 1000  - twist_aux.angular.z* Base_Width / 2.0;
            TempRSpeed = twist_aux.linear.x * 1000 + twist_aux.angular.z* Base_Width / 2.0;
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

    void dt_error_clear_callback(const std_msgs::msg::UInt8 &error_msg)
    {
        dt_error_clear();
    }

    void dt_stop_callback(const std_msgs::msg::UInt8 status)
    {
        dtstop(status.data);
    }
 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto chassis_node = std::make_shared<ADoraA1MiniNode>();
    rclcpp::spin(chassis_node);
    rclcpp::shutdown();
    return 0;
}