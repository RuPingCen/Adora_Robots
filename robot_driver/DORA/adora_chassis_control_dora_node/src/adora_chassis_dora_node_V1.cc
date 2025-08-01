extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <sys/time.h>
#include <mutex>

#include <cstdint>
#include <memory>
#include <string.h>
#include <cassert>
#include <string.h>
#include <thread>
#include <chrono>

//#include "chassis_mick_msg.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros_dt_msg.h"
#include "ros_dt_control.h"
#include "geomsgs_twist_msg.h"

#include "serial/serial.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

using namespace std;

 

serial::Serial ros_ser;

 



// 创建一个空的 JSON 对象
json j_pose;
uint32_t count_1=0,count_2;
Geomsgs_Twist cmdvel_twist; // 

uint32_t counter_odom_pub = 0;

/*************************************************************************/
 
#define Base_Width 348 // 轴距

serial::Serial ser; // 声明串口对象
int control_mode = 0;//1开启线速度、角速度反馈模式  2开启速度反馈模式,在launch文件设置

// 初始化串口
string usart_port = "/dev/ttyUSB0";
int baud_data = 115200;
int time_out = 1000;

uint16_t len = 0;
uint8_t data[200];
uint8_t buffer[200] = {0};
 
 
int len_time = 0;

struct Dt1
{
	int32_t vx;
	float vz;
	uint16_t voltage;
	uint16_t state;
};
struct Control1
{
	int16_t vx;
	float vz;
	int16_t lspeed;
	int16_t rspeed;
};
struct Dt2
{
	int16_t lspeed;
	int16_t rspeed;
	int16_t ladden;
	int16_t radden;
	uint16_t voltage;
	uint16_t state;
};
struct Error
{
	uint8_t error;
};

 Dt1 dt1_msg;
 Control1 control_msg;
 Dt2 dt2_msg;

 

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
void static mySigIntHandler(void)
{
    printf("close the com serial!\n");
    open20ms(0);
	//std::this_thread::sleep_for(std::chrono::milliseconds(200));
    //sleep(1);
    // ser.close();
   // ros::shutdown();
   //rclcpp::shutdown();
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

// void dt_control_callback(const geometry_msgs::msg::Twist::SharedPtr &twist_aux)
// {
//     if (control_mode == 1)
//     {
//         dt_control1(twist_aux->linear.x* 1000,twist_aux->angular.z);
//     }
//     else if (control_mode == 2)
//     {
//         s16 TempLSpeed = 0, TempRSpeed = 0;

//         TempLSpeed = twist_aux->linear.x * 1000  - twist_aux->angular.z* Base_Width / 2.0;
//         TempRSpeed = twist_aux->linear.x * 1000 + twist_aux->angular.z* Base_Width / 2.0;
//         dt_control2(TempLSpeed, TempRSpeed);
//     }
// }

void dt_go_charge_callback(u8 status)
{
    openGoCharge(status);
}

void dt_error_clear()
{
    u8 data[10] = {0xED, 0xDE, 0x0A, 0x02, 0x07, 0x01, 0x00, 0x00, 0xDF, 0x01};
    ser.write(data, 10);
}

void dt_error_clear_callback(u8 error_msg)
{
    dt_error_clear();
}

void dt_stop_callback(u8 status)
{
    dtstop(status);
}




void read_uart_buffer(void *dora_context)
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
				//pub1->publish(dt1_msg);

				//std::cout<<"  position_x:  "<<position_x<<"  position_y:  "<<position_y<<"   position_w: " <<position_w<<std::endl; 
				std::cout<<"  linear_x:  "<<dt1_msg.vx<<"  position_y:  "<<0<<"   linear_w: " <<dt1_msg.vz<<std::endl; 
				struct timeval tv;
				gettimeofday(&tv, NULL); 
				json j_odom_pub;
				j_odom_pub["header"]["frame_id"] = "odom";
				j_odom_pub ["header"]["seq"] = counter_odom_pub++;
				j_odom_pub["header"]["stamp"]["sec"] = tv.tv_sec;
				j_odom_pub["header"]["stamp"]["nanosec"] = tv.tv_usec*1e3;
				j_odom_pub["pose"]["position"]["x"] = 0;
				j_odom_pub["pose"]["position"]["y"] = 0;
				j_odom_pub["pose"]["position"]["z"] = 0;
			
				j_odom_pub["pose"]["orientation"]["x"] = 0;
				j_odom_pub["pose"]["orientation"]["y"] = 0;
				j_odom_pub["pose"]["orientation"]["z"] = 0;
				j_odom_pub["pose"]["orientation"]["w"] = 1;
			
				j_odom_pub["twist"]["linear"]["x"] = dt1_msg.vx;
				j_odom_pub["twist"]["linear"]["y"] = 0;
				j_odom_pub["twist"]["linear"]["z"] = 0;
			
				j_odom_pub["twist"]["angular"]["x"] = 0;
				j_odom_pub["twist"]["angular"]["y"] = 0;
				j_odom_pub["twist"]["angular"]["z"] = dt1_msg.vz;
			
			
				// 将 JSON 对象序列化为字符串
				std::string json_string = j_odom_pub.dump(4); // 参数 4 表示缩进宽度
				// 将字符串转换为 char* 类型
				char *c_json_string = new char[json_string.length() + 1];
				strcpy(c_json_string, json_string.c_str());
				std::string out_id = "Odometry";
				// std::cout<<json_string;
				int result = dora_send_output(dora_context, &out_id[0], out_id.length(), c_json_string, std::strlen(c_json_string));
				if (result != 0)
				{
					std::cerr << "failed to send output" << std::endl;
				}

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
				//pub2->publish(dt2_msg);
 
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

int run(void *dora_context);
void cmd_vel_callback(char *data,size_t data_len);

int main()
{
	std::cout << "AdoraMini chassis node for dora " << std::endl;
 
	cout<<"usart_port:   "<<usart_port<<endl;
	cout<<"baud_data:   "<<baud_data<<endl;
	cout<<"control_mode:   "<<control_mode<<endl;
	cout<<"time_out:   "<<time_out<<endl;
 
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
	 

 	auto dora_context = init_dora_context_from_env();
	auto ret = run(dora_context);
	free_dora_context(dora_context);
	mySigIntHandler();
	std::cout << "Exit Adora mini node ..." << std::endl;
	return ret;
}

int run(void *dora_context)
{
    //std::mutex mtx_DoraNavSatFix;
    //std::mutex mtx_DoraQuaternionStamped; // mtx.unlock();
	bool uart_recive_flag;
	uint8_t chassis_type = 1;
	
    while(true)
    {
         
        void *event = dora_next_event(dora_context);
        
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *id;
            size_t id_len;
            read_dora_input_id(event, &id, &id_len);
			//cout<<"id_len: "<<id_len<<endl;

			//read buffer and publish odometry
			read_uart_buffer(dora_context);
 

            if (strncmp(id, "CmdVelTwist",11) == 0)
            {
				char *data;
				size_t data_len;
				read_dora_input_data(event, &data, &data_len);
				cmd_vel_callback(data,data_len);
			}
      }
      else if (ty == DoraEventType_Stop)
      {
          printf("[c node] received stop event\n");
      }
      else
      {
          printf("[c node] received unexpected event: %d\n", ty);
      }
      free_dora_event(event);

    }
    return 0;
}


void cmd_vel_callback(char *data,size_t data_len)
{
	json j_cmd_vel;
	// 将数据转化为字符串
	std::string data_str(data, data_len);
	try 
	{
		j_cmd_vel = json::parse(data_str); // 解析 JSON 字符串               
	} 
	catch (const json::parse_error& e) 
	{
		std::cerr << "JSON 解析错误：" << e.what() << std::endl; // 处理解析失败的情况
		//free_dora_event(event);
	}
	
	count_1++;
	struct timeval tv;
	gettimeofday(&tv, NULL);

	cout << "Twist event count: "<<count_1<<" data_seq "<< j_cmd_vel["seq"]<<" time is: " << 
			std::fixed << std::setprecision(9) << tv.tv_sec +tv.tv_usec*1e-9<<" s " <<std::endl;
	std::cout << "<----print---->" <<j_cmd_vel<< std::endl;
	cmdvel_twist.header.frame_id = j_cmd_vel["header"]["frame_id"];
	cmdvel_twist.header.seq = 	j_cmd_vel ["header"]["seq"];
	cmdvel_twist.header.sec = j_cmd_vel["header"]["stamp"]["sec"];
	cmdvel_twist.header.nanosec = j_cmd_vel["header"]["stamp"]["nanosec"];
	cmdvel_twist.linear.x = j_cmd_vel["linear"]["x"];
	cmdvel_twist.linear.y = j_cmd_vel["linear"]["y"];
	// cmdvel_twist.linear.z = j_cmd_vel["linear"]["z"];
	// cmdvel_twist.angular.x = j_cmd_vel["angular"]["x"];
	// cmdvel_twist.angular.y = j_cmd_vel["angular"]["y"];
	cmdvel_twist.angular.z = j_cmd_vel["angular"]["z"];
  
	cout << "speed_x: "<<cmdvel_twist.linear.x 
		 << "  speed_y: "<<cmdvel_twist.linear.y<< "  speed_w: "<<cmdvel_twist.angular.z<< endl;
	
	if (control_mode == 1)
    {
        dt_control1(cmdvel_twist.linear.x* 1000,cmdvel_twist.angular.z);
    }
    else if (control_mode == 2)
    {
        s16 TempLSpeed = 0, TempRSpeed = 0;

        TempLSpeed = cmdvel_twist.linear.x * 1000  - cmdvel_twist.angular.z* Base_Width / 2.0;
        TempRSpeed = cmdvel_twist.linear.x * 1000 + cmdvel_twist.angular.z* Base_Width / 2.0;
        dt_control2(TempLSpeed, TempRSpeed);
    }
 
 
}
