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
int control_mode = 1;//1开启线速度、角速度反馈模式  2开启速度反馈模式,在launch文件设置

// 初始化串口
string usart_port ;
int baud_data;
int time_out;

uint16_t len = 0;
uint8_t data[200];
uint8_t buffer[200] = {0};
 
 
int len_time = 0;

 
struct AdoraA2Pro
{
	uint8_t control_mode;
	uint8_t percentage; //取值：0% - 100%
	uint16_t voltage; // 0.1V
	uint16_t flage;
	uint16_t error_flage;
	int16_t vx; //线速度 单位：mm/s
	int16_t wz; //角速度 单位：0.001rad/s
	int16_t vl;//左轮速 单位：mm/s
	int16_t vr;//右轮速 单位：mm/s
}dadoraa2pro_msg;
union Int16Data //union的作用为实现char数组和int16数据类型之间的转换
{
    int16_t int16_dat;
    unsigned char byte_data[2];
}Int16Data_tem;
 

 



//0：关闭  1：开启
static void adora_a2pro_stop(u8 data)
{
  AdoraA2Pro_Stop.prot.Header = HEADER;
  AdoraA2Pro_Stop.prot.Len = 0x07;
  AdoraA2Pro_Stop.prot.Cmd = 0x22;
  AdoraA2Pro_Stop.prot.Data = data;
  AdoraA2Pro_Stop.prot.Check = 0;
    for (int i = 0; i < AdoraA2Pro_Stop.prot.Len - 2; i++)
    {
      AdoraA2Pro_Stop.prot.Check += AdoraA2Pro_Stop.data[i];
    }
    ser.write(AdoraA2Pro_Stop.data, sizeof(AdoraA2Pro_Stop.data));
}


//4.1.1 状态上传开关 [下发帧]
void enable_states_upload(u8 enable_flag)
{
  
  memset(EnableStateUploadData.data, 0, sizeof(EnableStateUploadData.data));
  
  EnableStateUploadData.prot.Header = HEADER;
  EnableStateUploadData.prot.Len = 0x07;
  EnableStateUploadData.prot.Cmd = 0x01;
  EnableStateUploadData.prot.EnableState = enable_flag;   
  for (u8 i = 0; i < sizeof(EnableStateUploadData.data) - 2; i++)
  {
    EnableStateUploadData.prot.Check += EnableStateUploadData.data[i];
  }

  ser.write(EnableStateUploadData.data, sizeof(EnableStateUploadData.data));
}
// 当关闭包时调用，关闭
void static mySigIntHandler(void)
{
    printf("close the com serial!\n");
    enable_states_upload(0);
	//std::this_thread::sleep_for(std::chrono::milliseconds(200));
    //sleep(1);
    // ser.close();
   // ros::shutdown();
   //rclcpp::shutdown();
}

 
//4.11.1 线速角速控制 [下发帧]
// 线速度 单位：mm/s    角速度 单位：0.001 rad/s
void adora_a2pro_control1(s16 Vx, s16 Vw)
{
    memset(AdoraA2ProData1.data, 0, sizeof(AdoraA2ProData1.data));
  
    AdoraA2ProData1.prot.Header = HEADER;
    AdoraA2ProData1.prot.Len = 0x0A;
    AdoraA2ProData1.prot.Cmd = 0x20;
    AdoraA2ProData1.prot.Vx = Vx;  //线速度 单位：mm/s
    AdoraA2ProData1.prot.Vw = Vw;  //角速度 单位：0.001 rad/s 
    for (u8 i = 0; i < sizeof(AdoraA2ProData1.data) - 2; i++)
    {
      AdoraA2ProData1.prot.Check += AdoraA2ProData1.data[i];
    }

    ser.write(AdoraA2ProData1.data, sizeof(AdoraA2ProData1.data));

}

//4.11.1 线速角速控制 [下发帧]
// 左轮速 单位：mm/s    右轮速 单位：mm/s
void adora_a2pro_control2(s16 Lv, s16 Rv)
{
    memset(AdoraA2ProData2.data, 0, sizeof(AdoraA2ProData2.data));
  
    AdoraA2ProData2.prot.Header = HEADER;
    AdoraA2ProData2.prot.Len = 0x0A;
    AdoraA2ProData2.prot.Cmd = 0x21;
    AdoraA2ProData2.prot.Lv = Lv;  //左轮速 单位：mm/s
    AdoraA2ProData2.prot.Rv = Rv;  //右轮速 单位：mm/s
    for (u8 i = 0; i < sizeof(AdoraA2ProData2.data) - 2; i++)
    {
        AdoraA2ProData2.prot.Check += AdoraA2ProData2.data[i];
    }

    ser.write(AdoraA2ProData2.data, sizeof(AdoraA2ProData2.data));

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

// void dt_go_charge_callback(u8 status)
// {
//     openGoCharge(status);
// }

// void dt_error_clear()
// {
//     u8 data[10] = {0xED, 0xDE, 0x0A, 0x02, 0x07, 0x01, 0x00, 0x00, 0xDF, 0x01};
//     ser.write(data, 10);
// }

// void dt_error_clear_callback(u8 error_msg)
// {
//     dt_error_clear();
// }

// void dt_stop_callback(u8 status)
// {
//     dtstop(status);
// }




void read_uart_buffer(void *dora_context)
{
	
	len = ser.available();
	std::cout<<"  flage1:  len:"<<len<<std::endl;
	if (len >= sizeof(AdoraA2Pro_RxData_ChassisState.data))
	{
		std::cout<<"  flage2  "<<std::endl;
		ser.read(buffer, len);
		memset(AdoraA2Pro_RxData_ChassisState.data, 0, sizeof(AdoraA2Pro_RxData_ChassisState.data));
		for (u8 i = 0; i < sizeof(AdoraA2Pro_RxData_ChassisState.data); i++)
		{
			AdoraA2Pro_RxData_ChassisState.data[i] = buffer[i];
		}
		u16 TempCheck = 0;
		for(u8 i=0;i<sizeof(AdoraA2Pro_RxData_ChassisState.data)-2;i++)
		{
			TempCheck += AdoraA2Pro_RxData_ChassisState.data[i];
		}
		std::cout<<"  flage3  "<<std::endl;
		// 头和校验正确
		if (AdoraA2Pro_RxData_ChassisState.prot.Header == HEADER && AdoraA2Pro_RxData_ChassisState.prot.Check == TempCheck && AdoraA2Pro_RxData_ChassisState.prot.Cmd == 0x80)
		{
			len_time = 0;
			 
			dadoraa2pro_msg.control_mode = AdoraA2Pro_RxData_ChassisState.data[4];
			dadoraa2pro_msg.percentage = AdoraA2Pro_RxData_ChassisState.data[5];
			dadoraa2pro_msg.voltage	= AdoraA2Pro_RxData_ChassisState.data[7]<<8+AdoraA2Pro_RxData_ChassisState.data[6];
			dadoraa2pro_msg.flage	= AdoraA2Pro_RxData_ChassisState.data[9]<<8+AdoraA2Pro_RxData_ChassisState.data[8];
			dadoraa2pro_msg.error_flage	= AdoraA2Pro_RxData_ChassisState.data[11]<<8+AdoraA2Pro_RxData_ChassisState.data[10];
			
			Int16Data_tem.int16_dat = 0;
			Int16Data_tem.byte_data[0] = AdoraA2Pro_RxData_ChassisState.data[12];
			Int16Data_tem.byte_data[1] = AdoraA2Pro_RxData_ChassisState.data[13];
			dadoraa2pro_msg.vx	= Int16Data_tem.int16_dat;

			Int16Data_tem.int16_dat = 0;
			Int16Data_tem.byte_data[0] = AdoraA2Pro_RxData_ChassisState.data[14];
			Int16Data_tem.byte_data[1] = AdoraA2Pro_RxData_ChassisState.data[15];
			dadoraa2pro_msg.wz	= Int16Data_tem.int16_dat;

			Int16Data_tem.int16_dat = 0;
			Int16Data_tem.byte_data[0] = AdoraA2Pro_RxData_ChassisState.data[16];
			Int16Data_tem.byte_data[1] = AdoraA2Pro_RxData_ChassisState.data[17];
			dadoraa2pro_msg.vl	= Int16Data_tem.int16_dat;

			Int16Data_tem.int16_dat = 0;
			Int16Data_tem.byte_data[0] = AdoraA2Pro_RxData_ChassisState.data[18];
			Int16Data_tem.byte_data[1] = AdoraA2Pro_RxData_ChassisState.data[19];
			dadoraa2pro_msg.vr	= Int16Data_tem.int16_dat;



			std::cout<<"  control_mode:  "<<dadoraa2pro_msg.control_mode
					 <<"  percentage:  "<<dadoraa2pro_msg.control_mode
					 <<"  voltage:  "<<dadoraa2pro_msg.voltage
					 <<"  flage:  "<<dadoraa2pro_msg.flage
					 <<"  error_flage:  "<<dadoraa2pro_msg.error_flage
					 <<"   vx: " <<dadoraa2pro_msg.vx
					 <<"   wz: " <<dadoraa2pro_msg.wz
					 <<std::endl; 
		 

			// 消息赋值
			if (control_mode == 1)
			{
			  
				//std::cout<<"  position_x:  "<<position_x<<"  position_y:  "<<position_y<<"   position_w: " <<position_w<<std::endl; 
				std::cout<<"  linear_x:  "<<dadoraa2pro_msg.vx<<"  position_y:  "<<0<<"   linear_w: " <<dadoraa2pro_msg.wz<<std::endl; 
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
			
				j_odom_pub["twist"]["linear"]["x"] = dadoraa2pro_msg.vx;
				j_odom_pub["twist"]["linear"]["y"] = 0;
				j_odom_pub["twist"]["linear"]["z"] = 0;
			
				j_odom_pub["twist"]["angular"]["x"] = 0;
				j_odom_pub["twist"]["angular"]["y"] = 0;
				j_odom_pub["twist"]["angular"]["z"] = dadoraa2pro_msg.wz;
			
			
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
			//open20ms(control_mode);
			enable_states_upload(1);
			printf("ros dt open 20cm\n");               
		}
	}         
}

int run(void *dora_context);
void analy_dora_input_data(char *data,size_t data_len);
void cmd_vel_callback(float speed_x,float speed_w);
int main()
{
	std::cout << "AdoraMini chassis node for dora " << std::endl;
 
	const char* port = std::getenv("USART_PORT");
	usart_port = std::string(port);
	baud_data = std::getenv("USART_BUAD") ? std::stoi(std::getenv("USART_BUAD")) : 115200;
	time_out = std::getenv("USART_TIME_OUT") ? std::stoi(std::getenv("USART_TIME_OUT")) : 1000;


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
	enable_states_upload(1);
	 

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
				analy_dora_input_data(data,data_len);
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


void analy_dora_input_data(char *data,size_t data_len)
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
	//std::cout << "<----print---->" <<j_cmd_vel<< std::endl;
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
	
	// linear m/s     angular  rad/s
	cmd_vel_callback(cmdvel_twist.linear.x,cmdvel_twist.angular.z);
 
 
}

void cmd_vel_callback(float speed_x,float speed_w)
{
  
	if (control_mode == 1)
    {
      adora_a2pro_control1(speed_x* 1000,speed_w* 1000);
    }
    else if (control_mode == 2)
    {
        s16 TempLSpeed = 0, TempRSpeed = 0;

        TempLSpeed = speed_x * 1000  - speed_w* Base_Width / 2.0;
        TempRSpeed = speed_x * 1000 + speed_w* Base_Width / 2.0;
        adora_a2pro_control2(TempLSpeed, TempRSpeed);
    }
 
}
