#ifndef ROS_DT_MSG_H
#define ROS_DT_MSG_H

//#include <ros/ros.h>
//#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#define HEADER    0xDEED		//数据头

typedef char s8;
typedef unsigned char     u8;
typedef unsigned short int u16;
typedef short int    s16;
typedef unsigned  int u32;
typedef int s32;

#define HEADE_BYTE_NUM    (2)     /* 帧/包头   占2byte */
#define LEN_BYTE_NUM      (1)     /* 帧/包长度 占1byte */
#define CMD_BYTE_NUM      (1)     /* 命令码    占1byte */
#define TYPE_BYTE_NUM      (1)     /* 命令码    占1byte */
#define DATA_BYTE_NUM     (40)    /* 数据域     */
#define CHECK_BYTE_NUM    (2)     /* 校验码    占2byte */

//#define RX_BUFFER_SIZE    (HEADE_BYTE_NUM + LEN_BYTE_NUM + CMD_BYTE_NUM + TYPE_BYTE_NUM + DATA_BYTE_NUM + CHECK_BYTE_NUM)    /* 串口收数据缓存大小 */
//#define TX_BUFFER_SIZE    (HEADE_BYTE_NUM + LEN_BYTE_NUM + CMD_BYTE_NUM + TYPE_BYTE_NUM + DATA_BYTE_NUM + CHECK_BYTE_NUM)    /* 串口发数据缓存大小 */

#define RX_BUFFER_SIZE   (48)    


#define SERIAL_PACK_HEAD_1    (0xDE)
#define SERIAL_PACK_HEAD_2    (0xED)

// #define TX_BUFFER_SIZE   (18)
// struct serial_msg_check_s
// {
//         u8 tx_buffer[TX_BUFFER_SIZE];
// };
// typedef struct serial_msg_check_s serial_msg_check_t;
// serial_msg_check_t serial_msg_check;

// struct return_robot_data_s
// {
//       u8 rx_buffer[RX_BUFFER_SIZE];
// };
// typedef struct return_robot_data_s return_robot_data_t;
// return_robot_data_t return_robot_data;


union RxRobotData20MS
{
   u8 data[48];
   struct
   {
      u16 Header;       
      u8  Len;       
      u8  Type;         
      u8  Cmd;       
      u8  Num;       
      u8  data[40];     
      u16 Check;        
   }prot;
}RXRobotData20MS;

//打开20ms数据上传结构体
union OPen20MsData
{
   u8 data[10];
   struct
   {
      u16 Header;
      u8  Len;
      u8  Type;
      u8  Cmd;
      u8  Num;
      u16 Data;
      u16 Check;
   }prot;
}dt_Open20MsData,dt_OpenGoCharge,dt_Stop;

//下发轮子速度结构体
#pragma pack(1)
union TXRobotData1
{
   u8 data[16];
   struct
   {
      u16 Header;
      u8  Len;
      u8  Type;
      u8  Cmd;
      u8  Num; 
      s16 Mode;
      s16 Vx;
      float Vz;
      u16 Check;
   }prot;
}TXRobotData1;
#pragma pack()

//下发轮子速度结构体
union TXRobotData2
{
   u8 data[16];
   struct
   {
      u16 Header;
      u8  Len;
      u8  Type;
      u8  Cmd;
      u8  Num; 
      s16 Mode;
      s16 LSpeed;
      s16 RSpeed;
      u16 NC;  
      u16 Check;
   }prot;
}TXRobotData2;

union RESMode1
{
   u8 data[40];
   struct
   {
      s32   Vx;         //
      float Vz;         //
      float AccX;       //
      float AccY;       //
      float AccZ;       //
      float GyrX;       //
      float GyrY;       //
      float GyrZ;       //
      u16   Voltage;    //
      u16    State;    //
      u16   Light12;    //
      u16   Light34;    //
   }prot;
}RXMode1;

union RESMode2
{
   u8 data[40];
   struct
   {
      s16   LSpeed;     //
      s16   RSpeed;     //
      s16   LAddEN;     //
      s16   RAddEN;     //
      float AccX;       //
      float AccY;       //
      float AccZ;       //
      float GyrX;       //
      float GyrY;       //
      float GyrZ;       //
      u16   Voltage;    //
      u16     State;    //
      u16   Light12;    //
      u16   Light34;    //
   }prot;
}RXMode2;

#endif // ROS_DT_MSG_H
