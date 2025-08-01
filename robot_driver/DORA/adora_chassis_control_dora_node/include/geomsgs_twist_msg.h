#ifndef Geomsgs_Twist_MSG_H
#define Geomsgs_Twist_MSG_H

#include <stdint.h>
#include <iostream>
#include <vector>


struct Header{
    // 使用与 Python 类中相同的数据类型和字段名称
    std::string frame_id;
    //std::string stamp;
    uint32_t seq;
    int sec;
    int nanosec;
};

struct Position  {
  double x;
  double y;
  double z;
  double vx;
  double vy;
  double vz;
};

 
struct Quaternion final {
  double x;
  double y;
  double z;
  double w;
 
};

struct IMU {
  Header header;
  double ax,ay,az;
  double gx,gy,gz;
  double qx,qy,qz,qw;
  double roll,pitch,yaw;
};

struct XYZ {
  double x,y,z;
};
struct Geomsgs_Twist {
  Header header;
  XYZ linear;
  XYZ angular;
};

   
 
 
#endif 
