#ifndef IMU_MSG_H
#define IMU_MSG_H

#include "vector3.h"
// struct Vector3
// {
//     float x;
//     float y;
//     float z;
// };

struct imu_msg_h
{
    double stamp;
	struct vector3 linear_acceleration;
	struct vector3 angular_velocity;
};
#endif