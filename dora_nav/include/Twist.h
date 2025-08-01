#pragma once
#include "Vector3.h"
namespace geometry_msgs
{
    struct Twist
    {
        Vector3 linear;
        Vector3 angular;
    };

}