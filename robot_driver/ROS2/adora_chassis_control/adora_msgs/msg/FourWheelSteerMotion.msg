
uint8 motion_mode  #0:ackermann  1:four_wheel_independent_steering

#Ackermann

float32 linear  #unit: m/s
float32 steering  #unit: rad
float32 angular  #unit: rad/s

#Four-Wheel Independent Steering

float32 linear_x  #unit: m/s
float32 linear_y  #unit: m/s
float32 angular_z  #unit: rad/s
