# Adora  chassis node

 support: **Adora_A1_Mini**  、**Adora_A1_Pro**  、**Adora_A2_Mini**  、**Adora_A2_Pro** 

###   Usage 

**step1**：you should check dora  path in  “adoraa1mini_dora_bringup/CMakeLists.txt" line 47、48、60  and line 61 for mickrobot_chassis node path.   Then install dependency libraries using the following command

```
sudo apt-get install nlohmann-json3-dev
sudo apt-get install clang
```

and the ROS2 is necessary.

**step2**：unzip and build  thridpart_lib serial

```
cd adora_chassis_dora_node\thridpart_lib
unzip  serial.zip
cd serial 
mkdir build
cmake ..
sudo make install
```

**step3**： build chassis node 

```
cd adora_chassis_dora_node
mkdir build
cmake ..
make
```

**step4**： Grant permissions to the serial port

```
sudo chmod 777 /dev/ttyUSB0
```

step5： start  chassis node with  Dora 

```
dora start  adora2mini_dataflow.yml --name test
```

### show chassis logs

```
 dora logs test adoraa2mini_node
```

### Chassis receiving/publishing message 

This node receives the json string stream from **CmdVelTwist** and obtains the following data in the json string to control the chassis of the car

```
j_cmd_vel["linear"]["x"];
j_cmd_vel["linear"]["y"];
j_cmd_vel["linear"]["z"];
j_cmd_vel["angular"]["x"];
j_cmd_vel["angular"]["y"];
j_cmd_vel["angular"]["z"];
```

At the same time, the node will publish the chassis status (x speed, y speed, rotational angular velocity) at a frequency of 100Hz. The name of the published Json string data stream is "Odometry"

```
# publish Odometry Json string
# chassis position
j_odom_pub["pose"]["position"]["x"] = position_x;
j_odom_pub["pose"]["position"]["y"] = position_y;
j_odom_pub["pose"]["position"]["z"] = 0;
j_odom_pub["pose"]["orientation"]["x"] = 0;
j_odom_pub["pose"]["orientation"]["y"] = 0;
j_odom_pub["pose"]["orientation"]["z"] = 0;
j_odom_pub["pose"]["orientation"]["w"] = 1;
# chassis speed
j_odom_pub["twist"]["linear"]["x"] = linear_x;
j_odom_pub["twist"]["linear"]["y"] = linear_y;
j_odom_pub["twist"]["linear"]["z"] = 0;
j_odom_pub["twist"]["angular"]["x"] = 0;
j_odom_pub["twist"]["angular"]["y"] = 0;
j_odom_pub["twist"]["angular"]["z"] = linear_w;
```

