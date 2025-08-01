**如果在使用过程中遇到了问题，可以结合另一个文档autonomous_manual.pdf来看看**

#### MID-360雷达驱动

##### 1. 安装雷达SDK

```c
$ git clone https://github.com/Livox-SDK/Livox-SDK2.git
$ cd ./Livox-SDK2/
$ mkdir build
$ cd build
$ cmake .. && make -j
$ sudo make install
```

##### 2. 下载雷达驱动包

```c
mkdir -p livox_driver/src && cd livox_driver/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
source /opt/ros/noetic/setup.sh
./build.sh ROS1
```

##### 3. 配置orin的IP

![](pic/1.png)

##### 4. 查看雷达的IP地址

由于每个雷达的IP地址不同，因此要先测一下，这里采用wireshark进行测试。如果没有安装，可以用下面命令安装：

```c
sudo apt install wireshark-qt
```

运行wireshark：

```c
sudo wireshark
```

如图：

![](pic/2.png)

如果只连了一根网线的话，就默认是eth0，然后双击这里

![](pic/3.png)

红色框里面对应的就是MID-360的IP地址

##### 5. 修改MID-360的配置

输入下面指令打开配置文件：

```c
gedit livox_driver/src/livox_ros_driver2/config/MID360_config.json
```

![](pic/4.png)

需要修改以上5个地方，前4个就改为图中这样就行，这就是上面orin配置的IP，最后的IP就设置为自己所使用的雷达IP即可

##### 6. 测试MID-360

在配置完雷达后，可以通过运行launch文件进行测试

```c
cd livox_drive/
source devel/setup.bash
roslaunch livox_ros_driver2 rviz_MID360.launch
```

在rviz界面能看到雷达点云就说明配置好了

#### DORA导航部分的使用

##### 1. SLAM建图

###### 1.1 安装hdl_graph_slam

```c
sudo apt-get install ros-noetic-geodesy ros-noetic-pcl-ros ros-noetic-nmea-msgs
ros-noetic-libg2o
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git
git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive
git clone https://github.com/koide3/hdl_graph_slam
cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release
```

###### 1.2 修改launch文件配置

```c
gedit catkin_ws/src/hdl_graph_slam/launch/hdl_graph_slam_501.launch 
```

![](pic/5.png)

将这两个地方改为图中这样即可

###### 1.3 录制点云bag包

先启动激光雷达

```c
cd livox_ros_driver2
source devel/setup.bash
roslaunch livox_ros_driver2 rviz_MID360.launch
```

然后ctrl+shift+t打开一个新终端，录制点云话题

```c
source ./devel/setup.bash
rosbag record -o livoxbag.bag /livox/lidar
```

接着就可以绕着地图走一圈，在录完之后就可以杀掉这两个终端

###### 1.4 开始建图

先启动roscore

```c
roscore
```

启动建图节点

```c
cd catkin_ws
source devel/setup.bash
rosparam set use_sim_time true
roslaunch hdl_graph_slam hdl_graph_slam_501.launch
```

播放雷达点云包，找到刚才录制的bag包

```c
rosbag play --clock livoxbag.bag # 这里bag包的名字替换为自己录制的
```

然后在hdl目录下可以运行对应配置好的rviz，就可以看到建图的效果

```c
source devel/setup.bash
rviz -d src/hdl_graph_slam/rviz/hdl_graph_slam.rviz
```

在点云包播放完了之后，同样在hdl目录下运行以下命令用于保存建好的地图

```c
source ./devel/setup.bash
rosservice call /hdl_graph_slam/save_map "utm: false
resolution: 0.01
destination: 'yourpath/map.pcd'" #路径可自定义
```

通过pcl_viewer可以查看建好的地图

```c
pcl_viewer yourmap.pcd
```

###### 1.5 处理地图

由于小车底盘坐标轴是旋转了90°的，但是建图的时候使用的是激光雷达的坐标轴，所以还需要对地图进行旋转处理

map_process.cpp

```c
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

int main(int argc, char** argv) {
    // 检查输入参数
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_pcd_file> <output_pcd_file>" << std::endl;
        return -1;
    }

    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        std::cerr << "Error: Could not read file " << argv[1] << std::endl;
        return -1;
    }
    std::cout << "Loaded " << cloud->size() << " points from " << argv[1] << std::endl;

    // 创建变换矩阵（绕Z轴逆时针旋转90度）
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) =  0;  // cos(90°) = 0
    transform(0, 1) = -1;  // -sin(90°) = -1
    transform(1, 0) =  1;  // sin(90°) = 1
    transform(1, 1) =  0;  // cos(90°) = 0
    // Z轴和位移保持不变

    // 应用变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    // 保存结果
    if (pcl::io::savePCDFileBinary(argv[2], *transformed_cloud) == -1) {
        std::cerr << "Error: Could not save to " << argv[2] << std::endl;
        return -1;
    }
    std::cout << "Saved " << transformed_cloud->size() << " points to " << argv[2] << std::endl;

    return 0;
}
```

CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.10)
project(map_process)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
)

add_executable(map_process map_process.cpp)

target_link_libraries(map_process
    ${PCL_LIBRARIES}
)
```

然后编译运行

```c
mkdir build && cd build
cmake ..
make
./map_process yourmap.pcd map.pcd ## 这里改为自己的map名字
```

##### 2. 录制路线

首先先对dora工程进行编译

```c
mkdir build && cd build
cmake ..
make -j12
```

然后将地图放入data文件夹中，名字修改为map.pcd

然后运行录制路线的脚本

```c
dora up
dora start load_path.yml
```

按着预定路线走一圈之后， 杀掉终端。 路线点记录在/data/path/trajectory.txt中。但是这个时候可能记录的点会非常密集，可以通过一个python文件来进行间隔点采样，获得点距合适的路径点

```python
import matplotlib.pyplot as plt

def plot_waypoints(file_path, step=20, save_path='trajectory_sparse.txt'):
    x_coords = []
    y_coords = []
    
    with open(file_path, 'r') as f:
        for i, line in enumerate(f):
            if i % step != 0:
                continue
            x, y = map(float, line.strip().split())
            x_coords.append(x)
            y_coords.append(y)
    
    # 新增保存稀疏点的代码
    with open(save_path, 'w') as f:
        for x, y in zip(x_coords, y_coords):
            f.write(f"{x:.3f} {y:.3f}\n")  # 保留3位小数
    
    plt.figure(figsize=(10, 6))
    plt.plot(x_coords, y_coords, 'b-', linewidth=2)  # 蓝色实线连接路径点
    plt.plot(x_coords, y_coords, 'ro')  # 红色圆点标记路径点
    plt.xlabel('X Coordinate (m)')
    plt.ylabel('Y Coordinate (m)')
    plt.title('Waypoints Visualization')
    plt.grid(True)
    plt.axis('equal')  # 保持坐标轴等比例
    plt.show()

if __name__ == "__main__":
    plot_waypoints('data/path/trajectory.txt', 
              step=30,
              save_path='Waypoints.txt')
```

这里的step就是步长，可以通过不断调整来获得最佳的路径点，同时这里的Waypoints.txt就是最后导航所需要的路径点文件

##### 3. 运行导航系统

```c
sudo chmod 777 /dev/ttyUSB0
dora up
dora start run.yml
```

Rerun会自动弹出， 白色的线为全局路线图， 蓝色的为规划线

此时小车可能并不会直接启动起来，因为在planning/mission_planning/task_pub/main.cpp文件中，需要接收到一个本地回环的UDP消息"start"后才会开始动起来，这里给一个python程序用于模拟这样的通信：

```python
import socket
import threading

class NavigationClient:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setblocking(False)
        
        # 根据工程协议配置端口
        self.ports = {
            "task_exc": 33335,  # 执行节点反馈端口
            "task_pub": 33334,  # 任务节点指令端口
            "control": 33333    # 公共控制端口
        }
        
        # 启动接收线程
        self.receiver_thread = threading.Thread(target=self._receiver)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()

    def _receiver(self):
        """监听执行节点的反馈消息"""
        self.sock.bind(('0.0.0.0', self.ports["task_exc"]))
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                print(f"[UDP反馈] {data.decode()} from {addr}")
            except BlockingIOError:
                pass

    def send_command(self, cmd_type, message):
        """发送控制指令"""
        target_port = self.ports["control"] if cmd_type == "control" else self.ports[cmd_type]
        self.sock.sendto(message.encode(), ('127.0.0.1', target_port))
        print(f"已发送指令 [{message}] 到端口 {target_port}")

# 使用示例
if __name__ == "__main__":
    client = NavigationClient()
    # 
    # 发送任务启动指令到task_pub节点
    client.send_command("task_pub", "start")
    # client.send_command("task_exc", "finish")
```

如果在小车动起来后，发现走了一段路之后突然停下来，这是因为在planning/mission_planning/task_exc/main.cpp文件中，当小车的frenet坐标系下的s大于设置的值后，需要接收到UDP的"finish"消息才会继续移动，这里可以通过运行上面的python文件中的第二个指令来模拟。如果觉得在调试阶段麻烦，可以将文件中的Back_Car_pthread()函数里面的内容注释掉，这样就不会中途停下来。

#### 对整体代码框架的说明

按照run.yml中的节点和数据输入输出流来进行说明

##### 1. lidar

雷达节点，驱动MID-360激光雷达，发布雷达点云数据

##### 2. hdl_localization

定位节点，订阅雷达点云数据进行定位，发布小车x、y、theta数据

##### 3. pub_road

发布录制的路径，读取Waypoints.txt文件里面的坐标并发布

##### 4. road_lane_publisher_node

订阅路径和小车位姿数据，然后将小车位姿从笛卡尔坐标系(x、y、theta)转化为frenet坐标系(s、d)

对frenet坐标系的说明：s代表的是将路径的每个点连接起来后拉成一条直线后小车所在这条线的位置，d代表的是小车所在路径段的偏移量。

##### 5. task_pub_node

该节点用于解析road_msg.txt文件，最主要的是发布小车的前进速度数据

##### 6. task_exc_node

该节点主要是通过UDP通信来实现小车中途停和走的操作

##### 7. planning

从road_lane话题中得到路径的每个点自身的x、y、s数据

从road_attri_msg话题得到前进的速度

从SetStop_service话题判断当前的前进还是停止

从cur_pose_all话题中得到当前frenet坐标，然后根据全局路径的frenet坐标得到一条规划的frenet坐标系下的路径，这条路径就是实际上小车所要走的路径，接着再将这条路径的从frenet坐标系重新转换为笛卡尔坐标系，最后将地图坐标系下的路径规划点转化为小车坐标系下的坐标点，把这条路径给发布出去

##### 8. lon_control

这个节点是订阅planning节点的数据后用于设置小车最终前进的速度

##### 9. latcontrol

订阅规划好的路径，然后通过纯跟踪算法来计算出转角速度

##### 10. control

该节点主要是订阅前进速度数据和转角速度数据，实现最终的小车运动控制。
