cmake_minimum_required(VERSION 3.8)
project(adora_chassis_bringup)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_INSTALL_RPATH /usr/local/lib)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp QUIET)
find_package(rclpy QUIET)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(adora_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(serial_driver REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
 
#find_package(rosidl_typesupport_cpp REQUIRED)
include_directories(
  include
  #${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp  # 自动生成头文件路径
  ${ament_INCLUDE_DIRS}  # 自动包含所有ament依赖路径
)



# 适用于Adora-A1-Mini   Adora-A2-Pro
add_executable(adora_chassis_bringup_node src/adora_chassis_bringup_V2.cpp)

target_link_libraries(adora_chassis_bringup_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  serial
  adora_msgs::adora_msgs__rosidl_generator_c
  adora_msgs::adora_msgs__rosidl_typesupport_cpp
  # 显式链接类型支持库
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
)

ament_target_dependencies(adora_chassis_bringup_node 
  rclcpp 
  std_msgs
  sensor_msgs
  tf2
  nav_msgs
  tf2_geometry_msgs
  serial
  adora_msgs
  #${PROJECT_NAME}  # 关键链接自身消息
)

# 适用于Adora_a2_max_node
add_executable(adora_a2_max_node  src/adora_a2_max_ros2_node.cpp
                                  src/adora_a2_max_control.cpp)
target_link_libraries(adora_a2_max_node   
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES})
ament_target_dependencies(adora_a2_max_node
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  std_msgs
  adora_msgs
  serial_driver)

#rosidl_get_typesupport_target(cpp_typesupport_target
#  ${PROJECT_NAME} rosidl_typesupport_cpp)



install(TARGETS adora_chassis_bringup_node adora_a2_max_node
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include
DESTINATION share/${PROJECT_NAME}
)

ament_package()
