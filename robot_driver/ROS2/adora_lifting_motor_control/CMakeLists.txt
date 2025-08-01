cmake_minimum_required(VERSION 3.8)
project(adora_lifting_motor_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


set(CMAKE_INSTALL_RPATH /usr/local/lib)
find_package(serial REQUIRED)
find_package(rclcpp QUIET)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

 
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

 



add_executable(adora_lifting_motor_control_node src/serialliftingmotor.cpp)

ament_target_dependencies(adora_lifting_motor_control_node 
rclcpp 
std_msgs
sensor_msgs
tf2
nav_msgs
tf2_geometry_msgs
serial
)

install(TARGETS adora_lifting_motor_control_node
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

#install(DIRECTORY include
#DESTINATION share/${PROJECT_NAME}
#)
 
ament_package()
