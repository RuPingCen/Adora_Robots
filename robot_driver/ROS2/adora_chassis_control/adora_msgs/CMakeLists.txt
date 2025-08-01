cmake_minimum_required(VERSION 3.8)
project(adora_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
 
# find dependencies
 
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 添加生成消息的依赖
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control1.msg"
  "msg/Dt1.msg"
  "msg/Dt2.msg"
  "msg/Error.msg"
  "msg/ChassisState.msg"
  "msg/BatteryState.msg"
  "msg/ChassisVelocity.msg"
  "msg/FourWheelSteerAngle.msg"
  "msg/FourWheelSteerEncoder.msg"
  "msg/FourWheelSteerMotion.msg"
  "msg/TwoWheelDiffSpeed.msg"
  "msg/FourWheelDiffSpeed.msg"
  "msg/FourWheelSteerSpeed.msg"
  "msg/AckermannSpeed.msg"
  "msg/TwoWheelDiffCurrent.msg"
  "msg/FourWheelDiffCurrent.msg"
  "msg/FourWheelSteerCurrent.msg"
  "msg/AckermannCurrent.msg"
  "msg/AutoCharge.msg"
  "msg/RemoteControl.msg"
  "msg/ChassisParameter.msg"
  "msg/SoftwareVersion.msg"
  "msg/HardwareVersion.msg"
  "msg/LedStrip.msg"
  "msg/LedStrips.msg"
  "msg/ChassisDate.msg"
  "msg/DriveHollysysError.msg"
  "msg/DriveSdfzError.msg"
  "msg/Frame.msg"
  DEPENDENCIES std_msgs  # 如果使用了 time/duration 等类型
)
ament_export_dependencies(rosidl_default_runtime) 
ament_package()
