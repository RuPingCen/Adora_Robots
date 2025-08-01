#ifndef DT_CONTROL_HPP
#define DT_CONTROL_HPP

#include <cstdio>
#include <memory>
#include <chrono>
#include <vector>
#include <deque>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "serial_driver/serial_driver.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"

#include "binary.h"

#include "adora_msgs/msg/drive_hollysys_error.hpp"
#include "adora_msgs/msg/drive_sdfz_error.hpp"
#include "adora_msgs/msg/frame.hpp"

#include "adora_msgs/msg/ackermann_current.hpp"
#include "adora_msgs/msg/ackermann_speed.hpp"
#include "adora_msgs/msg/auto_charge.hpp"
#include "adora_msgs/msg/battery_state.hpp"
#include "adora_msgs/msg/chassis_date.hpp"
#include "adora_msgs/msg/chassis_parameter.hpp"
#include "adora_msgs/msg/chassis_state.hpp"
#include "adora_msgs/msg/chassis_velocity.hpp"
#include "adora_msgs/msg/four_wheel_diff_current.hpp"
#include "adora_msgs/msg/four_wheel_diff_speed.hpp"
#include "adora_msgs/msg/four_wheel_steer_angle.hpp"
#include "adora_msgs/msg/four_wheel_steer_current.hpp"
#include "adora_msgs/msg/four_wheel_steer_encoder.hpp"
#include "adora_msgs/msg/four_wheel_steer_motion.hpp"
#include "adora_msgs/msg/four_wheel_steer_speed.hpp"
#include "adora_msgs/msg/hardware_version.hpp"
#include "adora_msgs/msg/led_strip.hpp"
#include "adora_msgs/msg/led_strips.hpp"
#include "adora_msgs/msg/remote_control.hpp"
#include "adora_msgs/msg/software_version.hpp"
#include "adora_msgs/msg/two_wheel_diff_current.hpp"
#include "adora_msgs/msg/two_wheel_diff_speed.hpp"

namespace dt_control
{

  using DriveHollysysError = adora_msgs::msg::DriveHollysysError;
  using DriveSdfzError = adora_msgs::msg::DriveSdfzError;
  using Frame = adora_msgs::msg::Frame;

  using AckermannCurrent = adora_msgs::msg::AckermannCurrent;
  using AckermannSpeed = adora_msgs::msg::AckermannSpeed;
  using AutoCharge = adora_msgs::msg::AutoCharge;
  using BatteryState = adora_msgs::msg::BatteryState;
  using ChassisDate = adora_msgs::msg::ChassisDate;
  using ChassisParameter = adora_msgs::msg::ChassisParameter;
  using ChassisState = adora_msgs::msg::ChassisState;
  using ChassisVelocity = adora_msgs::msg::ChassisVelocity;
  using FourWheelDiffCurrent = adora_msgs::msg::FourWheelDiffCurrent;
  using FourWheelDiffSpeed = adora_msgs::msg::FourWheelDiffSpeed;
  using FourWheelSteerAngle = adora_msgs::msg::FourWheelSteerAngle;
  using FourWheelSteerCurrent = adora_msgs::msg::FourWheelSteerCurrent;
  using FourWheelSteerEncoder = adora_msgs::msg::FourWheelSteerEncoder;
  using FourWheelSteerMotion = adora_msgs::msg::FourWheelSteerMotion;
  using FourWheelSteerSpeed = adora_msgs::msg::FourWheelSteerSpeed;
  using HardwareVersion = adora_msgs::msg::HardwareVersion;
  using LedStrip = adora_msgs::msg::LedStrip;
  using LedStrips = adora_msgs::msg::LedStrips;
  using RemoteControl = adora_msgs::msg::RemoteControl;
  using SoftwareVersion = adora_msgs::msg::SoftwareVersion;
  using TwoWheelDiffCurrent = adora_msgs::msg::TwoWheelDiffCurrent;
  using TwoWheelDiffSpeed = adora_msgs::msg::TwoWheelDiffSpeed;

#define logs_debug(...)                \
  do                                   \
  {                                    \
    if (param_.log)                    \
    {                                  \
      printf("\033[33m[DT] [DEBUG] "); \
      printf(__VA_ARGS__);             \
      printf("\033[0m\n");             \
      fflush(stdout);                  \
    }                                  \
  } while (0)
#define logs_info(...)        \
  do                          \
  {                           \
    if (param_.log)           \
    {                         \
      printf("[DT] [INFO] "); \
      printf(__VA_ARGS__);    \
      printf("\n");           \
      fflush(stdout);         \
    }                         \
  } while (0)
#define logs_error(...)                \
  do                                   \
  {                                    \
    if (param_.log)                    \
    {                                  \
      printf("\033[31m[DT] [ERROR] "); \
      printf(__VA_ARGS__);             \
      printf("\033[0m\n");             \
      fflush(stdout);                  \
    }                                  \
  } while (0)

#define logs_debug_stream(args)                                                                       \
  do                                                                                                  \
  {                                                                                                   \
    if (param_.log)                                                                                   \
    {                                                                                                 \
      std::cout << "\033[33m" << "[DT]" << " " << "[DEBUG]" << " " << args << "\033[0m" << std::endl; \
      std::cout.flush();                                                                              \
    }                                                                                                 \
  } while (0)
#define logs_info_stream(args)                                            \
  do                                                                      \
  {                                                                       \
    if (param_.log)                                                       \
    {                                                                     \
      std::cout << "[DT]" << " " << "[INFO]" << " " << args << std::endl; \
      std::cout.flush();                                                  \
    }                                                                     \
  } while (0)
#define logs_error_stream(args)                                                                       \
  do                                                                                                  \
  {                                                                                                   \
    if (param_.log)                                                                                   \
    {                                                                                                 \
      std::cout << "\033[31m" << "[DT]" << " " << "[ERROR]" << " " << args << "\033[0m" << std::endl; \
      std::cout.flush();                                                                              \
    }                                                                                                 \
  } while (0)

#define pkgInfoDisplay(name, max_cnt, str) \
  do                                       \
  {                                        \
    static uint8_t name##_cnt = 0;         \
                                           \
    name##_cnt++;                          \
                                           \
    if (name##_cnt == max_cnt)             \
    {                                      \
      name##_cnt = 0;                      \
      logs_info(str);                      \
    }                                      \
  } while (0)

  enum class TransmitCmd
  {
    state_upload = 0x01,
    current_upload = 0x02,
    auto_charge_upload = 0x03,
    remote_ctrl_upload = 0x04,
    led_strip_upload = 0x05,

    set_velocity = 0x20,
    set_speed = 0x21,
    set_stop = 0x22,
    collision_clean = 0x23,
    fault_clean = 0x24,
    set_auto_charge = 0x25,
    set_led_strip_mode = 0x26,
    set_led_strip = 0x27,
    set_led_strips = 0x28,

    get_state = 0x40,
    get_current = 0x41,
    get_auto_charge = 0x42,
    get_remote_ctrl = 0x43,
    get_drive_error = 0x44,
    get_parameter = 0x45,
    get_software = 0x46,
    get_hardware = 0x47,
    get_date = 0x48,
    get_led_strip = 0x49,
  };

  enum class ReceiveCmd
  {
    state = 0x80,
    current = 0x81,
    auto_charge = 0x82,
    remote_ctrl = 0x83,
    led_strip = 0x84,

    drive_error = 0xA0,
    parameter = 0xA1,
    software = 0xA2,
    hardware = 0xA3,
    date = 0xA4,
  };

  enum class PkgGetCode
  {
    send,
    get,
    wait,
    skip,
    timeout,
  };

  struct Param
  {
    std::string port;
    int baudrate = 0;

    bool odom = true;
    std::string drive_type = "SDFZ";

    bool log = true;
    bool original = false;

    double sub_interval;
    rclcpp::Time sub_last_time;
  };

  struct Pub
  {
    rclcpp::Publisher<Frame>::SharedPtr frame;
    rclcpp::Publisher<DriveHollysysError>::SharedPtr drive_hollysys_left_error;
    rclcpp::Publisher<DriveHollysysError>::SharedPtr drive_hollysys_right_error;
    rclcpp::Publisher<DriveSdfzError>::SharedPtr drive_sdfz_left_error;
    rclcpp::Publisher<DriveSdfzError>::SharedPtr drive_sdfz_right_error;

    rclcpp::Publisher<AutoCharge>::SharedPtr auto_charge;
    rclcpp::Publisher<BatteryState>::SharedPtr battery;
    rclcpp::Publisher<ChassisDate>::SharedPtr date;
    rclcpp::Publisher<ChassisParameter>::SharedPtr parameter;
    rclcpp::Publisher<ChassisState>::SharedPtr state;
    rclcpp::Publisher<ChassisVelocity>::SharedPtr velocity;
    rclcpp::Publisher<HardwareVersion>::SharedPtr hardware_version;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr led_strip_mode;
    rclcpp::Publisher<LedStrips>::SharedPtr led_strips;
    rclcpp::Publisher<RemoteControl>::SharedPtr remote_ctrl;
    rclcpp::Publisher<SoftwareVersion>::SharedPtr software_version;
    rclcpp::Publisher<TwoWheelDiffCurrent>::SharedPtr current;
    rclcpp::Publisher<TwoWheelDiffSpeed>::SharedPtr speed;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
  };

  struct Sub
  {
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity;
    rclcpp::Subscription<TwoWheelDiffSpeed>::SharedPtr speed;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_ctrl;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr collision_clean;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr fault_clean;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr auto_charge_ctrl;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr led_strip_mode;
    rclcpp::Subscription<LedStrip>::SharedPtr led_strip;
    rclcpp::Subscription<LedStrips>::SharedPtr led_strips;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr odom_clean;
  };

  struct Tim
  {
    rclcpp::TimerBase::SharedPtr serial_init;
    rclcpp::TimerBase::SharedPtr pkg_loop;
    rclcpp::TimerBase::SharedPtr pkg_receive;
  };

  struct Msg
  {
    Frame frame;
    DriveHollysysError drive_hollysys_left_error;
    DriveHollysysError drive_hollysys_right_error;
    DriveSdfzError drive_sdfz_left_error;
    DriveSdfzError drive_sdfz_right_error;

    AckermannCurrent ackermann_current;
    AckermannSpeed ackermann_speed;
    AutoCharge auto_charge;
    BatteryState battery_state;
    ChassisDate chassis_date;
    ChassisParameter chassis_parameter;
    ChassisState chassis_state;
    ChassisVelocity chassis_velocity;
    FourWheelDiffCurrent four_wheel_diff_current;
    FourWheelDiffSpeed four_wheel_diff_speed;
    FourWheelSteerAngle four_wheel_steer_angle;
    FourWheelSteerCurrent four_wheel_steer_current;
    FourWheelSteerEncoder four_wheel_steer_encoder;
    FourWheelSteerMotion four_wheel_steer_motion;
    FourWheelSteerSpeed four_wheel_steer_speed;
    HardwareVersion hardware_version;
    std_msgs::msg::UInt8 led_strip_mode;
    LedStrips led_strips;
    RemoteControl remote_control;
    SoftwareVersion software_version;
    TwoWheelDiffCurrent two_wheel_diff_current;
    TwoWheelDiffSpeed two_wheel_diff_speed;
  };

  struct SerialStep
  {
    bool create = false;
    bool close = false;
    bool open = false;

    void clean(void)
    {
      create = false;
      close = false;
      open = false;
    }

    bool finish(void)
    {
      return ((create && close && open) == true) ? true : false;
    }
  };

  struct SerialPort
  {
    std::unique_ptr<drivers::common::IoContext> ctx{};
    std::unique_ptr<drivers::serial_driver::SerialDriver> driver;

    std::deque<uint8_t> rx;
    std::deque<std::vector<uint8_t>> tx;
  };

  struct Odom
  {
    double x = 0;
    double y = 0;
    double yaw = 0;
    rclcpp::Time last_time;
    nav_msgs::msg::Odometry value;

    void clean(void)
    {
      x = 0;
      y = 0;
      yaw = 0;
      value.pose.pose.position.x = x;
      value.pose.pose.position.y = y;
      last_time = rclcpp::Clock().now();
    }
  };

  struct PkgFlag
  {
    bool once = false;

    bool request = false;
    bool response = false;

    uint32_t timeout_cnt = 0;
  };

  struct PkgGet
  {
    bool do_once = false;
    uint32_t timeout_ms = 0;

    PkgFlag flag;

    void setOnce(bool set) { do_once = set; }
    void setTimeout(uint32_t ms) { timeout_ms = ms; }
    void flagClean(void) { flag = PkgFlag(); }

    void setResponse(void)
    {
      if (flag.request == true)
      {
        flag.response = true;
      }
    }
  };

  struct Pkg
  {
    uint32_t cycle = 100;
    uint32_t step = 0;

    const size_t max_num = 10;

    /* get once */

    PkgGet get_software_version;
    PkgGet get_hardware_version;
    PkgGet get_parameter;
    PkgGet get_date;

    /* upload */

    PkgGet state_upload;
    PkgGet auto_charge_upload;
    PkgGet remote_ctrl_upload;
    PkgGet led_strip_upload;

    /* loop get */

    PkgGet get_drive_error;

    /* not use */

    PkgGet current_upload;

    PkgGet get_state;
    PkgGet get_current;
    PkgGet get_auto_charge;
    PkgGet get_remote_ctrl;
    PkgGet get_led_strip;
  };

  class DtControl : public rclcpp::Node
  {

  public:
    DtControl(void);
    ~DtControl(void);

  private:
    Param param_;
    Pub pub_;
    Sub sub_;
    Tim tim_;
    Msg msg_;
    SerialStep step_;
    SerialPort port_;
    Odom odom_;
    Pkg pkg_;

    bool subIntervalCheck(void);

    void serialInit(void);
    void serialReset(void);
    bool serialCreate(void);
    bool serialOpen(void);
    bool serialClose(void);
    void serialRead(const std::vector<uint8_t> &array, const size_t &size);
    void serialWrite(std::vector<uint8_t> &array);

    void pkgReceive(void);
    void pkgProcess(void);

    void pkgOdomCalculation(double linear, double radian);
    void pkgFormatFrame(TransmitCmd cmd, size_t data_len, uint8_t *data_buffe, std::vector<uint8_t> &tx_buffer);

    void pkgSetStateUpload(bool enable);
    void pkgSetCurrentUpload(bool enable);
    void pkgSetAutoChargeUpload(bool enable);
    void pkgSetRemoteCtrlUpload(bool enable);
    void pkgSetLedStripUpload(bool enable);

    void pkgSetVelocity(double linear, double radian);
    void pkgSetSpeed(int16_t left_speed, int16_t right_speed);
    void pkgSetStop(bool enable);
    void pkgCollisionClean(void);
    void pkgFaultClean(void);
    void pkgSetAutoCharge(uint8_t charge_type);
    void pkgSetLedStripMode(uint8_t mode);
    void pkgSetLedStrip(LedStrip led_strip);
    void pkgSetLedStrips(LedStrips led_strips);

    void pkgGetState(void);
    void pkgGetCurrent(void);
    void pkgGetAutoCharge(void);
    void pkgGetRemoteCtrl(void);
    void pkgGetLedStrip(void);
    void pkgGetDriveError(void);

    void pkgGetParameter(void);
    void pkgGetSoftwareVersion(void);
    void pkgGetHardwareVersion(void);
    void pkgGetDate(void);

    void pkgLoopInit(void);
    void pkgLoopReset(void);
    void pkgGetLoop(void);
    PkgGetCode pkgGetCheck(PkgGet &pkg, uint32_t &step);
  };

}

#endif
