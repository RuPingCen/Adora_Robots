#include "dt_control.hpp"

#include <boost/array.hpp>

using namespace dt_control;

using namespace std::chrono_literals;

std::array<double, 36> cov_array = \
{1e-3, 0,    0,   0,   0,   0,
 0,    1e-3, 0,   0,   0,   0,
 0,    0,    1e6, 0,   0,   0,
 0,    0,    0,   1e6, 0,   0,
 0,    0,    0,   0,   1e6, 0,
 0,    0,    0,   0,   0,   1e3};

DtControl::DtControl(void) : Node("adora_bringup_node")
{
  declare_parameter("dt_port", "/dev/ttyUSB0");
  declare_parameter("dt_baudrate", 115200);
  declare_parameter("dt_odom_enable", true);
  declare_parameter("dt_drive_type", "SDFZ");
  declare_parameter("dt_log_display", true);
  declare_parameter("dt_original_display", false);

  param_.port       = get_parameter("dt_port").as_string();
  param_.baudrate   = get_parameter("dt_baudrate").as_int();
  param_.odom       = get_parameter("dt_odom_enable").as_bool();
  param_.drive_type = get_parameter("dt_drive_type").as_string();
  param_.log        = get_parameter("dt_log_display").as_bool();
  param_.original   = get_parameter("dt_original_display").as_bool();

  param_.sub_interval = (1 / param_.baudrate) * 10.0f;
  param_.sub_last_time = rclcpp::Clock().now();

  auto qos = rclcpp::QoS(100).transient_local();

  pub_.frame = create_publisher<Frame>("/adora_robot/chassis/frame_info", 100);

  if(param_.drive_type == "HLS")
  {
    pub_.drive_hollysys_left_error  = create_publisher<DriveHollysysError>("/adora_robot/chassis/drive_left_error_info", 100);
    pub_.drive_hollysys_right_error = create_publisher<DriveHollysysError>("/adora_robot/chassis/drive_right_error_info", 100);
  }
  else if(param_.drive_type == "SDFZ")
  {
    pub_.drive_sdfz_left_error  = create_publisher<DriveSdfzError>("/adora_robot/chassis/drive_left_error_info", 100);
    pub_.drive_sdfz_right_error = create_publisher<DriveSdfzError>("/adora_robot/chassis/drive_right_error_info", 100);
  }
  else
  {
    pub_.drive_sdfz_left_error  = create_publisher<DriveSdfzError>("/adora_robot/chassis/drive_left_error_info", 100);
    pub_.drive_sdfz_right_error = create_publisher<DriveSdfzError>("/adora_robot/chassis/drive_right_error_info", 100);
  }

  pub_.auto_charge      = create_publisher<AutoCharge>             ("/adora_robot/chassis/auto_charge_info", 100);
  pub_.battery          = create_publisher<BatteryState>           ("/adora_robot/chassis/battery_info", 100);
  pub_.date             = create_publisher<ChassisDate>            ("/adora_robot/chassis/date_info", qos);
  pub_.parameter        = create_publisher<ChassisParameter>       ("/adora_robot/chassis/parameter_info", qos);
  pub_.state            = create_publisher<ChassisState>           ("/adora_robot/chassis/state_info", 100);
  pub_.velocity         = create_publisher<ChassisVelocity>        ("/adora_robot/chassis/velocity_info", 100);
  pub_.hardware_version = create_publisher<HardwareVersion>        ("/adora_robot/chassis/hardware_version_info", qos);
  pub_.led_strip_mode   = create_publisher<std_msgs::msg::UInt8>   ("/adora_robot/chassis/led_strip_mode_info", 100);
  pub_.led_strips       = create_publisher<LedStrips>              ("/adora_robot/chassis/led_strips_info", 100);
  pub_.remote_ctrl      = create_publisher<RemoteControl>          ("/adora_robot/chassis/remote_control_info", 100);
  pub_.software_version = create_publisher<SoftwareVersion>        ("/adora_robot/chassis/software_version_info", qos);
  pub_.current          = create_publisher<TwoWheelDiffCurrent>    ("/adora_robot/chassis/current_info", 100);
  pub_.speed            = create_publisher<TwoWheelDiffSpeed>      ("/adora_robot/chassis/speed_info", 100);
  pub_.odom             = create_publisher<nav_msgs::msg::Odometry>("/adora_robot/chassis/odom_info", 100);


  sub_.velocity = create_subscription<geometry_msgs::msg::Twist>("/adora_robot/chassis/velocity_ctrl", 100,
  [this](const geometry_msgs::msg::Twist::ConstSharedPtr twist)
  {
    if(subIntervalCheck() == false) {return;}
    pkgSetVelocity(twist->linear.x, twist->angular.z);
  });
  sub_.speed = create_subscription<TwoWheelDiffSpeed>("/adora_robot/chassis/speed_ctrl", 100,
  [this](const TwoWheelDiffSpeed::ConstSharedPtr speed)
  {
    if(subIntervalCheck() == false) {return;}
    pkgSetSpeed(speed->left, speed->right);
  });
  sub_.stop_ctrl = create_subscription<std_msgs::msg::Bool>("/adora_robot/chassis/stop_ctrl", 100,
  [this](const std_msgs::msg::Bool::ConstSharedPtr enable)
  {
    if(subIntervalCheck() == false) {return;}
    pkgSetStop(enable->data);
  });
  sub_.collision_clean = create_subscription<std_msgs::msg::Empty>("/adora_robot/chassis/collision_clean", 100,
  [this](const std_msgs::msg::Empty::ConstSharedPtr)
  {
    if(subIntervalCheck() == false) {return;}
    pkgCollisionClean();
  });
  sub_.fault_clean = create_subscription<std_msgs::msg::Empty>("/adora_robot/chassis/fault_clean", 100,
  [this](const std_msgs::msg::Empty::ConstSharedPtr)
  {
    if(subIntervalCheck() == false) {return;}
    pkgFaultClean();
  });
  sub_.auto_charge_ctrl = create_subscription<std_msgs::msg::UInt8>("/adora_robot/chassis/auto_charge_ctrl", 100,
  [this](const std_msgs::msg::UInt8::ConstSharedPtr auto_charge_type)
  {
    if(subIntervalCheck() == false) {return;}
    pkgSetAutoCharge(auto_charge_type->data);
  });
  sub_.led_strip_mode = create_subscription<std_msgs::msg::UInt8>("/adora_robot/chassis/led_strip_mode_ctrl", 100,
  [this](const std_msgs::msg::UInt8::ConstSharedPtr mode)
  {
    if(subIntervalCheck() == false) {return;}
    pkgSetLedStripMode(mode->data);
  });
  sub_.led_strip = create_subscription<LedStrip>("/adora_robot/chassis/led_strip_ctrl", 100,
  [this](const LedStrip::ConstSharedPtr led_strip)
  {
    if(subIntervalCheck() == false) {return;}
    pkgSetLedStrip(*led_strip);
  });
  sub_.led_strips = create_subscription<LedStrips>("/adora_robot/chassis/led_strips_ctrl", 100,
  [this](const LedStrips::ConstSharedPtr led_strips)
  {
    if(subIntervalCheck() == false) {return;}
    pkgSetLedStrips(*led_strips);
  });
  sub_.odom_clean = create_subscription<std_msgs::msg::Empty>("/adora_robot/chassis/odom_clean", 100,
  [this](const std_msgs::msg::Empty::ConstSharedPtr)
  {
    odom_.clean();
  });


  pkgLoopInit();


  tim_.serial_init = create_wall_timer(1s, [this](void)
  {
    serialInit();
  });
  tim_.pkg_loop = create_wall_timer(std::chrono::milliseconds(pkg_.cycle), [this](void)
  {
    pkgGetLoop();
  });
  tim_.pkg_receive = create_wall_timer(1ms, [this](void)
  {
    pkgReceive();
  });

}

DtControl::~DtControl(void)
{
  if(port_.ctx)
    port_.ctx->waitForExit();
}

bool DtControl::subIntervalCheck(void)
{
  if(((rclcpp::Clock().now() - param_.sub_last_time).seconds()) < param_.sub_interval)
  {
    logs_error("Sending failed, the sending interval is too short.");
    logs_error("The data sending cycle should be greater than %.3f seconds, or less than %.3f Hz.", param_.sub_interval, 1 / param_.sub_interval);

    return false;
  }
  else
  {
    param_.sub_last_time = rclcpp::Clock().now();

    return true;
  }

  return false;
}

void DtControl::serialInit(void)
{
  if(step_.finish() == true) {return;}

  step_.create = serialCreate();
  if(step_.create == false) {serialReset(); return;}

  step_.close = serialClose();
  if(step_.close == false) {serialReset(); return;}

  step_.open = serialOpen();
  if(step_.open == false) {serialReset(); return;}
}

void DtControl::serialReset(void)
{
  if(port_.ctx) {port_.ctx->waitForExit();}
  step_.clean();
  port_.rx.clear();
  for(auto &item : port_.tx) {item.clear();}
  port_.tx.clear();
  pkgLoopReset();
}

bool DtControl::serialCreate(void)
{
  try
  {
    logs_info("serial port try create.");

    drivers::serial_driver::SerialPortConfig config(
      static_cast<uint32_t>(param_.baudrate),
      drivers::serial_driver::FlowControl::NONE,
      drivers::serial_driver::Parity::NONE,
      drivers::serial_driver::StopBits::ONE
    );

    port_.ctx = std::make_unique<drivers::common::IoContext>(1);
    port_.driver = std::make_unique<drivers::serial_driver::SerialDriver>(*port_.ctx);

    port_.driver->init_port(param_.port, config);
  }
  catch(const std::exception& e)
  {
    logs_error("serial port create fail.");

    logs_error_stream(e.what());

    return false;
  }

  logs_info("serial port create succeed.");

  return true;
}

bool DtControl::serialOpen(void)
{
  try
  {
    logs_info("serial port try open.");

    if(port_.driver->port()->is_open() == false)
      port_.driver->port()->open();

    port_.driver->port()->async_receive(std::bind(&dt_control::DtControl::serialRead, this, std::placeholders::_1, std::placeholders::_2));

  }
  catch(const std::exception& e)
  {
    logs_error("serial port open fail.");

    logs_error_stream(e.what());

    return false;
  }

  logs_info("serial port open succeed.");

  return true;
}

bool DtControl::serialClose(void)
{
  try
  {
    logs_info("serial port try close.");

    port_.driver->port()->close();
  }
  catch(const std::exception& e)
  {
    logs_error("serial port close fail.");

    logs_error_stream(e.what());

    return false;
  }

  logs_info("serial port close succeed.");

  return true;
}

void DtControl::serialRead(const std::vector<uint8_t> &array, const size_t &size)
{
  if(step_.finish() != true) {return;}

  if(size == 0) {return;}

  for(size_t loop = 0; loop < size; loop++)
    port_.rx.push_back(array.at(loop));
}

void DtControl::serialWrite(std::vector<uint8_t> &array)
{
  if(step_.finish() != true) {return;}

  if(array.size() == 0) {return;}

  try
  {
    /* logs_info("serial port try write."); */

    port_.driver->port()->send(array);
  }
  catch(const std::exception& e)
  {
    logs_error("serial port write fail.");

    logs_error_stream(e.what());

    serialReset();

    return;
  }
}

void DtControl::pkgReceive(void)
{
  if(step_.finish() != true) {return;}

  std::deque<uint8_t> &rx_buffer = port_.rx;

  while(rx_buffer.size() > 6)
  {
    if(rx_buffer.at(0) != 0xED) {rx_buffer.pop_front(); continue;}

    if(rx_buffer.at(1) != 0xDE) {rx_buffer.pop_front(); continue;}

    if(rx_buffer.size() < rx_buffer.at(2)) {return;}

    if(rx_buffer.at(2) < 6) {rx_buffer.pop_front(); continue;}

    uint16_t sum = 0;

    for(int32_t loop = 0; loop < (rx_buffer.at(2) - 2); loop++) {sum = sum + rx_buffer.at(loop);}

    if(rx_buffer.at(rx_buffer.at(2) - 2) != ((sum >> 0) & 0xFF)) {rx_buffer.pop_front(); continue;}

    if(rx_buffer.at(rx_buffer.at(2) - 1) != ((sum >> 8) & 0xFF)) {rx_buffer.pop_front(); continue;}

    if((param_.log == true) && (param_.original == true))
    {
      printf("rx frame: ");
      for(int32_t loop = 0; loop < rx_buffer.at(2); loop++)
      {
        printf("%02X ", rx_buffer.at(loop));
      }
      printf("\n");
    }

    pkgProcess();

    for(int32_t loop = 0; loop < rx_buffer.at(2); loop++)
    {
      rx_buffer.pop_front();
    }

    /* std::cout << "rx buffe size: " << rx_buffer.size() << std::endl; */
  }
}

void DtControl::pkgProcess(void)
{
  std::deque<uint8_t> &rx_buffer = port_.rx;

  switch((ReceiveCmd)rx_buffer.at(3))
  {
    case ReceiveCmd::state:
    {
      msg_.frame.cmd_80_state.clear();
      for(int32_t loop = 0; loop < rx_buffer.at(2); loop++)
      {msg_.frame.cmd_80_state.push_back(rx_buffer.at(loop));}
      pub_.frame->publish(msg_.frame);

      uint8_t ctrl_mode   = (uint8_t)rx_buffer.at(4);
      uint8_t  percent    = (uint8_t)rx_buffer.at(5);
      uint16_t voltage    = (uint16_t)((rx_buffer.at(7) << 8) | (rx_buffer.at(6) << 0));
      uint16_t state      = (uint16_t)((rx_buffer.at(9) << 8) | (rx_buffer.at(8) << 0));
      uint16_t err        = (uint16_t)((rx_buffer.at(11) << 8) | (rx_buffer.at(10) << 0));
      int16_t linear      = (int16_t)((rx_buffer.at(13) << 8) | (rx_buffer.at(12) << 0));
      int16_t angular     = (int16_t)((rx_buffer.at(15) << 8) | (rx_buffer.at(14) << 0));
      int16_t left_speed  = (int16_t)((rx_buffer.at(17) << 8) | (rx_buffer.at(16) << 0));
      int16_t right_speed = (int16_t)((rx_buffer.at(19) << 8) | (rx_buffer.at(18) << 0));

      msg_.chassis_state.control_mode          = ctrl_mode;
      msg_.battery_state.percent               = percent;
      msg_.battery_state.voltage               = ((float)voltage) * 0.1f;
      msg_.chassis_state.stop_button           = (state >> 0) & 0x01;
      msg_.chassis_state.remote_control_stop   = (state >> 1) & 0x01;
      msg_.chassis_state.software_stop         = (state >> 2) & 0x01;
      msg_.chassis_state.remote_control_online = !((state >> 3) & 0x01);
      msg_.chassis_state.front_collision       = (state >> 4) & 0x01;
      msg_.chassis_state.rear_collision        = (state >> 5) & 0x01;
      msg_.chassis_state.auto_charge_enable    = (state >> 6) & 0x01;
      msg_.chassis_state.motor_drive_online    = !((err >> 0) & 0x01);
      msg_.chassis_state.motor_drive_error     = (err >> 1) & 0x01;
      msg_.chassis_velocity.linear             = ((float)linear) * 0.001f;
      msg_.chassis_velocity.angular            = ((float)angular) * 0.001f;
      msg_.two_wheel_diff_speed.left           = left_speed;
      msg_.two_wheel_diff_speed.right          = right_speed;

      pub_.state->publish(msg_.chassis_state);
      pub_.battery->publish(msg_.battery_state);
      pub_.velocity->publish(msg_.chassis_velocity);
      pub_.speed->publish(msg_.two_wheel_diff_speed);

      pkgOdomCalculation(msg_.chassis_velocity.linear, msg_.chassis_velocity.angular);

      pkg_.state_upload.setResponse();
      pkg_.get_state.setResponse();

      pkgInfoDisplay(state, 100, "get state success.");

      break;
    }
    case ReceiveCmd::current:
    {
      msg_.frame.cmd_81_current.clear();
      for(int32_t loop = 0; loop < rx_buffer.at(2); loop++)
      {msg_.frame.cmd_81_current.push_back(rx_buffer.at(loop));}
      pub_.frame->publish(msg_.frame);

      int16_t left_current  = (int16_t)((rx_buffer.at(5) << 8) | (rx_buffer.at(4) << 0));
      int16_t right_current = (int16_t)((rx_buffer.at(7) << 8) | (rx_buffer.at(6) << 0));

      msg_.two_wheel_diff_current.left  = ((float)left_current) * 0.1f;
      msg_.two_wheel_diff_current.right = ((float)right_current) * 0.1f;

      pub_.current->publish(msg_.two_wheel_diff_current);

      pkg_.current_upload.setResponse();
      pkg_.get_current.setResponse();

      pkgInfoDisplay(current, 100, "get current success.");

      break;
    }
    case ReceiveCmd::auto_charge:
    {
      msg_.frame.cmd_82_charge.clear();
      for(int32_t loop = 0; loop < rx_buffer.at(2); loop++)
      {msg_.frame.cmd_82_charge.push_back(rx_buffer.at(loop));}
      pub_.frame->publish(msg_.frame);

      uint8_t state    = (uint8_t)rx_buffer.at(4);
      uint8_t relays   = (uint8_t)!!rx_buffer.at(5);
      uint8_t button   = (uint8_t)!!rx_buffer.at(6);
      uint8_t infrared = (uint8_t)rx_buffer.at(7);
      uint8_t voltage  = (uint8_t)rx_buffer.at(8);

      msg_.auto_charge.online       = (state == 0xFF) ? false : true;
      msg_.auto_charge.state        = (state == 0xFF) ? 0x00 : state;
      msg_.auto_charge.relays       = relays;
      msg_.auto_charge.limit_switch = button;
      msg_.auto_charge.infrared     = infrared;
      msg_.auto_charge.voltage      = voltage;

      pub_.auto_charge->publish(msg_.auto_charge);

      pkg_.auto_charge_upload.setResponse();
      pkg_.get_auto_charge.setResponse();

      pkgInfoDisplay(charge, 20, "get charge success.");

      break;
    }
    case ReceiveCmd::remote_ctrl:
    {
      msg_.frame.cmd_83_remote_control.clear();
      for(int32_t loop = 0; loop < rx_buffer.at(2); loop++)
      {msg_.frame.cmd_83_remote_control.push_back(rx_buffer.at(loop));}
      pub_.frame->publish(msg_.frame);

      int16_t right_x     = (int16_t)((rx_buffer.at(5) << 8) | (rx_buffer.at(4) << 0));
      int16_t right_y     = (int16_t)((rx_buffer.at(7) << 8) | (rx_buffer.at(6) << 0));
      int16_t left_y      = (int16_t)((rx_buffer.at(9) << 8) | (rx_buffer.at(8) << 0));
      int16_t left_x      = (int16_t)((rx_buffer.at(11) << 8) | (rx_buffer.at(10) << 0));
      int16_t left_round  = (int16_t)((rx_buffer.at(13) << 8) | (rx_buffer.at(12) << 0));
      int16_t right_round = (int16_t)((rx_buffer.at(15) << 8) | (rx_buffer.at(14) << 0));
      uint8_t key         = (uint8_t)rx_buffer.at(16);
      uint8_t state       = (uint8_t)rx_buffer.at(17);

      msg_.remote_control.rocker_right_x  = right_x;
      msg_.remote_control.rocker_right_y  = right_y;
      msg_.remote_control.rocker_left_y   = left_y;
      msg_.remote_control.rocker_left_x   = left_x;
      msg_.remote_control.round_left_vra  = left_round;
      msg_.remote_control.round_right_vrb = right_round;

      msg_.remote_control.key_swa = (key >> (0 * 2)) & B_0000_0011;
      msg_.remote_control.key_swb = (key >> (1 * 2)) & B_0000_0011;
      msg_.remote_control.key_swc = (key >> (2 * 2)) & B_0000_0011;
      msg_.remote_control.key_swd = (key >> (3 * 2)) & B_0000_0011;

      msg_.remote_control.online = !(state & 0x01);

      pub_.remote_ctrl->publish(msg_.remote_control);

      pkg_.remote_ctrl_upload.setResponse();
      pkg_.get_remote_ctrl.setResponse();

      pkgInfoDisplay(remote_ctrl, 40, "get remote ctrl success.");

      break;
    }
    case ReceiveCmd::led_strip:
    {
      msg_.frame.cmd_84_led_strip.clear();
      for(int32_t loop = 0; loop < rx_buffer.at(2); loop++)
      {msg_.frame.cmd_84_led_strip.push_back(rx_buffer.at(loop));}
      pub_.frame->publish(msg_.frame);

      msg_.led_strip_mode.data = rx_buffer.at(4);

      LedStrip led_strip;
      msg_.led_strips.leds.clear();
      for(size_t loop = 0; loop < 4; loop++)
      {
        led_strip.id         = loop + 1;
        led_strip.mode       = rx_buffer.at(5  + (loop * 6));
        led_strip.brightness = rx_buffer.at(6  + (loop * 6));
        led_strip.interval   = static_cast<float>(rx_buffer.at(7  + (loop * 6)) * 0.1f);
        led_strip.r          = rx_buffer.at(8  + (loop * 6));
        led_strip.g          = rx_buffer.at(9  + (loop * 6));
        led_strip.b          = rx_buffer.at(10 + (loop * 6));

        msg_.led_strips.leds.push_back(led_strip);
      }

      pub_.led_strip_mode->publish(msg_.led_strip_mode);
      pub_.led_strips->publish(msg_.led_strips);

      pkg_.led_strip_upload.setResponse();
      pkg_.get_led_strip.setResponse();

      pkgInfoDisplay(led_strip, 1, "get led strip success.");

      break;
    }
    case ReceiveCmd::drive_error:
    {
      msg_.frame.cmd_a0_drvie_error.clear();
      for(int32_t loop = 0; loop < rx_buffer.at(2); loop++)
      {msg_.frame.cmd_a0_drvie_error.push_back(rx_buffer.at(loop));}
      pub_.frame->publish(msg_.frame);

      uint16_t left_error  = (uint16_t)((rx_buffer.at(5) << 8) | (rx_buffer.at(4) << 0));
      uint16_t right_error = (uint16_t)((rx_buffer.at(7) << 8) | (rx_buffer.at(6) << 0));

      if(param_.drive_type == "HLS")
      {
        msg_.drive_hollysys_left_error = DriveHollysysError();
        msg_.drive_hollysys_right_error = DriveHollysysError();

        switch(left_error)
        {
          case 0x0001: {msg_.drive_hollysys_left_error.encoder_abz_fault_alarm = 1;                              break;}
          case 0x0002: {msg_.drive_hollysys_left_error.encoder_uvw_fault_alarm = 1;                              break;}
          case 0x0003: {msg_.drive_hollysys_left_error.position_error = 1;                                       break;}
          case 0x0004: {msg_.drive_hollysys_left_error.stall = 1;                                                break;}
          case 0x0005: {msg_.drive_hollysys_left_error.current_sampling_midpoint_fault = 1;                      break;}
          case 0x0006: {msg_.drive_hollysys_left_error.overload = 1;                                             break;}
          case 0x0007: {msg_.drive_hollysys_left_error.undervoltage = 1;                                         break;}
          case 0x0008: {msg_.drive_hollysys_left_error.overvoltage = 1;                                          break;}
          case 0x0009: {msg_.drive_hollysys_left_error.overcurrent = 1;                                          break;}
          case 0x000A: {msg_.drive_hollysys_left_error.discharge_alarm_instant_power_high = 1;                   break;}
          case 0x000B: {msg_.drive_hollysys_left_error.discharge_circuit_frequent_action_average_power_high = 1; break;}
          case 0x000C: {msg_.drive_hollysys_left_error.parameter_read_write_error = 1;                           break;}
          case 0x000D: {msg_.drive_hollysys_left_error.input_function_duplicate_definition = 1;                  break;}
          case 0x000E: {msg_.drive_hollysys_left_error.communication_watchdog_triggered = 1;                     break;}
          case 0x000F: {msg_.drive_hollysys_left_error.motor_overtemperature_alarm = 1;                          break;}
          default: {break;}
        }

        switch(right_error)
        {
          case 0x0001: {msg_.drive_hollysys_right_error.encoder_abz_fault_alarm = 1;                              break;}
          case 0x0002: {msg_.drive_hollysys_right_error.encoder_uvw_fault_alarm = 1;                              break;}
          case 0x0003: {msg_.drive_hollysys_right_error.position_error = 1;                                       break;}
          case 0x0004: {msg_.drive_hollysys_right_error.stall = 1;                                                break;}
          case 0x0005: {msg_.drive_hollysys_right_error.current_sampling_midpoint_fault = 1;                      break;}
          case 0x0006: {msg_.drive_hollysys_right_error.overload = 1;                                             break;}
          case 0x0007: {msg_.drive_hollysys_right_error.undervoltage = 1;                                         break;}
          case 0x0008: {msg_.drive_hollysys_right_error.overvoltage = 1;                                          break;}
          case 0x0009: {msg_.drive_hollysys_right_error.overcurrent = 1;                                          break;}
          case 0x000A: {msg_.drive_hollysys_right_error.discharge_alarm_instant_power_high = 1;                   break;}
          case 0x000B: {msg_.drive_hollysys_right_error.discharge_circuit_frequent_action_average_power_high = 1; break;}
          case 0x000C: {msg_.drive_hollysys_right_error.parameter_read_write_error = 1;                           break;}
          case 0x000D: {msg_.drive_hollysys_right_error.input_function_duplicate_definition = 1;                  break;}
          case 0x000E: {msg_.drive_hollysys_right_error.communication_watchdog_triggered = 1;                     break;}
          case 0x000F: {msg_.drive_hollysys_right_error.motor_overtemperature_alarm = 1;                          break;}
          default: {break;}
        }

        pub_.drive_hollysys_left_error->publish(msg_.drive_hollysys_left_error);
        pub_.drive_hollysys_right_error->publish(msg_.drive_hollysys_right_error);
      }

      if(param_.drive_type == "SDFZ")
      {
        msg_.drive_sdfz_left_error = DriveSdfzError();
        msg_.drive_sdfz_right_error = DriveSdfzError();

        msg_.drive_sdfz_left_error.internal               = (left_error >> 0) & 0x01;
        msg_.drive_sdfz_left_error.encoder_abz            = (left_error >> 1) & 0x01;
        msg_.drive_sdfz_left_error.encoder_uvw            = (left_error >> 2) & 0x01;
        msg_.drive_sdfz_left_error.encoder_count          = (left_error >> 3) & 0x01;
        msg_.drive_sdfz_left_error.drive_overtemp         = (left_error >> 4) & 0x01;
        msg_.drive_sdfz_left_error.drive_bus_overvoltage  = (left_error >> 5) & 0x01;
        msg_.drive_sdfz_left_error.drive_bus_undervoltage = (left_error >> 6) & 0x01;
        msg_.drive_sdfz_left_error.drive_short_circuit    = (left_error >> 7) & 0x01;
        msg_.drive_sdfz_left_error.brake_overtemp         = (left_error >> 8) & 0x01;
        msg_.drive_sdfz_left_error.actual_following       = (left_error >> 9) & 0x01;
        msg_.drive_sdfz_left_error.reserved               = (left_error >> 10) & 0x01;
        msg_.drive_sdfz_left_error.i2t                    = (left_error >> 11) & 0x01;
        msg_.drive_sdfz_left_error.speed_following        = (left_error >> 12) & 0x01;
        msg_.drive_sdfz_left_error.motor_overtemp         = (left_error >> 13) & 0x01;
        msg_.drive_sdfz_left_error.encoder_comm           = (left_error >> 14) & 0x01;
        msg_.drive_sdfz_left_error.comm_loss              = (left_error >> 15) & 0x01;

        msg_.drive_sdfz_right_error.internal               = (right_error >> 0) & 0x01;
        msg_.drive_sdfz_right_error.encoder_abz            = (right_error >> 1) & 0x01;
        msg_.drive_sdfz_right_error.encoder_uvw            = (right_error >> 2) & 0x01;
        msg_.drive_sdfz_right_error.encoder_count          = (right_error >> 3) & 0x01;
        msg_.drive_sdfz_right_error.drive_overtemp         = (right_error >> 4) & 0x01;
        msg_.drive_sdfz_right_error.drive_bus_overvoltage  = (right_error >> 5) & 0x01;
        msg_.drive_sdfz_right_error.drive_bus_undervoltage = (right_error >> 6) & 0x01;
        msg_.drive_sdfz_right_error.drive_short_circuit    = (right_error >> 7) & 0x01;
        msg_.drive_sdfz_right_error.brake_overtemp         = (right_error >> 8) & 0x01;
        msg_.drive_sdfz_right_error.actual_following       = (right_error >> 9) & 0x01;
        msg_.drive_sdfz_right_error.reserved               = (right_error >> 10) & 0x01;
        msg_.drive_sdfz_right_error.i2t                    = (right_error >> 11) & 0x01;
        msg_.drive_sdfz_right_error.speed_following        = (right_error >> 12) & 0x01;
        msg_.drive_sdfz_right_error.motor_overtemp         = (right_error >> 13) & 0x01;
        msg_.drive_sdfz_right_error.encoder_comm           = (right_error >> 14) & 0x01;
        msg_.drive_sdfz_right_error.comm_loss              = (right_error >> 15) & 0x01;

        pub_.drive_sdfz_left_error->publish(msg_.drive_sdfz_left_error);
        pub_.drive_sdfz_right_error->publish(msg_.drive_sdfz_right_error);
      }

      pkg_.get_drive_error.setResponse();

      pkgInfoDisplay(drive_error, 10, "get drive error info success.");

      break;
    }
    case ReceiveCmd::parameter:
    {
      msg_.frame.cmd_a1_parameter.clear();
      for(int32_t loop = 0; loop < rx_buffer.at(2); loop++)
      {msg_.frame.cmd_a1_parameter.push_back(rx_buffer.at(loop));}
      pub_.frame->publish(msg_.frame);

      uint16_t track_width    = (uint16_t)((rx_buffer.at(5) << 8) | (rx_buffer.at(4) << 0));
      uint16_t wheel_base     = (uint16_t)((rx_buffer.at(7) << 8) | (rx_buffer.at(6) << 0));
      uint16_t wheel_diameter = (uint16_t)((rx_buffer.at(9) << 8) | (rx_buffer.at(8) << 0));
      uint16_t gear_ratio     = (uint16_t)((rx_buffer.at(11) << 8) | (rx_buffer.at(10) << 0));
      uint16_t encoder_line   = (uint16_t)((rx_buffer.at(13) << 8) | (rx_buffer.at(12) << 0));

      msg_.chassis_parameter.track_width    = track_width;
      msg_.chassis_parameter.wheel_base     = wheel_base;
      msg_.chassis_parameter.wheel_diameter = wheel_diameter;
      msg_.chassis_parameter.gear_ratio     = gear_ratio;
      msg_.chassis_parameter.encoder_line   = encoder_line;

      pub_.parameter->publish(msg_.chassis_parameter);

      pkg_.get_parameter.setResponse();

      pkgInfoDisplay(param, 1, "get parameter success.");

      break;
    }
    case ReceiveCmd::software:
    {
      msg_.frame.cmd_a2_software_version.clear();
      for(int32_t loop = 0; loop < rx_buffer.at(2); loop++)
      {msg_.frame.cmd_a2_software_version.push_back(rx_buffer.at(loop));}
      pub_.frame->publish(msg_.frame);

      uint8_t major_version    = (uint8_t)rx_buffer.at(4);
      uint8_t minor_version    = (uint8_t)rx_buffer.at(5);
      uint8_t revision_version = (uint8_t)rx_buffer.at(6);
      uint8_t reserved_version = (uint8_t)rx_buffer.at(7);
      uint8_t year_version     = (uint8_t)rx_buffer.at(8);
      uint8_t month_version    = (uint8_t)rx_buffer.at(9);
      uint8_t day_version      = (uint8_t)rx_buffer.at(10);
      uint8_t other_version    = (uint8_t)rx_buffer.at(11);

      msg_.software_version.version.at(0) = msg_.software_version.major    = major_version;
      msg_.software_version.version.at(1) = msg_.software_version.minor    = minor_version;
      msg_.software_version.version.at(2) = msg_.software_version.revision = revision_version;
      msg_.software_version.version.at(3) = msg_.software_version.reserved = reserved_version;
      msg_.software_version.version.at(4) = msg_.software_version.year     = year_version;
      msg_.software_version.version.at(5) = msg_.software_version.month    = month_version;
      msg_.software_version.version.at(6) = msg_.software_version.day      = day_version;
      msg_.software_version.version.at(7) = msg_.software_version.other    = other_version;

      pub_.software_version->publish(msg_.software_version);

      pkg_.get_software_version.setResponse();

      pkgInfoDisplay(software, 1, "get software version success.");

      break;
    }
    case ReceiveCmd::hardware:
    {
      msg_.frame.cmd_a3_hardware_version.clear();
      for(int32_t loop = 0; loop < rx_buffer.at(2); loop++)
      {msg_.frame.cmd_a3_hardware_version.push_back(rx_buffer.at(loop));}
      pub_.frame->publish(msg_.frame);

      msg_.hardware_version.version.at(0) = (uint8_t)rx_buffer.at(4);
      msg_.hardware_version.version.at(1) = (uint8_t)rx_buffer.at(5);
      msg_.hardware_version.version.at(2) = (uint8_t)rx_buffer.at(6);
      msg_.hardware_version.version.at(3) = (uint8_t)rx_buffer.at(7);
      msg_.hardware_version.version.at(4) = (uint8_t)rx_buffer.at(8);
      msg_.hardware_version.version.at(5) = (uint8_t)rx_buffer.at(9);
      msg_.hardware_version.version.at(6) = (uint8_t)rx_buffer.at(10);
      msg_.hardware_version.version.at(7) = (uint8_t)rx_buffer.at(11);

      pub_.hardware_version->publish(msg_.hardware_version);

      pkg_.get_hardware_version.setResponse();

      pkgInfoDisplay(hardware, 1, "get hardware version success.");

      break;
    }
    case ReceiveCmd::date:
    {
      msg_.frame.cmd_a4_date.clear();
      for(int32_t loop = 0; loop < rx_buffer.at(2); loop++)
      {msg_.frame.cmd_a4_date.push_back(rx_buffer.at(loop));}
      pub_.frame->publish(msg_.frame);

      msg_.chassis_date.year  = rx_buffer.at(4);
      msg_.chassis_date.month = rx_buffer.at(5);
      msg_.chassis_date.day   = rx_buffer.at(6);

      pub_.date->publish(msg_.chassis_date);

      pkg_.get_date.setResponse();

      pkgInfoDisplay(date, 1, "get date success.");

      break;
    }
    default:
    {
      break;
    }
  }

}

void DtControl::pkgOdomCalculation(double linear, double radian)
{
  if(param_.odom == false) {return;}

  odom_.value.header.stamp = rclcpp::Clock().now();
  odom_.value.header.frame_id = "odom";
  odom_.value.child_frame_id = "base_link";

  double vx = linear;
  double vy = 0;
  double vyaw = radian;
  double dt = (rclcpp::Clock().now() - odom_.last_time).seconds();
  odom_.last_time = rclcpp::Clock().now();

  double dx = (vx * cos(odom_.yaw) - vy * sin(odom_.yaw)) * dt;
  double dy = (vx * sin(odom_.yaw) + vy * cos(odom_.yaw)) * dt;
  double dyaw = vyaw * dt;

  odom_.x += dx;
  odom_.y += dy;
  odom_.yaw += dyaw;

  tf2::Quaternion q;
  q.setRPY(0, 0, odom_.yaw);

  odom_.value.pose.pose.position.x = odom_.x;
  odom_.value.pose.pose.position.y = odom_.y;
  odom_.value.pose.pose.position.z = 0;
  odom_.value.pose.pose.orientation = tf2::toMsg(q);
  odom_.value.twist.twist.linear.x = vx;
  odom_.value.twist.twist.angular.z = vyaw;
  odom_.value.pose.covariance = cov_array;
  odom_.value.twist.covariance = cov_array;

  pub_.odom->publish(odom_.value);
}

void DtControl::pkgFormatFrame(TransmitCmd cmd, size_t data_len, uint8_t *data_buffe, std::vector<uint8_t> &tx_buffer)
{
  size_t loop = 0;
  uint16_t sum_value = 0;

  tx_buffer.clear();

  tx_buffer.push_back(0xED);
  tx_buffer.push_back(0xDE);
  tx_buffer.push_back(6 + data_len);
  tx_buffer.push_back((uint8_t)cmd);

  for(loop = 0; loop < data_len; loop++)
  {
    tx_buffer.push_back(data_buffe[loop]);
  }

  for(loop = 0; loop < tx_buffer.size(); loop++)
  {
    sum_value = sum_value + tx_buffer.at(loop);
  }

  tx_buffer.push_back((sum_value >> 0) & 0xFF);
  tx_buffer.push_back((sum_value >> 8) & 0xFF);

  #if 0
  printf("tx frame: ");
  for(loop = 0; loop < tx_buffer.size(); loop++)
  {printf("%02X ", tx_buffer.at(loop));}
  printf("\n");
  #endif
}

void DtControl::pkgSetStateUpload(bool enable)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  uint8_t *enable_ptr = nullptr;

  enable_ptr = (uint8_t *)&data_buffer[0];

  *enable_ptr = enable;

  pkgFormatFrame(TransmitCmd::state_upload, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgSetCurrentUpload(bool enable)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  uint8_t *enable_ptr = nullptr;

  enable_ptr = (uint8_t *)&data_buffer[0];

  *enable_ptr = enable;

  pkgFormatFrame(TransmitCmd::current_upload, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgSetAutoChargeUpload(bool enable)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  uint8_t *enable_ptr = nullptr;

  enable_ptr = (uint8_t *)&data_buffer[0];

  *enable_ptr = enable;

  pkgFormatFrame(TransmitCmd::auto_charge_upload, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgSetRemoteCtrlUpload(bool enable)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  uint8_t *enable_ptr = nullptr;

  enable_ptr = (uint8_t *)&data_buffer[0];

  *enable_ptr = enable;

  pkgFormatFrame(TransmitCmd::remote_ctrl_upload, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgSetLedStripUpload(bool enable)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  uint8_t *enable_ptr = nullptr;

  enable_ptr = (uint8_t *)&data_buffer[0];

  *enable_ptr = enable;

  pkgFormatFrame(TransmitCmd::led_strip_upload, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgSetVelocity(double linear, double radian)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[4] = {0};

  int16_t *linear_ptr = nullptr;
  int16_t *radian_ptr = nullptr;

  linear_ptr = (int16_t *)&data_buffer[0];
  radian_ptr = (int16_t *)&data_buffer[2];

  *linear_ptr = (int16_t)(linear * 1000);
  *radian_ptr = (int16_t)(radian * 1000);

  pkgFormatFrame(TransmitCmd::set_velocity, 4, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgSetSpeed(int16_t left_speed, int16_t right_speed)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[4] = {0};

  int16_t *left_speed_ptr = nullptr;
  int16_t *right_speed_ptr = nullptr;

  left_speed_ptr = (int16_t *)&data_buffer[0];
  right_speed_ptr = (int16_t *)&data_buffer[2];

  *left_speed_ptr = left_speed;
  *right_speed_ptr = right_speed;

  pkgFormatFrame(TransmitCmd::set_speed, 4, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgSetStop(bool enable)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  uint8_t *enable_ptr = nullptr;

  enable_ptr = (uint8_t *)&data_buffer[0];

  *enable_ptr = enable;

  pkgFormatFrame(TransmitCmd::set_stop, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgCollisionClean(void)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  uint8_t *enable_ptr = nullptr;

  enable_ptr = (uint8_t *)&data_buffer[0];

  *enable_ptr = true;

  pkgFormatFrame(TransmitCmd::collision_clean, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgFaultClean(void)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  uint8_t *enable_ptr = nullptr;

  enable_ptr = (uint8_t *)&data_buffer[0];

  *enable_ptr = true;

  pkgFormatFrame(TransmitCmd::fault_clean, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgSetAutoCharge(uint8_t charge_type)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  uint8_t *enable_ptr = nullptr;

  if((charge_type != 0x00) && (charge_type != 0x01) && (charge_type != 0x02)) {return;}

  enable_ptr = (uint8_t *)&data_buffer[0];

  *enable_ptr = charge_type;

  pkgFormatFrame(TransmitCmd::set_auto_charge, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgSetLedStripMode(uint8_t mode)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  uint8_t *enable_ptr = nullptr;

  if((mode != 0x00) && (mode != 0x01)) {return;}

  enable_ptr = (uint8_t *)&data_buffer[0];

  *enable_ptr = mode;

  pkgFormatFrame(TransmitCmd::set_led_strip_mode, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgSetLedStrip(LedStrip led_strip)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[7] = {0};

  data_buffer[0] = led_strip.id;
  data_buffer[1] = led_strip.mode;
  data_buffer[2] = led_strip.brightness;
  data_buffer[3] = static_cast<uint8_t>(led_strip.interval * 10);
  data_buffer[4] = led_strip.r;
  data_buffer[5] = led_strip.g;
  data_buffer[6] = led_strip.b;

  pkgFormatFrame(TransmitCmd::set_led_strip, 7, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgSetLedStrips(LedStrips led_strips)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[24] = {0};

  for(LedStrip &led_strip : led_strips.leds)
  {
    if((led_strip.id > 0) && (led_strip.id <= 4))
    {
      data_buffer[0 + (6 * (led_strip.id - 1))] = led_strip.mode;
      data_buffer[1 + (6 * (led_strip.id - 1))] = led_strip.brightness;
      data_buffer[2 + (6 * (led_strip.id - 1))] = static_cast<uint8_t>(led_strip.interval * 10);
      data_buffer[3 + (6 * (led_strip.id - 1))] = led_strip.r;
      data_buffer[4 + (6 * (led_strip.id - 1))] = led_strip.g;
      data_buffer[5 + (6 * (led_strip.id - 1))] = led_strip.b;
    }
  }

  pkgFormatFrame(TransmitCmd::set_led_strips, 24, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgGetState(void)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  pkgFormatFrame(TransmitCmd::get_state, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgGetCurrent(void)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  pkgFormatFrame(TransmitCmd::get_current, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgGetAutoCharge(void)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  pkgFormatFrame(TransmitCmd::get_auto_charge, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgGetRemoteCtrl(void)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  pkgFormatFrame(TransmitCmd::get_remote_ctrl, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgGetLedStrip(void)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  pkgFormatFrame(TransmitCmd::get_led_strip, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgGetDriveError(void)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  pkgFormatFrame(TransmitCmd::get_drive_error, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgGetParameter(void)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  pkgFormatFrame(TransmitCmd::get_parameter, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgGetSoftwareVersion(void)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  pkgFormatFrame(TransmitCmd::get_software, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgGetHardwareVersion(void)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  pkgFormatFrame(TransmitCmd::get_hardware, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgGetDate(void)
{
  std::vector<uint8_t> tx_buffer;
  uint8_t data_buffer[1] = {0};

  pkgFormatFrame(TransmitCmd::get_date, 1, data_buffer, tx_buffer);
  serialWrite(tx_buffer);
}

void DtControl::pkgLoopInit(void)
{
  pkg_.step = 0;

  /* get once */

  pkg_.get_software_version.setOnce(true);
  pkg_.get_software_version.setTimeout(3000);
  pkg_.get_software_version.flagClean();

  pkg_.get_hardware_version.setOnce(true);
  pkg_.get_hardware_version.setTimeout(3000);
  pkg_.get_hardware_version.flagClean();

  pkg_.get_parameter.setOnce(true);
  pkg_.get_parameter.setTimeout(3000);
  pkg_.get_parameter.flagClean();

  pkg_.get_date.setOnce(true);
  pkg_.get_date.setTimeout(3000);
  pkg_.get_date.flagClean();

  /* upload */

  pkg_.state_upload.setOnce(true);
  pkg_.state_upload.setTimeout(1000);
  pkg_.state_upload.flagClean();

  pkg_.get_current.setOnce(false);
  pkg_.get_current.setTimeout(1000);
  pkg_.get_current.flagClean();

  pkg_.auto_charge_upload.setOnce(true);
  pkg_.auto_charge_upload.setTimeout(1000);
  pkg_.auto_charge_upload.flagClean();

  pkg_.remote_ctrl_upload.setOnce(true);
  pkg_.remote_ctrl_upload.setTimeout(1000);
  pkg_.remote_ctrl_upload.flagClean();

  pkg_.led_strip_upload.setOnce(true);
  pkg_.led_strip_upload.setTimeout(1000);
  pkg_.led_strip_upload.flagClean();

  /* loop get */

  pkg_.get_drive_error.setOnce(false);
  pkg_.get_drive_error.setTimeout(100);
  pkg_.get_drive_error.flagClean();

  /* not use */

  pkg_.current_upload.setOnce(true);
  pkg_.current_upload.setTimeout(1000);
  pkg_.current_upload.flagClean();

  pkg_.get_state.setOnce(false);
  pkg_.get_state.setTimeout(1000);
  pkg_.get_state.flagClean();

  pkg_.get_auto_charge.setOnce(false);
  pkg_.get_auto_charge.setTimeout(1000);
  pkg_.get_auto_charge.flagClean();

  pkg_.get_remote_ctrl.setOnce(false);
  pkg_.get_remote_ctrl.setTimeout(1000);
  pkg_.get_remote_ctrl.flagClean();

  pkg_.get_led_strip.setOnce(false);
  pkg_.get_led_strip.setTimeout(1000);
  pkg_.get_led_strip.flagClean();

}

void DtControl::pkgLoopReset(void)
{
  pkg_.step = 0;

  pkg_.get_software_version.flagClean();
  pkg_.get_hardware_version.flagClean();
  pkg_.get_parameter.flagClean();
  pkg_.get_date.flagClean();

  pkg_.state_upload.flagClean();
  pkg_.current_upload.flagClean();
  pkg_.auto_charge_upload.flagClean();
  pkg_.remote_ctrl_upload.flagClean();
  pkg_.led_strip_upload.flagClean();

  pkg_.get_drive_error.flagClean();

  pkg_.get_state.flagClean();
  pkg_.get_current.flagClean();
  pkg_.get_auto_charge.flagClean();
  pkg_.get_remote_ctrl.flagClean();
  pkg_.get_led_strip.flagClean();
}

void DtControl::pkgGetLoop(void)
{
  if(step_.finish() != true) {return;}

  PkgGetCode code = PkgGetCode::wait;

  for(size_t step_loop = 0; step_loop < pkg_.max_num; step_loop++)
  {
    /* get once */

    if(pkg_.step == 0)
    {
      code = pkgGetCheck(pkg_.get_software_version, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetSoftwareVersion();}
      if(code == PkgGetCode::send)    {logs_info("try get software version."); break;}
      if(code == PkgGetCode::timeout) {logs_error("get software version timeout."); break;}
    }
    else if(pkg_.step == 1)
    {
      code = pkgGetCheck(pkg_.get_hardware_version, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetHardwareVersion();}
      if(code == PkgGetCode::send)    {logs_info("try get hardware version."); break;}
      if(code == PkgGetCode::timeout) {logs_error("get hardware version timeout."); break;}
    }
    else if(pkg_.step == 2)
    {
      code = pkgGetCheck(pkg_.get_parameter, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetParameter();}
      if(code == PkgGetCode::send)    {logs_info("try get parameter."); break;}
      if(code == PkgGetCode::timeout) {logs_error("get parameter timeout."); break;}
    }
    else if(pkg_.step == 3)
    {
      code = pkgGetCheck(pkg_.get_date, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetDate();}
      if(code == PkgGetCode::send)    {logs_info("try get date."); break;}
      if(code == PkgGetCode::timeout) {logs_error("get date timeout."); break;}
    }

    /* upload */

    else if(pkg_.step == 4)
    {
      code = pkgGetCheck(pkg_.state_upload, pkg_.step);
      if(code == PkgGetCode::send)    {pkgSetStateUpload(true);}
      if(code == PkgGetCode::send)    {logs_info("try open state upload."); break;}
      if(code == PkgGetCode::get)     {logs_info("open state upload success."); break;}
      if(code == PkgGetCode::timeout) {logs_error("open state upload timeout."); break;}
    }
    else if(pkg_.step == 5)
    {
      code = pkgGetCheck(pkg_.current_upload, pkg_.step);
      if(code == PkgGetCode::send)    {pkgSetCurrentUpload(true);}
      if(code == PkgGetCode::send)    {logs_info("try open current upload."); break;}
      if(code == PkgGetCode::get)     {logs_info("open current upload success."); break;}
      if(code == PkgGetCode::timeout) {logs_error("open current upload timeout."); break;}
    }
    else if(pkg_.step == 6)
    {
      code = pkgGetCheck(pkg_.auto_charge_upload, pkg_.step);
      if(code == PkgGetCode::send)    {pkgSetAutoChargeUpload(true);}
      if(code == PkgGetCode::send)    {logs_info("try open auto charge upload."); break;}
      if(code == PkgGetCode::get)     {logs_info("open auto charge upload success."); break;}
      if(code == PkgGetCode::timeout) {logs_error("open auto charge upload timeout."); break;}
    }
    else if(pkg_.step == 7)
    {
      code = pkgGetCheck(pkg_.remote_ctrl_upload, pkg_.step);
      if(code == PkgGetCode::send)    {pkgSetRemoteCtrlUpload(true);}
      if(code == PkgGetCode::send)    {logs_info("try open remote ctrl upload."); break;}
      if(code == PkgGetCode::get)     {logs_info("open remote ctrl upload success."); break;}
      if(code == PkgGetCode::timeout) {logs_error("open remote ctrl upload timeout."); break;}
    }
    else if(pkg_.step == 8)
    {
      code = pkgGetCheck(pkg_.led_strip_upload, pkg_.step);
      if(code == PkgGetCode::send)    {pkgSetLedStripUpload(true);}
      if(code == PkgGetCode::send)    {logs_info("try open led strip upload."); break;}
      if(code == PkgGetCode::get)     {logs_info("open led strip upload success."); break;}
      if(code == PkgGetCode::timeout) {logs_error("open led strip upload timeout."); break;}
    }

    /* loop get */

    else if(pkg_.step == 9)
    {
      code = pkgGetCheck(pkg_.get_drive_error, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetDriveError();}
      if(code == PkgGetCode::send)    {/* logs_info("try get drive error info."); */ break;}
      // if(code == PkgGetCode::get)     {logs_info("get drive error info success."); break;}
      if(code == PkgGetCode::timeout) {logs_error("get drive error info timeout."); break;}
    }

    /* not use */

    else if(pkg_.step == (pkg_.max_num + 1))
    {
      code = pkgGetCheck(pkg_.get_current, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetCurrent();}
      if(code == PkgGetCode::send)    {/* logs_info("try get current info."); */ break;}
      // if(code == PkgGetCode::get)     {logs_info("get current info success."); break;}
      if(code == PkgGetCode::timeout) {logs_error("get current info timeout."); break;}
    }
    else if(pkg_.step == (pkg_.max_num + 1))
    {
      code = pkgGetCheck(pkg_.get_auto_charge, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetAutoCharge();}
      if(code == PkgGetCode::send)    {/* logs_info("try get auto charge info."); */ break;}
      // if(code == PkgGetCode::get)     {logs_info("get auto charge info success."); break;}
      if(code == PkgGetCode::timeout) {logs_error("get auto charge info timeout."); break;}
    }
    else if(pkg_.step == (pkg_.max_num + 1))
    {
      code = pkgGetCheck(pkg_.get_remote_ctrl, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetRemoteCtrl();}
      if(code == PkgGetCode::send)    {/* logs_info("try get remote ctrl info."); */ break;}
      // if(code == PkgGetCode::get)     {logs_info("get remote ctrl info success."); break;}
      if(code == PkgGetCode::timeout) {logs_error("get remote ctrl info timeout."); break;}
    }
    else if(pkg_.step == (pkg_.max_num + 1))
    {
      code = pkgGetCheck(pkg_.get_led_strip, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetLedStrip();}
      if(code == PkgGetCode::send)    {/* logs_info("try get led strip info."); */ break;}
      // if(code == PkgGetCode::get)     {logs_info("get led strip info success."); break;}
      if(code == PkgGetCode::timeout) {logs_error("get led strip info timeout."); break;}
    }

    if(code == PkgGetCode::get) {break;}
    if(code == PkgGetCode::wait) {break;}
    if(code == PkgGetCode::skip) {continue;}
  }

  if(pkg_.step == pkg_.max_num)
  {
    pkg_.step = 0;
  }
}

PkgGetCode DtControl::pkgGetCheck(PkgGet &pkg, uint32_t &step)
{
  if((pkg.do_once == true) && (pkg.flag.once == true))
  {
    step++;
    return PkgGetCode::skip;
  }

  if(pkg.flag.request == false)
  {
    pkg.flag.request = true;
    pkg.flag.response = false;
    pkg.flag.timeout_cnt = 0;
    return PkgGetCode::send;
  }

  if(pkg.flag.request == true)
  {
    pkg.flag.timeout_cnt++;

    if(pkg.flag.response == true)
    {
      step++;
      pkg.flag.once = true;
      pkg.flag.request = false;
      pkg.flag.response = false;
      pkg.flag.timeout_cnt = 0;
      return PkgGetCode::get;
    }

    if(pkg.flag.timeout_cnt > ((pkg.timeout_ms < pkg_.cycle) ? 1 : (pkg.timeout_ms / pkg_.cycle)))
    {
      step++;
      pkg.flag.request = false;
      pkg.flag.response = false;
      pkg.flag.timeout_cnt = 0;
      return PkgGetCode::timeout;
    }

    if(pkg.flag.response == false)
    {
      return PkgGetCode::wait;
    }
  }

  return PkgGetCode::wait;
}
