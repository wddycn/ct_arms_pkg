#include "eyou_ros2_control/eyou_system.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <iostream>
#include <cstdio>

namespace eyou_ros2_control
{
EyouSystem::EyouSystem()
: can_left_("can1"), can_right_("can2")
{}

hardware_interface::CallbackReturn EyouSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::vector<std::string> expected_joint_order = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
    "joint7", "joint8", "joint9", "joint10", "joint11", "joint12"
  };
  if (info.joints.size() != expected_joint_order.size())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("eyou_hw"), "Expected 12 joints, got %zu", info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_num_ = info.joints.size();

  joint_names_.resize(joint_num_);
  joint_pos_state_.resize(joint_num_, 0.0);
  joint_pos_cmd_.resize(joint_num_, 0.0);
  joint_vel_state_.resize(joint_num_, 0.0);
  joint_eff_state_.resize(joint_num_, 0.0);

  for (size_t i = 0; i < joint_num_; ++i)
  {
    joint_names_[i] = info.joints[i].name;
    if (joint_names_[i] != expected_joint_order[i])
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("eyou_hw"),
        "Joint %zu must be '%s', got '%s'",
        i, expected_joint_order[i].c_str(), joint_names_[i].c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("eyou_hw"), "Register joint: %s", joint_names_[i].c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Humble 导出状态接口
std::vector<hardware_interface::StateInterface> EyouSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for(size_t i = 0; i < joint_num_; i++)
  {
    // 位置
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_names_[i], "position", &joint_pos_state_[i])
    );
    // 新增速度
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_names_[i], "velocity", &joint_vel_state_[i])
    );
    // 新增力矩/电流
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_names_[i], "effort", &joint_eff_state_[i])
    );
  }
  return state_interfaces;
}

// Humble 导出命令接口
std::vector<hardware_interface::CommandInterface> EyouSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmd_interfaces;
  for(size_t i = 0; i < joint_num_; i++)
  {
    cmd_interfaces.emplace_back(
      hardware_interface::CommandInterface(joint_names_[i], "position", &joint_pos_cmd_[i])
    );
  }
  return cmd_interfaces;
}

hardware_interface::CallbackReturn EyouSystem::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(rclcpp::get_logger("eyou_hw"), "Activate hardware, init CAN bus");
  if(!connect_motor_bus())
  {
    RCLCPP_ERROR(rclcpp::get_logger("eyou_hw"), "CAN or motor enable failed");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!read_all_motor_states(joint_pos_state_, joint_vel_state_, joint_eff_state_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("eyou_hw"), "Failed to read initial motor states");
    disconnect_motor_bus();
    return hardware_interface::CallbackReturn::ERROR;
  }
  joint_pos_cmd_ = joint_pos_state_;
  has_sent_pos_cmd_ = false;

  RCLCPP_INFO(rclcpp::get_logger("eyou_hw"), "All 12 motors online");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EyouSystem::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(rclcpp::get_logger("eyou_hw"), "Deactivate hardware, shutdown CAN");
  disconnect_motor_bus();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EyouSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  if (!read_all_motor_states(joint_pos_state_, joint_vel_state_, joint_eff_state_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("eyou_hw"), "Failed to read one or more motor states");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EyouSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  if (!send_all_motor_targets(joint_pos_cmd_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("eyou_hw"), "Failed to send one or more motor targets");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool EyouSystem::connect_motor_bus()
{
  bool left_ok = can_left_.openSocket();
  if (!left_ok)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("eyou_hw"),
      "Cannot open can1. Configure and bring up the CAN interface before launching.");
  }
  for(uint8_t mid : left_motor_ids_)
  {
    if(!can_left_.enableMotor(mid))
    {
      RCLCPP_ERROR(rclcpp::get_logger("eyou_hw"), "Left motor id %d enable fail", mid);
      left_ok = false;
    }
  }
  left_ready_ = left_ok;

  bool right_ok = can_right_.openSocket();
  if (!right_ok)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("eyou_hw"),
      "Cannot open can2. Configure and bring up the CAN interface before launching.");
  }
  for(uint8_t mid : right_motor_ids_)
  {
    if(!can_right_.enableMotor(mid))
    {
      RCLCPP_ERROR(rclcpp::get_logger("eyou_hw"), "Right motor id %d enable fail", mid);
      right_ok = false;
    }
  }
  right_ready_ = right_ok;

  return left_ok && right_ok;
}

void EyouSystem::disconnect_motor_bus()
{
  for(uint8_t mid : left_motor_ids_)
    can_left_.disableMotor(mid);
  can_left_.closeSocket();

  for(uint8_t mid : right_motor_ids_)
    can_right_.disableMotor(mid);
  can_right_.closeSocket();
}

bool EyouSystem::read_all_motor_states(std::vector<double>& pos_rad,
                                       std::vector<double>& vel_rad,
                                       std::vector<double>& eff)
{
  bool all_ok = true;
  // 左臂 joint1~joint6
  for(size_t i=0; i<6; i++)
  {
    uint8_t mid = left_motor_ids_[i];
    const double position = can_left_.readPosition(mid);
    const double velocity = can_left_.readSpeed(mid);
    const double torque = can_left_.readTorque(mid);

    if (std::isfinite(position)) pos_rad[i] = position; else all_ok = false;
    if (std::isfinite(velocity)) vel_rad[i] = velocity; else all_ok = false;
    if (std::isfinite(torque)) eff[i] = torque; else all_ok = false;
  }
  // 右臂 joint7~joint12
  for(size_t i=0; i<6; i++)
  {
    uint8_t mid = right_motor_ids_[i];
    const double position = can_right_.readPosition(mid);
    const double velocity = can_right_.readSpeed(mid);
    const double torque = can_right_.readTorque(mid);

    if (std::isfinite(position)) pos_rad[6 + i] = position; else all_ok = false;
    if (std::isfinite(velocity)) vel_rad[6 + i] = velocity; else all_ok = false;
    if (std::isfinite(torque)) eff[6 + i] = torque; else all_ok = false;
  }
  return all_ok;
}

bool EyouSystem::send_all_motor_targets(const std::vector<double>& cmd_rad)
{
  if (!left_ready_ || !right_ready_ || cmd_rad.size() != joint_num_)
  {
    return false;
  }
  for (const double command : cmd_rad)
  {
    if (!std::isfinite(command)) return false;
  }
  if (has_sent_pos_cmd_ && cmd_rad == last_sent_pos_cmd_)
  {
    return true;
  }

  std::vector<double> spd = {0.5,0.5,0.5,0.5,0.5,0.5};
  std::vector<float> trq = {5.0f,5.0f,5.0f,5.0f,2.0f,2.0f};
  std::vector<double> acc = {8.0,8.0,8.0,8.0,8.0,8.0};
  std::vector<double> dec = {8.0,8.0,8.0,8.0,8.0,8.0};

  std::vector<double> left_target(cmd_rad.begin(), cmd_rad.begin()+6);
  const bool left_ok =
    can_left_.setProfilePositionMulti(left_motor_ids_, left_target, spd, trq, acc, dec);

  std::vector<double> right_target(cmd_rad.begin()+6, cmd_rad.end());
  const bool right_ok =
    can_right_.setProfilePositionMulti(right_motor_ids_, right_target, spd, trq, acc, dec);
  if (left_ok && right_ok)
  {
    last_sent_pos_cmd_ = cmd_rad;
    has_sent_pos_cmd_ = true;
    return true;
  }
  return false;
}

}  // namespace eyou_ros2_control

PLUGINLIB_EXPORT_CLASS(
  eyou_ros2_control::EyouSystem,
  hardware_interface::SystemInterface
)
