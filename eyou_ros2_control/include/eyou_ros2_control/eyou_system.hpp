#ifndef EYOU_ROS2_CONTROL__EYOU_SYSTEM_HPP_
#define EYOU_ROS2_CONTROL__EYOU_SYSTEM_HPP_

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>
#include "eyou.h"

namespace eyou_ros2_control
{
class EyouSystem : public hardware_interface::SystemInterface
{
public:
  // 构造函数声明
  EyouSystem();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Humble 必须实现的两个纯虚接口
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  size_t joint_num_;
  std::vector<std::string> joint_names_;

  // 原有
  std::vector<double> joint_pos_state_;
  std::vector<double> joint_pos_cmd_;
  // 新增速度、力矩存储
  std::vector<double> joint_vel_state_;
  std::vector<double> joint_eff_state_;


  MotorCAN can_left_;
  MotorCAN can_right_;

  const std::vector<uint8_t> left_motor_ids_ = {2,3,4,5,6,7};
  const std::vector<uint8_t> right_motor_ids_ = {8,9,10,11,12,13};

  bool left_ready_ = false;
  bool right_ready_ = false;

  bool connect_motor_bus();
  void disconnect_motor_bus();
  void read_all_motor_pos(std::vector<double>& pos_rad,
                          std::vector<double>& vel_rad,
                          std::vector<double>& eff_rad);
  void send_all_motor_target(const std::vector<double>& cmd_rad);

  inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
  inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }
};
}  // namespace eyou_ros2_control

#endif  // EYOU_ROS2_CONTROL__EYOU_SYSTEM_HPP_
