#include "my_franka_controller/my_controller.h"
#include <pluginlib/class_list_macros.h>
#include <cmath>

namespace my_franka_controller {

bool MyController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) {
  std::string joint_name;
  for (int i = 1; i <= 7; ++i) {
    joint_name = "panda_joint" + std::to_string(i);
    joint_handles_.push_back(hw->getHandle(joint_name));
  }
  return true;
}

void MyController::update(const ros::Time& time, const ros::Duration& /*period*/) {
  double effort_amplitude = 1.0;  // 力矩幅值，可根据需要调整
  double frequency = 0.5;         // 正弦波频率（单位：Hz）

  // 为每个关节施加一个随时间变化的正弦波力矩
  for (size_t i = 0; i < joint_handles_.size(); ++i) {
    double effort = effort_amplitude * std::sin(2 * M_PI * frequency * time.toSec());
    joint_handles_[i].setCommand(effort);  // 为关节设置力矩命令
  }
}

}  // namespace my_franka_controller

PLUGINLIB_EXPORT_CLASS(my_franka_controller::MyController, controller_interface::ControllerBase)
