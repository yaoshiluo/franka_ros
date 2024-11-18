#ifndef MY_FRANKA_CONTROLLER_MY_CONTROLLER_H
#define MY_FRANKA_CONTROLLER_MY_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>

namespace my_franka_controller {

class MyController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  
private:
  std::vector<hardware_interface::JointHandle> joint_handles_;
};

}  // namespace my_franka_controller

#endif  // MY_FRANKA_CONTROLLER_MY_CONTROLLER_H
