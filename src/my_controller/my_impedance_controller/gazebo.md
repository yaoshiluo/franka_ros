# 1 配置 FrankaHWSim 插件
要让你的机器人能够支持 Franka 特定接口，你需要在机器人描述文件 (URDF) 中声明一个自定义的 `robotSimType`
```bash
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>${arm_id}</robotNamespace>
    <controlPeriod>0.001</controlPeriod>
    <robotSimType>franka_gazebo/FrankaHWSim</robotSimType>
  </plugin>
  <self_collide>true</self_collide>
</gazebo>
```
`<robotSimType>franka_gazebo/FrankaHWSim </robotSimType>`:声明使用 `franka_gazebo/FrankaHWSim` 模拟类型，这是关键配置，使 `Gazebo` 支持 `Franka` 的自定义硬件接口
## 1. 加载 FrankaHWSim
- `gazebo_ros_control` 插件会根据 robotSimType 的值，加载并初始化对应的硬件仿真插件
- `FrankaHWSim` 作为一个 C++ 类，实现了标准的硬件接口方法：
  - 初始化接口：`init()` 方法从 `URDF` 文件中读取关节信息，并注册 `ROS` 控制器所需的硬件接口
  - 状态更新：`read()` 方法从 `Gazebo` 获取关节状态（如位置、速度、力矩等），并同步到硬件接口
  - 命令更新：`write()` 方法将控制器的命令写入 `Gazebo` 模拟环境
## 2. 通过 ROS 控制器框架的硬件接口注册机制
- ROS 控制器框架（`ros_control`）的核心机制是 硬件接口（`Hardware Interface`），它为控制器和硬件（或仿真）之间的交互提供了统一的标准。硬件仿真插件通过以下步骤注册硬件接口，使得控制器能够识别和使用这些接口：
- 每种控制模式（如位置控制、速度控制、力矩控制）都有对应的硬件接口类，例如：
  - `EffortJointInterface`：用于力矩控制
  - `PositionJointInterface`：用于位置控制
  - `VelocityJointInterface`：用于速度控制
### 2.1具体流程：
1. 创建硬件接口对象：
```bash
hardware_interface::EffortJointInterface effort_joint_interface_;
```
2. 将关节句柄绑定到硬件接口：
   - 对于每个关节，创建一个关节句柄（`JointHandle`），并将其添加到对应的硬件接口中
```bash
hardware_interface::JointHandle joint_handle(joint_state_interface_.getHandle("joint_name"), &effort_command_);
effort_joint_interface_.registerHandle(joint_handle);
```
这里，`joint_state_interface_` 提供关节的状态（位置、速度、力矩等），`effort_command_` 是控制器要施加的力矩命令

# `cartesian_impedance_example_controller`
## 通过`std::vector<hardware_interface::JointHandle>`定义句柄 `joint_handles_`
1. 获取关节状态
   - 每个句柄可以访问关节的状态，例如位置（Position）、速度（`Velocity`）和力矩（`Effort`）
```bash
double position = joint_handles_[i].getPosition();  // 获取位置
double velocity = joint_handles_[i].getVelocity();  // 获取速度
double effort = joint_handles_[i].getEffort();      // 获取力矩
```
2. 设置控制命令
   - 每个句柄可以向对应的关节发送控制命令
```bash
joint_handles_[i].setCommand(target_position);
joint_handles_[i].setCommand(target_velocity);
joint_handles_[i].setCommand(target_torque);
```
## Gazebo 力矩指令传递流程

在 Gazebo 仿真中，力矩指令从控制器到物理引擎的传递过程如下：

### **流程图**
```plaintext
Gazebo 物理引擎 ➡ gazebo_ros_control ➡ EffortJointInterface ➡ JointHandle
```
## `Hardware Interface`
硬件接口是 ROS 控制框架中的一个核心概念，它连接机器人控制器和硬件（或仿真环境）。通过硬件接口，控制器可以向机器人硬件发送控制指令（如力矩、速度、位置），并从硬件获取关节状态（如位置、速度、力矩）

## `<franka_hw::FrankaModelInterface>`
是 `Franka Control Interface`（Franka 控制接口）的一部分，专门用于访问 `Franka Panda` 机械臂的动态模型。它提供了一个硬件接口，用于计算机械臂的动力学特性，包括雅可比矩阵、惯性矩阵、科里奥利力矩等。

这个接口可以在控制器中通过 `ROS` 控制框架注册和使用，通常用于高级控制算法（如阻抗控制、轨迹规划或力控制），以实时访问 `Panda` 机械臂的动力学模型

## `void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);`
- 实例应用：
```bash
void CartesianImpedanceExampleController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}
```
- `msg`消息由外部通过`equilibrium_pose`话题直接发布，将值赋给`position_d_target_`,由于`position_d_target_`由外部输入，所以变化会太过剧烈，所以经过滤波后让`position_d_ = position_d_target_`

## `geometry_msgs::WrenchStamped`
```bash
# Represents force in free space, separated into
# its linear and angular parts.
Vector3 force
Vector3 torque
```
在`gazebo`中可以通过`rosservice`施加外力，有两种使用方法：
1. 使用`cartesian_controller`不断发布新的`Pose`,产生力矩，机械臂移动，在移动过程中，通过`rosservice`施加外力，查看机械臂的柔顺性。
具体例子：
```bash
rosservice call /gazebo/apply_body_wrench "body_name: 'panda_link7'
reference_frame: 'world'
wrench:
  force:
    x: 10.0
    y: 0.0
    z: 0.0
  torque:
    x: 0.0
    y: 0.0
    z: 0.0
duration:
  secs: 2
  nsecs: 0"
```
2. 如果需要施加力和力矩来控制机械臂的关节，可以使用 `Gazebo` 的关节控制器`Joint Controllers`
   - 使用 `ROS` 控制器（如 `effort_controllers/JointEffortController`）控制力矩

**问题**：输入外力wrench的作用是什么，如果是为了让机械臂往指定的位置移动，通过`pose`控制就可以，如果直接输入让机械臂移动的外力，那就不需要`pose`输入了。或者这个外力是模拟的接触到物体表面的力，那这样就可以表现出柔顺性。