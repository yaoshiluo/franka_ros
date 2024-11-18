# 解释`include`中的头文件

## `pragma once`
防止头文件被多次包含

## `<controller_interface/multi_interface_controller.h>`
多接口控制器可以同时访问多个硬件接口（例如关节位置接口和模型接口），适用于复杂控制场景

## `<dynamic_reconfigure/server.h>`
支持在运行时调整参数（例如阻抗参数），以适应不同控制需求

## `<geometry_msgs/PoseStamped.h>`
用于表示笛卡尔空间中的位姿（位置 + 方向），包含时间戳和参考坐标系

## `<hardware_interface/joint_command_interface.h>`
- 提供关节命令接口，例如关节位置、速度或力矩控制
- 控制器通过该接口向硬件发送命令

## `<hardware_interface/robot_hw.h>`
- 提供机器人硬件的抽象接口
- 定义硬件层的抽象类，用于与实际机器人硬件交互

## `<franka_hw/franka_model_interface.h>`
- 获取机器人动力学信息（例如惯性矩阵、科氏力矩、重力向量）
- 常用于实现动力学补偿或高级控制算法

## `<franka_hw/franka_state_interface.h>`
- 获取当前机器人的状态（例如关节角度、速度、力矩）
- 常用于状态反馈控制