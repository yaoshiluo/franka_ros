# 启动示例控制器`cartesian_impedance_example_controller`
```bash
roslaunch franka_gazebo panda.launch controller:=cartesian_impedance_example_controller
```
## 由`rosservice`施加外力
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
## 检查控制器是否已经加载
```bash
rosservice call /controller_manager/list_controllers
```
# 自己写一个`cartesian_impedance_example_controller`

## 1. `launch`文件的关系

### 1.1 `panda.launch`：`franka_ros_ws/src/franka_ros/franka_gazebo/launch`
- 启动 Gazebo 世界，并加载 Panda 机械臂
- 支持动态设置位置、姿态和关节角度
- 使用 `franka_gazebo/robot.launch` 加载 Panda 的 URDF 模型
- 根据用户指定的 controller 参数加载相应的控制器 
#### 使用方法：指定参数
```bash
roslaunch franka_gazebo panda_gazebo.launch controller:=cartesian_impedance_example_controller
```
### 1.2 `robot.launch` : `franka_ros_ws/src/franka_ros/franka_gazebo/launch`
- 加载指定的 Gazebo 世界文件
- 动态生成 Panda 的 URDF 模型，并加载到仿真环境中
- 支持动态调整 Panda 的初始位置、姿态、以及是否加载夹爪
- 启动 Gazebo 的 `controller_manager`，加载并初始化所需的控制器
- 使用 `robot_state_publisher` 和 `joint_state_publisher` 发布 Panda 的关节状态和 `TF` 变换
#### 加载URDF配置
- `<param name="robot_description"
       command="$(find xacro)/xacro $(find franka_description)/robots/$(arg robot)/$(arg robot).urdf.xacro"/>
`

## 2. URDF配置文件

### 2.1. `panda.urdf.xacro`: `franka_ros_ws/src/franka_ros/franka_description/robots/panda`:

- 包含 `common/franka_robot.xacro`，这是 Panda 机械臂的基础模型定义
- 使用 `<xacro:arg>` 定义了一系列可配置参数，允许用户通过命令行或上层 launch 文件动态调整模型
- 提供 gazebo 和 parent 参数，用于指定 Panda 在 Gazebo 中的父级框架和初始位置
- 设置机械臂的初始位置和姿态（`xyz` 和 `rpy`）

### 2.2. `franka_robot.xacro`: `franka_ros_ws/src/franka_ros/franka_description/robots/common`:

- 动态加载 Panda 机械臂模型
- 当参数 `gazebo=true` 时，会添加 Gazebo 相关的关节接口和插件
- 支持与 Gazebo 的 `ros_control` 插件交互，仿真硬件接口
- 提供多种接口（位置、速度、力矩）支持 Gazebo 和硬件控制

## 3. `YAML`文件

```bash
cartesian_impedance_example_controller:
    type: franka_example_controllers/CartesianImpedanceExampleController
    arm_id: $(arg arm_id)
    joint_names:
        - $(arg arm_id)_joint1
        - $(arg arm_id)_joint2
        - $(arg arm_id)_joint3
        - $(arg arm_id)_joint4
        - $(arg arm_id)_joint5
        - $(arg arm_id)_joint6
        - $(arg arm_id)_joint7
```
### 3.1 定义控制器类型
- type：指定使用的控制器插件，例如 `franka_example_controllers/CartesianImpedanceExampleController`
- 这是 ROS 控制器架构中的必要配置，ROS 会通过 `pluginlib` 动态加载此插件
### 3.2 绑定控制的机械臂
- `arm_id`：绑定控制器到特定的机械臂实例
### 3.3 指定控制的关节
- `joint_names`：列出此控制器需要控制的关节
- 每个关节名以机械臂标识符`（arm_id）`作为前缀，例如 `panda_joint1`

## 4. plugin.xml文件
```bash
<library path="libfranka_example_controllers">
  <class name="franka_example_controllers/CartesianImpedanceExampleController"
         type="franka_example_controllers::CartesianImpedanceExampleController"
         base_class_type="controller_interface::ControllerBase">
    <description>
      A controller that renders a spring damper system in cartesian space. Compliance parameters and the equilibrium pose can be modified online with dynamic reconfigure and an interactive marker respectively.
    </description>
  </class>
</library>

```
### 4.1 定义一个 ROS 控制器插件类，使其能够被 ROS 的插件库（`pluginlib`）动态加载
```bash
<class name="franka_example_controllers/CartesianImpedanceExampleController" type="franka_example_controllers::CartesianImpedanceExampleController" base_class_type="controller_interface::ControllerBase">
```
#### 4.1.1 `name`： 定义插件的名称，通常是 <包名>/<插件名> 格式
- 在 `YAML` 配置文件中，`type` 字段会引用这个名称，例如：
```bash
cartesian_impedance_example_controller:
  type: franka_example_controllers/CartesianImpedanceExampleController
```
#### 4.1.2 `type`： 
- 指定插件类的 C++ 实现名称
- 这里是 `franka_example_controllers::CartesianImpedanceExampleController`

#### 4.1.3 `base_class_type`:
- 指定插件继承的基类
- 该控制器基于 `controller_interface::ControllerBase`，这表明它是一个标准的 ROS 控制器

## 5. 控制器运行的完整流程

1. **启动控制器：Launch 文件**
   - 启动控制器管理器和控制器。
   - 加载 YAML 配置文件，完成控制器的参数初始化。

2. **加载配置：YAML 文件**
   - 指定控制器类型（`type`），并绑定机械臂和关节。
   - 提供控制器运行所需的关节名称、实例名称等。

3. **注册插件：plugin.xml 文件**
   - 注册 C++ 控制器类，使其可以被 `controller_manager` 动态加载。

4. **控制逻辑实现：C++ 文件**
   - 定义控制器的核心逻辑，包括：
     - 初始化硬件接口。
     - 在实时循环中计算控制命令（如力矩）。
   - 实现关键功能，如笛卡尔空间的阻抗控制。

5. **加载机械臂模型：URDF/Xacro**
   - 提供控制器运行所需的运动学和动力学模型。
   - 如果在 Gazebo 仿真中运行，还加载相关插件支持。



