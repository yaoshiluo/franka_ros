# 创建自定义 ROS 控制器 - 教程

## 1. 创建控制器包

首先，创建一个 ROS 包，并添加必要的依赖项。
```bash
catkin_create_pkg my_franka_controller controller_interface hardware_interface pluginlib roscpp
```

## 2. 目录结构
```bash
my_franka_controller/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── controllers.yaml          # 控制器的配置文件
├── include/
│   └── my_franka_controller/
│       └── my_controller.h       # 控制器头文件
├── launch/
│   └── my_controller.launch      # 启动文件
├── src/
│   └── my_controller.cpp         # 控制器实现文件
└── plugin.xml                    # 插件描述文件
```

# 自定义 ROS 控制器插件的完整配置指南

本指南介绍了如何使用 `pluginlib` 正确导出控制器类，并在 ROS 中加载您的自定义控制器。

## 1. 实现控制器类并导出

在自定义控制器的 `.cpp` 文件中，定义控制器类的实现并通过 `pluginlib` 导出该类，使其能够作为 ROS 插件被识别和加载。确保在 `.cpp` 文件的末尾添加 `PLUGINLIB_EXPORT_CLASS` 宏。

示例代码：

```cpp
#include <pluginlib/class_list_macros.h>
#include "name_of_your_controller_package/NameOfYourControllerClass.h"

namespace name_of_your_controller_package {

bool NameOfYourControllerClass::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) {
  // 控制器的初始化代码
  return true;
}

void NameOfYourControllerClass::update(const ros::Time& time, const ros::Duration& period) {
  // 控制器的更新逻辑
}

}  // namespace name_of_your_controller_package

// 使用 pluginlib 导出控制器类
PLUGINLIB_EXPORT_CLASS(name_of_your_controller_package::NameOfYourControllerClass, controller_interface::ControllerBase)
```

## 2. 编写 `plugin.xml` 文件

```bash
<?xml version="1.0"?>
<library path="lib/lib<name_of_your_controller_library>">
  <class name="name_of_your_controller_package/NameOfYourControllerClass"
         type="name_of_your_controller_package::NameOfYourControllerClass"
         base_class_type="controller_interface::ControllerBase">
    <description>
      Some text to describe what your controller is doing
    </description>
  </class>
</library>
```
## 3. 在 `package.xml` 中导出插件

```bash
<export>
  <controller_interface plugin="${prefix}/plugin.xml"/>
</export>
```

## 4. 配置控制器的 `YAML` 文件

```bash
your_custom_controller_name:
  type: name_of_your_controller_package/NameOfYourControllerClass
  additional_example_parameter: 0.0
  # 可以添加更多控制器参数
```

## 5. 启动文件示例

```bash
<launch>
  <!-- 加载控制器的配置参数 -->
  <rosparam file="$(find name_of_your_controller_package)/config/controllers.yaml" command="load"/>

  <!-- 启动控制器管理器并加载自定义控制器 -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen"
        args="your_custom_controller_name"/>
</launch>
```

## 6. 编译并运行

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch my_franka_controller panda.launch
```
