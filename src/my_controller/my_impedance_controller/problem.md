# Problem 1:
```bash
CMake Error at my_controller/my_impedance_controller/CMakeLists.txt:61 (add_dependencies):
  The dependency target "my_impedance_controller_gencpp" of target
  "my_impedance_controller" does not exist.
```
## 解决办法：`cmake`中：
```bash
add_dependencies(my_impedance_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  # ${PROJECT_NAME}_generate_messages_cpp
  # ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)
```

# Problem 2
```bash
[ERROR] [1732055802.575054778, 0.026000000]: Could not load controller 'my_impedance_controller' because the type was not specified. Did you load the controller configuration on the parameter server (namespace: '/my_impedance_controller')?
```
### 解决办法：
```bash
  <rosparam command="load" file="$(find my_impedance_controller)/config/my_impedance_controller.yaml" subst_value="true" />
  ```
文件名错误

# Problem 3:
```bash
[FATAL] [1732056133.202917217]: Failed to create robot simulation interface loader: Could not find library corresponding to plugin franka_gazebo/FrankaHWSim. Make sure the plugin description XML file has the correct name of the library and that the library actually exists.
```
删除工作空间，重新编译，是franka_gazebo出问题了

# problem 4:
```bash
ERROR: cannot launch node of type [my_impedance_controller/interactive_marker.py]: Cannot locate node of type [interactive_marker.py] in package [my_impedance_controller]. Make sure file exists in package path and permission is set to executable (chmod +x)
```
缺少`scripts`文件下的`interactive_marker.py`