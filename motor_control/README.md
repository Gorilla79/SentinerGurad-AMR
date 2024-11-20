cd my_workspace
colcon build --packages-select motor_control
source install/setup.bash 
ros2 run motor_control motor_control_node.py [or] ros2 run motor_control motor_control_joy 

[new terminal]
 ros2 run teleop_twinst_keyboard teleop_twist_keyboard [or] ros2 run joy joy_node 

[joy controller check]
ros2 topic list -> /joy

ros2 topic echo /joy

[
header:
  stamp:
    sec: 1732128207
    nanosec: 396487759
  frame_id: joy
axes:
- -0.0
- -0.0
- 1.0
- -0.0
- -0.0
- 1.0
- -0.0
- -0.0
buttons:
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
---
]
