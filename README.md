# ROS2-Gazebo Marine Simulator v1.2

ROS2-Gazebo Marine Simulator (currently under development) aims to be a comprehensive marine simulation environment utilizing ROS2 (Robot Operating System 2) and Gazebo. 

marsim currently consists of these packages:
- bluerov2_ctrl_serv (planned action servers for BlueROV2 control)
- marsim_control
- marsim_gazebo


# MARSIM1.2.pdf

Read the Marsim1.2.pdf file for information and details on the simulator. The pdf covers the fundamental components of the simulation stack, including Docker containerization, ROS2 architecture, and Gazebo simulation capabilities.


# Citation
Xing, Y. (2024). northxing/marsim. GitHub. https://github.com/northxing/marsim 


# Licensing

The ROS2/Gazebo Marine Simulator is licensed under the Apache License 2.0.


# Project License

Copyright 2024 Yihan Xing

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.

You may obtain a copy of the License at:
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


# Third-Party Licenses

The original Marine Simulator Gazebo (marsim_gazebo) package, especially the BlueROV2 part, was extensively modified from and built on clydemcqueen/bluerov2_ignition, which is licensed under the MIT liense.

Copyright 2023 Clyde McQueen
License: MIT
https://github.com/clydemcqueen/bluerov2_ignition/tree/main


# Contributing:

Contributions to this project are welcome. By submitting a contribution, you agree to license your work under the same Apache License 2.0.


# Example commands:

Launch the empty world:
ros2 launch marsim_gazebo empty_world.launch.py

Spawn a BlueROV2 robot:
ros2 launch marsim_gazebo spawn_bluerov2.launch.py

Start the ROS2-GZ bridge:
ros2 launch marsim_gazebo start_bridge_bluerov2.launch.py

Start the keyboard controller:
ros2 run marsim_gazebo ros_keyboard_controller bluerov2

Start the PID depth control:
ros2 run marsim_control bluerov2_pid_pose_z_control.py --ros-args -p model_name:=my_rov -p Kp:=15.0 -p Ki:=0.02 -p Kd:=0.5 -p desired_z:=-30.0


