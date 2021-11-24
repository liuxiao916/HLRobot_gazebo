# HLRobot_gazebo
A simulation for QKM HL6-0900 6DOF robot based on gazebo

## Install
```bash
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control ros-melodic-hector-gazebo-plugins

cd catkin_ws/src
git clone https://github.com/liuxiao916/HLRobot_gazebo.git
cd catkin_ws/
catkin_make
```


## Launch
```bash
roslaunch hlrobot_gazebo gazebo.launch
```

## Control
In order to verify the forward and inverse kinematics, I use `position_controllers/JointGroupPositionController` for controller and `PositionJointInterface` for each joint. It means for every transmission the input is position and output is also position.

You can simply use this command to specify the angle (radian) of every joint.
```bash
rostopic pub /HL_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [1.1, 2.1, 1.2, 2.3, 1.2, 3.1]" 

```

## Todo
- [ ] load angle of joints from txt file to control the robot
- [ ] Implement the forward and inverse kinematics in this simulation
- [ ] Implement Motion Planning (Linear Function with Parabolic Blends) 
- [ ] Simply control the robot by giving Cartesian coordinate
- [ ] Anything useful 



