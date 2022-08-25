# HLRobot_gazebo
A simulation for QKM HL6-0900 6DOF robot based on gazebo
![robot](hlrobot_gazebo/data/picture/robot_model.png)

## Demo
[Demo Link](https://www.bilibili.com/video/BV1qL4y1p7Gr?p=4)

## Install
```bash
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control ros-melodic-hector-gazebo-plugins 
sudo apt-get install ros-melodic-sound-play 
sudo apt-get install ros-melodic-joint-trajectory-controller
sudo apt-get install ros-melodic-moveit*
pip install scipy
cd catkin_ws/src
git clone https://github.com/liuxiao916/HLRobot_gazebo.git
cd catkin_ws/
catkin_make
```

## Launch
Just launch a robot.
```bash
roslaunch hlrobot_gazebo hl_gazebo.launch
```
Launch robot to play music.
```bash
roslaunch hlrobot_gazebo bringup_music.launch
```
Launch robot to plan with moveit.
```bash
roslaunch hlrobot_gazebo bringup_moveit.launch
```
Launch robot to start hand-eye calibration.
```bash
roslaunch hlrobot_gazebo bringup_calibration.launch
```

## File Structure
| File                    | Instruction                                                                                                                                                                                                           |
| ----------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| config                  | config file of controller in Gazebo                                                                                                                                                                                   |
| cubicTrajectoryPlanning | Include Forward and Inverse kinematics source file and motion planning source file to generate the music book. The music book (note and PPB) and the location of the insturment in Joint space (q_down) is also in it |
| data                    | The sound file of roll call and the picture for readme                                                                                                                                                                |
| launch                  | Launch file for simulation                                                                                                                                                                                            |
| meshes                  | The models for robot manipulators                                                                                                                                                                                     |
| rviz                    | The config file of rviz                                                                                                                                                                                               |
| scripts                 | Ros Python files                                                                                                                                                                                                      |
| udrf                    | The description file of the robot manipulators                                                                                                                                                                        |


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
data: [0.426645, 0.515256, 1.761281, -0.169471, 0.998398, 5.370273]" 
```

For Moveit, I change the controller into `position_controllers/JointTrajectoryController`.

## Inverse Kinematice
Set IKsolver.cpp to solve the inverse kinematic and control the robot.
You can also use moveit now.

## Frame
Frame `base_foorprint` and `tool_frame` are used to calibrate the world frame.(To get the world frame in lab) If you want to know the coordination in world frame, you can check the transform between them.
```bash
rosrun tf tf_echo base_footprint tool_frame
```
OR
```bash
rosrun hlrobot_gazebo show_tf.py
```
### Compare the frame in simulation and reality

![](hlrobot_gazebo/data/picture/Frame1_real.jpg)
![](hlrobot_gazebo/data/picture/Frame1_Simulation.png)

## Load music book and play music
**Edit the path of music book(PPB files) in `scripts/publisher.py `**  
Music book is in `cubicTrajectoryPlanning/data/PPB`  
**Edit the path of the `cubicTrajectoryPlanning/data/q_down.txt` in `scripts/player.py`** 

![Instrument](hlrobot_gazebo/data/picture/Instrument.jpg)
```bash
rosrun sound_play soundplay_node.py
rosrun hlrobot_gazebo play.py
rosrun hlrobot_gazebo publisher.py 
```

OR
```bash
roslaunch hlrobot_gazebo bringup_music.launch
roslaunch hlrobot_gazebo play_music.launch
```

## Hand eye calibration
Use this robot to get the data for hand eye calibration.
```bash
roslaunch hlrobot_gazebo bringup_calibration.launch
```
Use image_view to collect image.
```bash
rosrun image_view image_view image:=/camera/image_raw
```
Get pose
```bash
rosrun hlrobot_gazebo show_tf.py
```
![eye in hand](hlrobot_gazebo/data/picture/eye_in_hand.png)  
![eye to hand](hlrobot_gazebo/data/picture/handeye.png)

## Todo
- [x] load angle of joints from txt file to control the robot
- [x] Try to play music in gazebo
- [x] Show world coordination 
- [x] Implement the forward and inverse kinematics in this simulation
- [x] Simply control the robot by giving Cartesian coordinate
- [x] Moveit!
- [x] Hand eye calibration!
- [ ] Anything useful 



