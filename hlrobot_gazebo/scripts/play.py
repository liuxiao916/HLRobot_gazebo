#!/usr/bin/env python
import os
from sound_play.libsoundplay import SoundClient
import rospy
import numpy as np
from sensor_msgs.msg import JointState
import roslib
roslib.load_manifest('sound_play')

parent_path = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
path = os.path.join(parent_path,"cubicTrajectoryPlanning/data/q_down.txt")

#path = "/home/liuxiao/catkin_ws/src/HLRobot_gazebo/cubicTrajectoryPlanning/data/q_down.txt"


sound_client = SoundClient()
# sound_C = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Do.wav')
# sound_D = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Re.wav')
# sound_E = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Mi.wav')
# sound_F = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Fa.wav')
# sound_G = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Sol.wav')
# sound_A = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/La.wav')
# sound_B = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Si.wav')
sound_C = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/Do.wav"))
sound_D = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/Re.wav"))
sound_E = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/Mi.wav"))
sound_F = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/Fa.wav"))
sound_G = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/Sol.wav"))
sound_A = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/La.wav"))
sound_B = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/Si.wav"))

# sound_1 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/1.wav')
# sound_2 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/2.wav')
# sound_3 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/3.wav')
# sound_4 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/4.wav')
# sound_5 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/5.wav')
# sound_6 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/6.wav')
# sound_7 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/7.wav')
# sound_8 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/8.wav')
# sound_9 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/9.wav')
# sound_10 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/10.wav')
# sound_11 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/11.wav')
# sound_12 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/12.wav')
# sound_13 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/13.wav')
# sound_14 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/14.wav')
# sound_15 = sound_client.waveSound(
#     '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/15.wav')

sound_1 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/1.wav"))
sound_2 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/2.wav"))
sound_3 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/3.wav"))
sound_4 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/4.wav"))
sound_5 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/5.wav"))
sound_6 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/6.wav"))
sound_7 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/7.wav"))
sound_8 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/8.wav"))
sound_9 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/9.wav"))
sound_10 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/10.wav"))
sound_11 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/11.wav"))
sound_12 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/12.wav"))
sound_13 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/13.wav"))
sound_14 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/14.wav"))
sound_15 = sound_client.waveSound(
    os.path.join(parent_path,"data/sounds/15.wav"))

threshold = 0.01

q_down = np.zeros([15, 6])
with open(path, 'r') as f:
    lines = f.readlines()
    for index, line in enumerate(lines):
        txtdata = line.split()
        q_down[index] = txtdata
    q_down = q_down/180*3.1415926


def callback(data):
    location_1 = np.array([data.position[0], data.position[1], data.position[2],
                         data.position[3], data.position[4], data.position[5]])
    location_2 = np.array([data.position[0], data.position[1], data.position[2],
                         data.position[3], data.position[4], data.position[5]+2*3.1415926])
    diff_1 = np.linalg.norm(location_1 - q_down, axis=1)
    diff_2 = np.linalg.norm(location_2 - q_down, axis=1)
    if(diff_1[0] > diff_2[0]):
        diff = diff_2
    else:
        diff = diff_1
            
    if(np.min(diff) < threshold):
        index = np.argmin(diff)
        print(index)
        if(index == 0):
            sound_G.play()
        elif(index == 1):
            sound_A.play()
        elif(index == 2):
            sound_B.play()
        elif(index == 3):
            sound_C.play()
        elif(index == 4):
            sound_D.play()
        elif(index == 5):
            sound_E.play()
        elif(index == 6):
            sound_F.play()
        elif(index == 7):
            sound_G.play()
        elif(index == 8):
            sound_A.play()
        elif(index == 9):
            sound_B.play()
        elif(index == 10):
            sound_C.play()
        elif(index == 11):
            sound_D.play()
        elif(index == 12):
            sound_E.play()
        elif(index == 13):
            sound_F.play()
        elif(index == 14):
            sound_G.play()
        rospy.sleep(0.3)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Music_player', anonymous=True)

    rospy.Subscriber("/joint_states", JointState, callback,
                     queue_size=1, buff_size=2**24)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
