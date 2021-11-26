#!/usr/bin/env python
from sound_play.libsoundplay import SoundClient
import rospy
import numpy as np
from sensor_msgs.msg import JointState
import roslib
roslib.load_manifest('sound_play')


sound_client = SoundClient()
sound_C = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Do.wav')
sound_D = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Re.wav')
sound_E = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Mi.wav')
sound_F = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Fa.wav')
sound_G = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Sol.wav')
sound_A = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/La.wav')
sound_B = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Si.wav')

sound_1 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/1.wav')
sound_2 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/2.wav')
sound_3 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/3.wav')
sound_4 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/4.wav')
sound_5 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/5.wav')
sound_6 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/6.wav')
sound_7 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/7.wav')
sound_8 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/8.wav')
sound_9 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/9.wav')
sound_10 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/10.wav')
sound_11 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/11.wav')
sound_12 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/12.wav')
sound_13 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/13.wav')
sound_14 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/14.wav')
sound_15 = sound_client.waveSound(
    '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/15.wav')

threshold = 0.02

q_down = np.zeros([15, 6])
with open('/Users/liuxiao/Documents/GitHub/HLRobot_gazebo/cubicTrajectoryPlanning/data/q_down.txt', 'r') as f:
    lines = f.readlines()
    for index, line in enumerate(lines):
        txtdata = line.split()
        q_down[index] = txtdata/180*3.1415926


def callback(data):
    location = np.array([data.position[0], data.position[1], data.position[2],
                         data.position[3], data.position[4], data.position[5]+3.1415926])
    diff = np.linalg.norm(location - q_down, axis=1)
    if(np.min(diff) < threshold):
        index = np.argmin(diff)
        print(index)
        if(index == 0):
            sound_1.play()
        elif(index == 1):
            sound_2.play()
        elif(index == 2):
            sound_3.play()
        elif(index == 3):
            sound_4.play()
        elif(index == 4):
            sound_5.play()
        elif(index == 5):
            sound_6.play()
        elif(index == 6):
            sound_7.play()
        elif(index == 7):
            sound_8.play()
        elif(index == 8):
            sound_9.play()
        elif(index == 9):
            sound_10.play()
        elif(index == 10):
            sound_11.play()
        elif(index == 11):
            sound_12.play()
        elif(index == 12):
            sound_13.play()
        elif(index == 13):
            sound_14.play()
        elif(index == 14):
            sound_15.play()
        rospy.sleep(0.5)


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
