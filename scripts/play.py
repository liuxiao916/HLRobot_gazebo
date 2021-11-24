#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
import roslib; roslib.load_manifest('sound_play')
from sound_play.libsoundplay import SoundClient


sound_client = SoundClient()
sound_C = sound_client.waveSound('/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Do.wav')
sound_D = sound_client.waveSound('/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Re.wav')
sound_E = sound_client.waveSound('/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Mi.wav')
sound_F = sound_client.waveSound('/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Fa.wav')
sound_G = sound_client.waveSound('/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Sol.wav')
sound_A = sound_client.waveSound('/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/La.wav')
sound_B = sound_client.waveSound('/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/sounds/Si.wav')

threshold = 0.02
q_down = np.array([-20.5356, 32.5688, 97.2993, -15.9786, 75.987, 266.835,\
                    -17.6588, 31.5739, 98.986, -14.6972, 75.9011, 269.207,\
                    -14.6871, 30.7313, 100.405, -13.3365, 75.8866, 271.6,\
                    -11.6324, 30.0452, 101.553, -11.9043,  75.9363, 274.011,\
                    -8.50929, 29.5196, 102.429, -10.4105,  76.0437, 276.435,\
                    -5.3344, 29.1581, 103.029, -8.8669, 76.2031, 278.87,\
                    -2.12634, 28.9631, 103.352, -7.28682, 76.4101, 281.308,\
                    1.0951, 28.9361, 103.397, -5.68454, 76.6616, 283.743,\
                    4.30963, 29.0774, 103.163, -4.07493, 76.9562, 286.169,\
                    7.49724, 29.3858, 102.651, -2.47275, 77.294, 288.578,\
                    10.6389, 29.8592, 101.864, -0.892131, 77.6767, 290.963,\
                    13.7172, 30.4943, 100.802, 0.653931, 78.1078, 293.32,\
                    16.7169, 31.2873, 99.4697, 2.15402, 78.5917, 295.643,\
                    19.6252, 32.2339, 97.8685,  3.5986, 79.1343,  297.93,\
                    22.432, 33.33, 96.001, 4.98014, 79.7421, 300.178])
q_down = q_down.reshape([15,6])
q_down = q_down/180*3.1415926



def callback(data):
    location = np.array([data.position[0],data.position[1],data.position[2],data.position[3],data.position[4],data.position[5]+3.1415926])
    diff = np.linalg.norm(location - q_down,axis = 1)
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
        elif(index == 15):
            sound_G.play()
        rospy.sleep(0.5)
        



    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Music_player', anonymous=True)

    rospy.Subscriber("/joint_states", JointState, callback, queue_size = 1, buff_size=2**24)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()