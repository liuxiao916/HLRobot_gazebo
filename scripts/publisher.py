#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

path = '/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/test_fromOneToAnother.txt'

def publish():
    PI = 3.1415926
    pub = rospy.Publisher('/HL_controller/command', Float64MultiArray, queue_size=10)
    rospy.init_node('commander', anonymous=True)
    rate = rospy.Rate(1000) 
    with open(path, 'r') as f:
        lines = f.readlines()
        for line in lines:  
            if not rospy.is_shutdown():
                txtdata = line.split()
                angle = []
                angle.append(float(txtdata[0])/180*PI)
                angle.append(float(txtdata[1])/180*PI)
                angle.append(float(txtdata[2])/180*PI)
                angle.append(float(txtdata[3])/180*PI)
                angle.append(float(txtdata[4])/180*PI)
                angle.append(float(txtdata[5])/180*PI)
                command = Float64MultiArray(data = angle)
                pub.publish(command)
                rate.sleep()


if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass