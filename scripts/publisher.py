#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def loadtxt():
    with open('vector.txt', 'r') as f:
        data = f.readlines()

def publish():
    pub = rospy.Publisher('/HL_controller/command', Float64MultiArray, queue_size=10)
    rospy.init_node('commander', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        array = []
        command = Float64MultiArray(data = array)
        pub.publish(command)
        rate.sleep()


if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass