#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
import numpy as np

def callback(data):
    ispublished = 0
    while not ispublished:
        try:
            (trans,rot) = listener.lookupTransform('/base_footprint', '/tool_frame', rospy.Time(0))
            r = R.from_quat(rot)
            print("Joint State:")
            location = np.array([data.position[0], data.position[1], data.position[2],
                                data.position[3], data.position[4], data.position[5]])
            print(location*180/3.14)
            print('--------------------------------------------------')
            r = R.from_quat(rot)
            print("Translation:")
            translation = np.array(trans)
            print(translation*1000)
            print('--------------------------------------------------')
            print("Rotation in Quaternion:")
            print(rot)
            print("Rotation in RPY(ZYZ in degree):")
            print(r.as_euler('ZYZ',degrees = 'True'))
            print('--------------------------------------------------')
            print(' ')
            ispublished = 1
            rospy.sleep(2)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #(trans,rot) = listener.lookupTransform('/base_footprint', '/tool_frame', rospy.Time(0))




if __name__ == '__main__':
    rospy.init_node('TF_listener', anonymous=True)
    listener = tf.TransformListener()
    rospy.Subscriber("/joint_states",JointState,callback)
    rospy.spin()

    # while not rospy.is_shutdown():
        # try:
        #     (trans,rot) = listener.lookupTransform('/base_footprint', '/tool_frame', rospy.Time(0))
        #     r = R.from_quat(rot)

        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue
    #     print("Translation:")
    #     print(trans)
    #     print("Rotation in Quaternion:")
    #     print(rot)
    #     print("Rotation in RPY(ZYZ in degree):")
    #     print(r.as_euler('ZYZ',degrees = 'True'))
    #     print(' ')
    #     rospy.sleep(1)