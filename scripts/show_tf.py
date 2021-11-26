#!/usr/bin/env python
import rospy
import tf
from scipy.spatial.transform import Rotation as R


if __name__ == '__main__':
    rospy.init_node('TF_listener', anonymous=True)
    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_footprint', '/tool_frame', rospy.Time(0))
            r = R.from_quat(rot)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print("Translation:")
        print(trans)
        print("Rotation in Quaternion:")
        print(rot)
        print("Rotation in RPY(ZYZ in degree):")
        print(r.as_euler('ZYZ',degrees = 'True'))
        print(' ')
        rospy.sleep(1)