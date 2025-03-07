#!/usr/bin/env python

import rospy
import tf

def get_transform():
    rospy.init_node('tf_listener', anonymous=True)

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            print("Translation: ", trans)
            print("Rotation: ", rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        get_transform()
    except rospy.ROSInterruptException:
        pass

