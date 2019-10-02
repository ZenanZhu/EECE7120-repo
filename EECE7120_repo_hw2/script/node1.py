#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from random import uniform

def node1():
    rospy.init_node('node1', anonymous=True)
    pub = rospy.Publisher('/obstacles_detected', Point, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = Point()
        msg.x = uniform(0,50)
        msg.y = uniform(0,50)
        msg.z = uniform(0,50)
        rospy.loginfo("x-direction: %s, y-direction: %f, z-direction: %f", msg.x, msg.y, msg.z)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        node1()
    except rospy.ROSInterruptException:
        pass
