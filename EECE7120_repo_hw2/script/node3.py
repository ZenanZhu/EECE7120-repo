#!/usr/bin/env python
import rospy
from EECE7120_repo_hw2.msg import hw2

def callback(data):
    rospy.loginfo('Detected obstacle <%s> at position <%f>,<%f>,<%f>', data.id, data.x, data.y, data.z)

def node3():

    rospy.init_node('node3', anonymous=True)

    rospy.Subscriber("/registered_obstacles", hw2, callback)

    rospy.spin()

if __name__ == '__main__':
    node3()
