#!/usr/bin/env python

import rospy
from math import pi
from duckietown_msgs.msg import Twist2DStamped

if __name__ == '__main__':
    rospy.init_node('open_loop', anonymous=True)
    pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    t_start = rospy.get_time()
    while not rospy.is_shutdown():
        t = rospy.get_time()
        msg = Twist2DStamped()
        dt = t - t_start
        if dt > 5 and dt < 7:
            msg.v = .5
            msg.omega = -0.5
        elif dt > 7.5 and dt < 8.5:
            msg.v = 0.0
            msg.omega = pi/2.5
        elif dt > 9 and dt < 11:
            msg.v = 0.5
            msg.omega = -0.3
        elif dt > 11.5 and dt < 12.5:
            msg.v = 0.0
            msg.omega = pi/2.5
        elif dt > 13 and dt < 15:
            msg.v = 0.5
            msg.omega =-0.2
        elif dt > 15.5 and dt < 16.5:
            msg.v = 0.0
            msg.omega = pi/2.5
        elif dt > 17 and dt < 19:
            msg.v = 0.5
            msg.omega = -0.1
        pub.publish(msg)
        rate.sleep()
