#!/usr/bin/env python
import rospy
from EECE7120_repo_hw2.msg import hw2
from std_msgs.msg import String
from geometry_msgs.msg import Point

id_num = 0

def callback(data):
    global id_num
    out_msg = hw2()
    out_msg.id = str(id_num)
    out_msg.x = data.x
    out_msg.y = data.y
    out_msg.z = data.z
    id_num += 1
    pub.publish(out_msg)

if __name__ == '__main__':
    rospy.init_node('node2', anonymous=True)
    rospy.Subscriber('/obstacles_detected', Point, callback)
    pub = rospy.Publisher('/registered_obstacles', hw2, queue_size=10)
    rate= rospy.Rate(10)
    rospy.spin()
