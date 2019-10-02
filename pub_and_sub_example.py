#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
from basic_ros_example.msg import Example

# adapted from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

class PubAndSubExample:
    def __init__(self):
        self.pub = rospy.Publisher('output', Example, queue_size=10)
        rospy.Subscriber("input", Int64, self.number_cb)
        self.current_num = -1
        self.id_num = 0
        


    def number_cb(self, message):
        out_msg = Example()
        out_msg.id = 'input_{0}'.format(self.id_num)
        out_msg.value = message.data
        self.current_num = int(message.data)
        self.id_num += 1
        self.pub.publish(out_msg)

if __name__ == '__main__':
    try:
        ex = PubAndSubExample()
        rospy.init_node('pub_and_sub_example', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():             
            rospy.loginfo(ex.current_num)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
