#!/usr/bin/env python
import math
import time
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose
import time
import numpy as np

class lane_controller():

    def __init__(self):
        self.node_name = rospy.get_name()

        self.prev_time = None

        # Publication
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.updatePose, queue_size=1)
        self.pose_msg = LanePose()

        # Setup PID gains
        self.k_P_phi = -3.5
        self.k_P_d = -2.5

        self.k_I_d = 0.5
        self.k_I_phi = 0.1

        self.k_D_d = 0.7
        self.k_D_phi = 0.4

        # Initialization
        self.prev_d_err = 0
        self.prev_phi_err = 0
        self.v_ref = 0.5
        self.v_max = 1

        self.d_offset = 0
        self.d_integral_top_cutoff = 0.3
        self.d_integral_bottom_cutoff = -0.3
        self.phi_integral_top_cutoff = 1.0
        self.phi_integral_bottom_cutoff = -1.0


    def updatePose(self, pose_msg):

        d_err = pose_msg.d - self.d_offset
        phi_err = pose_msg.phi

        car_control_msg = Twist2DStamped()

        t = rospy.get_time()

        if self.prev_time is not None:
            dt = t-self.prev_time
            d_integral = d_err * dt
            phi_integral = phi_err * dt

            if d_integral > self.d_integral_top_cutoff:
                d_integral = self.d_integral_top_cutoff
            if d_integral < self.d_integral_bottom_cutoff:
                d_integral = self.d_integral_bottom_cutoff

            if phi_integral > self.phi_integral_top_cutoff:
                phi_integral = self.phi_integral_top_cutoff
            if phi_integral < self.phi_integral_bottom_cutoff:
                phi_integral = self.phi_integral_bottom_cutoff

            # To not keep correcting intergral
            if abs(d_err) <= 0.02:
                d_integral = 0
            if abs(phi_err) <= 0.25:
                phi_integral = 0
            if np.sign(d_err) != np.sign(self.prev_d_err):  # sign of error changed => error passed zero
                d_integral = 0
            if np.sign(phi_err) != np.sign(self.prev_phi_err):  # sign of error changed => error passed zero
                phi_integral = 0

            omega = self.k_P_d * d_err + self.k_P_phi * phi_err
            omega += d_integral + phi_integral
            omega += self.k_D_d * (d_err - self.prev_d_err)/dt + self.k_D_phi * (phi_err - self.prev_phi_err)/dt
            omega = max(min(omega,5),-5)
            car_control_msg.v = 0.2
            car_control_msg.omega = omega

            self.prev_d_err = d_err
            self.prev_phi_err = phi_err

        self.prev_time = t

        self.pub_car_cmd.publish(car_control_msg)


if __name__ == "__main__":

    rospy.init_node("lane_controller_node", anonymous=False)  # adapted to sonjas default file

    lane_controller_node = lane_controller()
rospy.spin()
