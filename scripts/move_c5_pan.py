#!/usr/bin/env python

import rospy
import rospkg
import sys
import math
import tf
from math import pi
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from sensor_msgs.msg import RegionOfInterest
from sts_msgs.msg import Float64

FREQ = 10

class LeicaMovUtils:

    def __init__(self):
        self.move_permit = False
        self.first_positioned = False
        self.ini_scan_ori = 0
        self.end_scan_ori = 0
        self.horizontal_res = 1024 # default
        
    @staticmethod
    def orientation_to_quaternion(orientation):
        """Interpret orientation as quaternion"""

        quaternion = (orientation.x,
                      orientation.y,
                      orientation.z,
                      orientation.w)
        return quaternion

    @staticmethod
    def quaternion_to_orientation(quaternion):
        """Interpret quaternion as orientation"""

        pose = Pose()
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose.orientation

    def get_next_orientation(self, current_orientation, roll, pitch, yaw):
        """Apply roll, pitch, yaw to current orientation"""

        current_rotation = self.orientation_to_quaternion(current_orientation)
        euler_angle = tf.transformations.euler_from_quaternion(current_rotation)
        # update with angles increments
        r = euler_angle[0] + roll
        p = euler_angle[1] + pitch
        y = euler_angle[2] + yaw
        quaternion = tf.transformations.quaternion_from_euler(r, p, y)
        orientation = self.quaternion_to_orientation(quaternion)

        return orientation

    @staticmethod
    def get_init_pose():
        """Read model pose from Gazebo model state"""

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_state = rospy.ServiceProxy(
                '/gazebo/get_model_state', GetModelState)
            resp = get_state("c5", "world")
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def leica_window_callback(self, msg):
        self.move_permit = True
        # leica yaw-motion array
        self.ini_scan_ori = msg.x_offset - math.atan(msg.width/2)
        self.end_scan_ori = msg.x_offset + math.atan(msg.width/2)
        yaw_increment = abs(self.ini_scan_ori - self.end_scan_ori)/self.horizontal_res
        # array of <horizontal_res> elements populated with <yaw_increment>
        self.z_orientations = [yaw_increment] * self.horizontal_res
        # with <ini_scan_ori> as first element
        self.z_orientations[0] = self.ini_scan_ori

    def leica_resolution_callback(self, msg):
        self.horizontal_res = msg.data

    def main(self):
        """Start moving C5 model with defined YAW increment"""

        rospy.init_node('move_c5_pan')

        rospy.Subscriber("c5/simulator/window",
                         RegionOfInterest, self.leica_window_callback)

        rospy.Subscriber("c5/simulator/resolution",
                         Float64, self.leica_resolution_callback)

        r = rospy.Rate(FREQ)

        init_model_state = self.get_init_pose()

        state_msg = ModelState()
        state_msg.model_name = 'c5'
        state_msg.pose = init_model_state.pose

        rospy.wait_for_service('/gazebo/set_model_state')

        while not rospy.is_shutdown():
            if self.move_permit == True:
                for z_ori in self.z_orientations:
                    state_msg.pose.orientation = self.get_next_orientation(state_msg.pose.orientation,0,0,z_ori)
                    try:
                        set_state = rospy.ServiceProxy(
                            '/gazebo/set_model_state', SetModelState)
                        resp = set_state(state_msg)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s" % e
                    r.sleep()
                self.move_permit = False


if __name__ == '__main__':
    try:
        c5 = LeicaMovUtils()
        c5.main()
    except rospy.ROSInterruptException:
        pass
