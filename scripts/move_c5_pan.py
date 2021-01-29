#!/usr/bin/env python

import rospy
import rospkg
import sys
import math
import tf
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Int64
from std_srvs.srv import Trigger, TriggerRequest

FREQ = 40

class LeicaMovUtils:

    def __init__(self):
        """Initialise atributes
        """
        self.move_permit = False
        self.first_positioned = False
        self.ini_scan_ori = 0
        self.end_scan_ori = 0
        self.horizontal_res = 1024 # default
        
    @staticmethod
    def orientation_to_quaternion(orientation):
        """Interpret orientation as quaternion

        Args:
            orientation (Quaternion): orientation of the Leica with Quaternion type

        Returns:
            quaternion (tuple): orientation of the Leica with tuple type
        """
        quaternion = (orientation.x,
                      orientation.y,
                      orientation.z,
                      orientation.w)
        return quaternion


    @staticmethod
    def quaternion_to_orientation(quaternion):
        """Interpret quaternion as orientation

        Args:
            quaternion (tuple): orientation of the Leica with tuple type

        Returns:
            pose.orientation (Quaternion): orientation of the Leica with Quaternion type
        """
        pose = Pose()
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose.orientation


    def get_next_orientation(self, current_orientation, roll, pitch, yaw):
        """Apply roll, pitch and yaw to the current orientation

        Args:
            current_orientation (tuple): current orientation of the Leica
            roll (int): angle increment around the x-axis
            pitch (int): angle increment around the y-axis
            yaw (int): angle increment around the z-axis

        Returns:
            orientation (Quaternion): orientation of the Leica after rotation 
        """
        # get current orientation of the Leica with Quaternion format
        current_rotation = self.orientation_to_quaternion(current_orientation)

        # get orientation in the form of Euler angles
        euler_angle = tf.transformations.euler_from_quaternion(current_rotation)
        
        # update orientation with angles increments
        r = euler_angle[0] + roll
        p = euler_angle[1] + pitch
        y = euler_angle[2] + yaw

        # put orientation back to Quaternion form with tuple type
        quaternion = tf.transformations.quaternion_from_euler(r, p, y)

        # put orientation back to Quaternion type
        orientation = self.quaternion_to_orientation(quaternion)

        return orientation

    @staticmethod
    def get_init_pose():
        """Read model pose from Gazebo model state

        Returns:
            (ModelState): initial position of the Leica
        """
        # make sure the service is available before calling it
        rospy.wait_for_service('/gazebo/get_model_state')

        # call the service
        try:
            get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = get_state("c5", "world")
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: " + str(e))

    def leica_window_callback(self, msg):
        """callback to generate the list of waypoints for the Leica to move through

        Args:
            msg (RegionOfInterest): region of interest for the scanning
        """
        # enables the movement of the Leica after the configuration of the scan window
        self.move_permit = True

        # calculation of the angle increment that the Leica has to be moved in yaw
        # convert width to m
        self.ini_scan_ori = msg.x_offset - math.atan(msg.width*1e-3/2)
        self.end_scan_ori = msg.x_offset + math.atan(msg.width*1e-3/2)
        yaw_increment = abs(self.ini_scan_ori - self.end_scan_ori)/self.horizontal_res

        # array of <horizontal_res> elements populated with <yaw_increment> with <ini_scan_ori> as first element
        self.z_orientations = [float(yaw_increment)] * self.horizontal_res
        self.z_orientations[0] = float(self.ini_scan_ori)

    def leica_resolution_callback(self, msg):
        """function for adjusting the resolution of the sensor

        Args:
            msg (RegionOfInterest): region of interest for the scanning
        """
        self.horizontal_res = msg.data

    def main(self):
        """Move the Leica according to the defined yaw increment
        """
        # initialise the node responsible for the Leica's movement and subscribe to the scanning configuration topics
        rospy.init_node('move_c5_pan')
        rospy.Subscriber("c5/simulator/window", RegionOfInterest, self.leica_window_callback)
        rospy.Subscriber("c5/simulator/resolution", Int64, self.leica_resolution_callback)

        # prepares the point cloud saving service
        rospy.wait_for_service('/c5/store_cloud')
        srv_save_cloud = rospy.ServiceProxy('/c5/store_cloud', Trigger)
        trigger = TriggerRequest()

        # this value is used to maintain the loop which is responsible for moving the Leica to the desired frequency.
        r = rospy.Rate(FREQ)

        # gets the initial position of the Leica and populates the state message with it
        init_model_state = self.get_init_pose()
        state_msg = ModelState()
        state_msg.model_name = 'c5'
        state_msg.pose = init_model_state.pose

        # stops the progam until the topic is enabled
        rospy.wait_for_service('/gazebo/set_model_state')

        # rotate the Leica to each of the generated angles once the scan window has been configured
        while not rospy.is_shutdown():

            if self.move_permit == True:

                for z_ori in self.z_orientations:
                    state_msg.pose.orientation = self.get_next_orientation(state_msg.pose.orientation, 0.0, 0.0, z_ori)
                    
                    try:
                        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                        resp = set_state(state_msg)
                    except rospy.ServiceException as e:
                        print("Service call failed: " + str(e))
                    
                    r.sleep()

                rospy.loginfo("move_c5_pan: movement finished")
                srv_res = srv_save_cloud(trigger)
                self.move_permit = False


if __name__ == '__main__':
    try:
        # create instance of LeicaMovUtils and run the Leica's motion program
        c5 = LeicaMovUtils()
        c5.main()
    except rospy.ROSInterruptException:
        pass