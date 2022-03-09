import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from ambf_client import Client
import time
import os
import math
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

#TODO: need to create a subscriber - can't have ambf node and the one I'm initiang in the one file
#Two things to figure out - how do you tell which direction is down -
class imu_camerapos:

    def __init__(self):
        # Set up the python client connection
        bridge = CvBridge()
        self._client = Client()
        self._client.connect()

        # Constants
        self.deadband_x = 0.15
        self.deadband_y = 0.15
        self.G = 9.81
        self.time = time.time()


        self.velocity = 0
        self.acceleration = 0
        #self.position = 0

        self.velocity_y = 0
        self.acceleration_y = 0
        #self.position_y = 0

        self.velocity_z = 0
        self.acceleration_z = 0
        #self.position_z = 0

        # Stating position of the camera
        self.camera_name = '/ambf/env/cameras/main_camera'
        self.mCam_obj = self._client.get_obj_handle(self.camera_name)
        camera_pos = self.mCam_obj.get_pos()
        self.position = camera_pos.x # Initial position of the camera ! Updates to this will come from double integration
        self.position_y = camera_pos.y
        self.position_z = camera_pos.z

        self.min_y = 0.
        self.max_y = 0.

    def callback(self, data):

        # There is a unit issue happening here! - changes need to be more obvious
        x = data.linear_acceleration.x # Unit = m/s^2
        y = data.linear_acceleration.y
        z = data.linear_acceleration.z - self.G

        vel, pos, acc = self.computerPosition(x, self.velocity, self.position, self.acceleration, self.deadband_x)
        self.velocity = vel
        self.position = pos
        self.acceleration = acc
        #print('position | x', self.position)

        vel, pos, acc = self.computerPosition(y, self.velocity_y, self.position_y, self.acceleration_y, self.deadband_y)
        self.velocity_y = vel
        self.position_y = pos
        self.acceleration_y = acc
        #print('position | y', self.position_y)

        # vel, pos, acc = self.computerPosition(z, self.velocity_z, self.position_z, self.acceleration_z)
        # self.velocity_z = vel
        # self.position_z = pos
        # self.acceleration_z = acc
        #print('position x |', self.position, '| position y |', self.position_y) #'| position z |', self.position_z)

        self.time = time.time()

        # Update the camera
        ros_point = Point()
        ros_point.x = self.position_z
        ros_point.y = self.position_y
        ros_point.z = self.position


        self.mCam_obj.set_pos(ros_point.x, ros_point.y, ros_point.z)

    #TODO: make the current position just the starting point and then update the position from there - so we're not ever resetting to that place we just
    # have position be an accurate reference - double check that code works with emailed version

    def computerPosition(self, acc, vel_global, pos_global, acc_global, deadband):

        dt = time.time() - self.time

        if np.abs(acc) < deadband:
            acc = 0.
            position = pos_global
            velocity = vel_global + ((acc_global + acc)*(1./2.))*dt # compute dt once
        else:
            velocity = vel_global + ((acc_global + acc)*(1./2.))*dt # compute dt once
            position = pos_global + ((vel_global + velocity)*(1./2.))*dt

        return velocity, position, acc

    def listener(self):

        rospy.Subscriber("/imu", Imu, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

p = imu_camerapos()
p.listener()
