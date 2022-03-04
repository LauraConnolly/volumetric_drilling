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
        self.deadband = 0.01
        self.G = 9.81
        self.time = time.time()

        # x update
        self.first_pass = True
        self.velocity_0 = 0
        self.acceleration_0 = 0
        self.position_0 = 0
        self.velocity = 0
        self.acceleration = 0
        self.position = 0

        # y update
        self.first_pass_y = True
        self.velocity_0_y = 0
        self.acceleration_0_y = 0
        self.position_0_y = 0
        self.velocity_y = 0
        self.acceleration_y = 0
        self.position_y = 0

        # z update
        self.first_pass_z = True
        self.velocity_0_z = 0
        self.acceleration_0_z = 0
        self.position_0_z = 0
        self.velocity_z = 0
        self.acceleration_z = 0
        self.position_z = 0

        # Stating position of the camera
        self.camera_name = '/ambf/env/cameras/main_camera'
        self.mCam_obj = self._client.get_obj_handle(self.camera_name)
        camera_pos = self.mCam_obj.get_pos()
        # Add the position changed based on the current pos (so it doesn't jump at the start)
        self.position_curr_x = camera_pos.x # Updates to this will come from double integration
        self.position_curr_y = camera_pos.y
        self.position_curr_z = camera_pos.z

    def callback(self, data):
        #TODO: Need to figure out direction

        # There is a unit issue happening here! - changes need to be more obvious
        x = data.linear_acceleration.x # Unit = m/s^2
        y = data.linear_acceleration.y
        z = data.linear_acceleration.z
        #print('acc x:{}, y:{}, z:{}'.format(x,y,z))

        #TODO: figure out direction change issue

        self.updateXposition(x)
        self.updateYposition(y)
        self.updateZposition(z - self.G) # Add in gravity compensation

        # Update reference variables - which will act like previous position, velcotiy and accleration in the next pass
        #print("Camera position is x: {}, y: {}, z: {}".format(self.position, self.position_y, self.position_z))
        self.time = time.time()

        # Update the camera
        ros_point = Point()

        ros_point.x = self.position_curr_x #+ self.position_z
        ros_point.y = self.position_curr_y + self.position_y
        ros_point.z = self.position_curr_z #+ self.position

        self.mCam_obj.set_pos(ros_point.x, ros_point.y, ros_point.z)


    def updateXposition(self, x):


        # Just do it for x to start
        if self.first_pass is True:
            delta_a = x - self.acceleration_0
            if np.abs(delta_a) < self.deadband: # This is called the deadband
                self.position = self.position
                self.velocity = self.velocity
                self.acceleration = x
            else:
                velocity = self.velocity_0 + ((self.acceleration_0 + x)*(1/2))*(time.time() - self.time)
                delta_a = x - self.acceleration_0
                position = self.position_0 + ((velocity +  self.velocity_0)*(1/2))*(time.time() - self.time)
                self.first_pass = False
                self.velocity = velocity
                self.position = position
                self.acceleration = x
        else:
            delta_a = x - self.acceleration
            if np.abs(delta_a) < self.deadband:
                self.position = self.position
                self.velocity = self.velocity
                self.acceleration = x
            else:
                velocity = self.velocity + ((self.acceleration + x)*(1/2))*(time.time() - self.time)
                position = self.position + ((velocity +  self.velocity)*(1/2))*(time.time() - self.time)
                self.velocity = velocity
                self.position = position
                self.acceleration = x


    def updateYposition(self, y):


        if self.first_pass_y is True:
            delta_a = y - self.acceleration_0_y
            if np.abs(y) < self.deadband:
                #print('deadband active: {}'.format(y))
                y = 0
                self.position_y = self.position_y
                self.velocity_y = self.velocity_y
                self.acceleration_y = y
            else:
                print('deadband inactive: {}'.format(y))
                velocity = self.velocity_0_y + ((self.acceleration_0_y + y)*(1/2))*(time.time() - self.time)
                delta_a = y - self.acceleration_0_y
                position = self.position_0_y + ((velocity +  self.velocity_0_y)*(1/2))*(time.time() - self.time)
                self.first_pass_y = False
                self.velocity_y = velocity
                self.position_y = position
                self.acceleration_y = y
                print('started')
        else:
            delta_a = y - self.acceleration_y
            #if np.abs(delta_a) < self.deadband:
            #if (0 < y < self.deadband) or (0 > y > self.deadband_neg):
            if np.abs(y) < self.deadband:
                #print('deadband active: {}'.format(y))
                y = 0
                self.position_y = self.position_y
                self.velocity_y = self.velocity_y
                self.acceleration_y = y
            else:
                print('deadband inactive: {}'.format(y))
                velocity = self.velocity_y + ((self.acceleration_y + y)*(1/2))*(time.time() - self.time)
                position = self.position_y + ((velocity +  self.velocity_y)*(1/2))*(time.time() - self.time)
                self.velocity_y = velocity
                self.position_y = position
                self.acceleration_y = y

    def updateZposition(self, z):

        # Just do it for x to start
        if self.first_pass_z is True:
            delta_a = z - self.acceleration_0_z
            if np.abs(delta_a) < self.deadband:
                self.position_z = self.position_z
                self.velocity_z = self.velocity_z
                self.acceleration_z = z
            else:
                velocity = self.velocity_0_z + ((self.acceleration_0_z + z)*(1/2))*(time.time() - self.time)
                delta_a = z - self.acceleration_0_z
                position = self.position_0_z + ((velocity +  self.velocity_0_z)*(1/2))*(time.time() - self.time)
                self.first_pass_z = False
                self.velocity_z = velocity
                self.position_z = position
                self.acceleration_z = z
        else:
            delta_a = z - self.acceleration_z
            if np.abs(delta_a) < self.deadband:
                self.position_z = self.position_z
                self.velocity_z = self.velocity_z
                self.acceleration_z = z
            else:
                velocity = self.velocity_z + ((self.acceleration_z + z)*(1/2))*(time.time() - self.time)
                position = self.position_z + ((velocity +  self.velocity_z)*(1/2))*(time.time() - self.time)
                self.velocity_z = velocity
                self.position_z = position
                self.acceleration_z = z

    def listener(self):

        rospy.Subscriber("/imu", Imu, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

p = imu_camerapos()
p.listener()
