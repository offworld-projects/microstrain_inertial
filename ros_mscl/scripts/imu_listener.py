#!/usr/bin/env python
# Copyright offworld.ai 2020

import numpy as np
import enum, pdb, time, signal, subprocess

import rospy, rosbag, roslib.packages
from sensor_msgs.msg import Imu

import tf

class IMUListener:
    
    def __init__(self, **kwargs):

        #self.oapi = OverseerAPI()

        # From microstrain_inertial/Examples/ros_mscl_py_example/src/listener/listener.py
        self._device_name = None
        self._name_param = None

        # From data_retention.py
        #self._topics = None
        #self._storage_path = None
        #self._state = OperationState.PAUSED
        #self._collection_thread = None
        #self._subsystem_name = None
        #self._rosbag_file_name = None
        #self._collection_flag = False # True when rosbag collection has started
        #self._do_compression = None # Compression will be available in future
        #self._do_encryption = None # Encryption will be available in future
        self._initialized = False # True when class attributes used to retention have initialized

        self.__config_params__ = ['encryption', 'topics', 'subsystem_name', 'compression', 'storage_path']
    
    def initialize(self):

        #self._rosbag_file_name = 'imu.bag'
        #bag = rosbag.Bag(self._rosbag_file_name)

        rospy.init_node('listener_node', anonymous=True)

        # get the device name parameter
        self._device_name = 'cv5'
        self._name_param = rospy.get_name()# + '/device'

        if rospy.has_param(self._name_param):
            self._device_name = rospy.get_param(self._name_param)

            # clear parameter for future use
            rospy.delete_param(self._name_param)
        
        self._initialized = True

        return True

    def callback(self, imu):
        
        angles = tf.transformations.euler_from_quaternion([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])

        print
        rospy.loginfo("Quaternion:          [%9.6f, %9.6f, %9.6f, %9.6f]", imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
        rospy.loginfo("Euler Angles (ZYX):  [%9.6f, %9.6f, %9.6f] rad", angles[0], angles[1], angles[2])
        rospy.loginfo("Angular Velocity:    [%9.6f, %9.6f, %9.6f] rad/s", imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
        rospy.loginfo("Linear Acceleration: [%9.6f, %9.6f, %9.6f] m/s^2", imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z)
    
    def listen(self):

        # subscribe to the imu/data topic
        #Parameters:
        #   topic - namespace (defined in launch file) and topic name
        #   message type - type of message expected
        #   callback - callback function to handle this data
        rospy.Subscriber(("/" + self._device_name + "/imu/data"), Imu, self.callback)
        rospy.loginfo(("listening to /" + self._device_name + "/imu/data"))
        
        rospy.spin()

if __name__ == "__main__":

    listener = IMUListener()
    if not listener._initialized:
        listener.initialize()
    listener.listen()