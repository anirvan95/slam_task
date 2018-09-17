#!/usr/bin/env python

#program to process the data.mat and send message to robot_localization package in ROS
import math
from math import sin, cos, pi
import scipy.io as sio
import numpy as np
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu


class data_pub():

    def __init__(self):#constructor
        control_frequency = 100
        rospy.loginfo("To stop sending data CTRL + C")
        rospy.on_shutdown(self.shutdown)
        #defining the publishers
        self.pub_odom = rospy.Publisher("/speedometer", Odometry, queue_size=1)
        self.pub_imu = rospy.Publisher('/imu/raw', Imu, queue_size=1)
        self.pub_gnss = rospy.Publisher('/gnss', Odometry, queue_size=1)
        self.looprate = rospy.Rate(control_frequency)

        self.imu_data = []
        self.speedometer_data = []
        self.gnss_data = []

    def load_data(self,data):#function which loads data to list
        self.imu_data = data["IMU"][0][0]
        self.speedometer_data = data["SPEEDOMETER"][0][0]
        self.gnss_data = data["GNSS"][0][0]

    def start(self):
        imu_count = 0
        speed_count = 0
        gnss_count = 0
        time = 0
        total_time  = self.imu_data["t"][0][0][-1][0]
        while ((not rospy.is_shutdown()) and time < total_time):#checking if the IMU has stopped publishin

            time = self.imu_data["t"][0][0][imu_count][0]

            #sending IMU data
            acc = self.imu_data["acc"][0][0][0:, imu_count]
            gyro = self.imu_data["gyro"][0][0][0:, imu_count]
            imuMsg = Imu()
            imuMsg.header.stamp.secs = time
            imuMsg.header.frame_id = "base_link"
            #defining the covariance matrix for IMU measurement
            imuMsg.angular_velocity_covariance = [
                0.002, 0, 0,
                0, 0.002, 0,
                0, 0, 0.002
            ]

            imuMsg.linear_acceleration_covariance = [
                0.08, 0, 0,
                0, 0.08, 0,
                0, 0, 0.08
            ]

            imuMsg.angular_velocity.x = gyro[1]
            imuMsg.angular_velocity.y = gyro[0]
            imuMsg.angular_velocity.z = -gyro[2]

            imuMsg.linear_acceleration.x = acc[1]
            imuMsg.linear_acceleration.y = acc[0]
            imuMsg.linear_acceleration.z = -acc[2]
            self.pub_imu.publish(imuMsg)

            imu_count = imu_count + 1


            # sending Odom data
            if(self.speedometer_data["t"][0][0][speed_count][0] < time):
                odomMsg = Odometry()
                odomMsg.header.stamp.secs = self.speedometer_data["t"][0][0][speed_count][0]
                odomMsg.header.frame_id = "odom"
                odomMsg.child_frame_id = "base_link"
                odomMsg.twist.twist.linear.x = self.speedometer_data["speed"][0][0][0][speed_count]
                odomMsg.twist.twist.linear.y = 0.0
                self.pub_odom.publish(odomMsg)
                speed_count = speed_count + 1

            #sending Map data
            if (self.gnss_data["t"][0][0][gnss_count][0] < time):
                gnssMsg = Odometry()
                ned_pos = self.gnss_data["pos_ned"][0][0][0:, gnss_count]
                gnssMsg.header.stamp.secs = self.gnss_data["t"][0][0][gnss_count][0]
                gnssMsg.header.frame_id = "map"
                gnssMsg.pose.pose.position.x = ned_pos[1]
                gnssMsg.pose.pose.position.y = ned_pos[0]
                gnssMsg.pose.pose.position.z = 0.0
                self.pub_gnss.publish(gnssMsg)
                gnss_count = gnss_count + 1

            self.looprate.sleep()

    def shutdown(self):
        rospy.loginfo("Stopped sending data\n")
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('car_data_pub', anonymous=False)  # initialize node
    data_node = data_pub()
    # !!....................change the file path here ........... !!
    data = sio.loadmat('/home/anirvan/catkin_ws/src/slam_kthsdv/script/data.mat')['in_data']
    data_node.load_data(data)
    data_node.start()
    data_node.shutdown()
