#! /usr/bin/env python3

###############################################################################
# roboboat_navio_imu.py
#
# Package that provides uses the Navio2 Python API to read the IMU and publish
# the information on standard ROS messages
#
# Created: 05/19/21
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - @doc_vaughan
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * 
#
# TODO:
#   * 
###############################################################################

import numpy as np

# ROS related imports
import rospy
from sensor_msgs.msg import Imu, MagneticField

class NavioIMUNode(object):
    def __init__(self):
        rospy.init_node('navio_imu')

        self.frame_id = rospy.get_param('~frame_id', "/imu")

        self.throttle_rate = rospy.get_param('~throttle_rate', 10000)

        self.link = rospy.get_param('~link', 'imu_link')
        rospy.loginfo("tf link: {}".format(self.link))

        self.imu_data = Imu()
        self.imu_data = Imu(header=rospy.Header(frame_id=self.link))
        
        # TODO: 05/19/21 - JEV - Determine this values, either from datasheet or experimentally
        linear_acceleration_stdev = rospy.get_param("~linear_acceleration_stdev", 4.0 * 1e-3 * 9.80665)
        angular_velocity_stdev = rospy.get_param("~angular_velocity_stdev", np.deg2rad(0.06))

        linear_acceleration_cov = linear_acceleration_stdev * linear_acceleration_stdev
        angular_velocity_cov = angular_velocity_stdev * angular_velocity_stdev

        # TODO: 05/19/21 - JEV - Determine this values, either from datasheet or experimentally
        orientation_x_stdev = rospy.get_param("~orientation_x_stdev", np.deg2rad(3.0))
        orientation_y_stdev = rospy.get_param("~orientation_y_stdev", np.deg2rad(3.0))
        orientation_z_stdev = rospy.get_param("~orientation_z_stdev", np.deg2rad(5.0))

        orientation_x_covar = orientation_x_stdev * orientation_x_stdev
        orientation_y_covar = orientation_y_stdev * orientation_y_stdev
        orientation_z_covar = orientation_z_stdev * orientation_z_stdev

        self.imu_data.orientation_covariance = [orientation_x_covar, 0, 0, 
                                                0, orientation_y_covar, 0, 
                                                0, 0, orientation_z_covar]

        self.imu_data.angular_velocity_covariance = [angular_velocity_cov, 0, 0,
                                                     0, angular_velocity_cov, 0, 
                                                     0, 0, angular_velocity_cov]

        self.imu_data.linear_acceleration_covariance = [linear_acceleration_cov, 0, 0, 
                                                        0, linear_acceleration_cov, 0, 
                                                        0, 0, linear_acceleration_cov]

        # Set up the imu data message
        self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=1)

        # Set up the magnometer message
        self.mag_data = MagneticField()
        self.mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=1)


    def gather_and_publish(self):
        """ 
        Method that runs while ROS is up. It fetches, then processes the data
        from the IMU. It only publishes the data when there are subscribers 
        """
        
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if (now.to_sec() - self.imu_data.header.stamp.to_sec()) * self.throttle_rate < 0.1:
                # Ignore data at this rate (ok for a boat)
                return
            else:
                # TODO: 05/19/21 - JEV - Read the IMU data here

            self.imu_data.header.stamp = now
                    
            # If someone is subscribed to these messages, then process the data and
            # publish it. This avoids spending CPU cycles to process data no other
            # nodes need
            if self.imu_pub.get_num_connections() != 0:

                # convert to radians from degrees
                # again note NED to ENU conversion (negative signs on y and z)
                # 05/19/21 - JEV - Is the navio NED or ENU?
                self.imu_data.angular_velocity.x = np.deg2rad()
                self.imu_data.angular_velocity.y = np.deg2rad(-)
                self.imu_data.angular_velocity.z = np.deg2rad(-) 
            

                # 05/19/21 - JEV - Is the navio NED or ENU? 
                (negative signs on y and z)
                self.imu_data.linear_acceleration.x = 
                self.imu_data.linear_acceleration.y = -
                self.imu_data.linear_acceleration.z = -

                self.imu_pub.publish(self.imu_data)

            # If someone is subscribed to these messages, then process the data and
            # publish it. This avoids spending CPU cycles to process data no other
            # nodes need
            if self.mag_pub.get_num_connections() != 0:
                self.mag_data.header = self.imu_data.header
                self.mag_data.vector.x = #self.driver.state['mag_proc_y']  
                self.mag_data.vector.y = #self.driver.state['mag_proc_x']  
                self.mag_data.vector.z = #-self.driver.state['mag_proc_z']

                self.mag_pub.publish(self.mag_data)


if __name__ == "__main__":
    imu_node = NavioIMUNode()
    imu_node.gather_and_publish()
    