#! /usr/bin/env python

###############################################################################
# imu_um7_node.py
#
# Node to publish IMU data from the CH Robotics UM7 IMU with a GPS attached.
#
# Work was guided by the C++-based ros-driver UM7 package:
#    https://github.com/ros-drivers/um7
#    https://wiki.ros.org/um7
#
# the nmea_navsat_driver:
#    https://wiki.ros.org/nmea_navsat_driver 
#
# and the (depricated) Clearpath UM6 package:
#    https://github.com/clearpathrobotics/imu_um6
#
#
# Created: 05/23/19
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * 
#
# TODO:
#   * 05/23/19 - JEV - Add reset service processing, etc
#   * 05/23/19 - JEV - Verify the mapping from NED of IMU to ENU for ROS
#   * 05/23/19 - JEV - Estimate covariance from HDOP?
#   * 05/23/19 - JEV - Should be publish NaN or 0 if we don't have a GPS fix?
###############################################################################


import roslib
roslib.load_manifest('imu_um7')

import rospy
import numpy as np
import tf
import struct
from time import sleep
from serial import SerialException

import um7

# Import the ROS message types we need
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3Stamped

# Use the ROS quaternion transformations
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_about_axis
from tf.transformations import quaternion_from_euler

GRAVITY = 9.81

class ImuUm7Node(object):
    def __init__(self, default_port='/dev/ttyUSB1'):
        """
        @param default_port: default serial port to use for
            establishing a connection to the UM7 IMU sensor.
            This will be overridden by ~port param if 
            available.
        """
        rospy.init_node('imu_um7')

        self.port = rospy.get_param('~port', default_port)
        self.frame_id = rospy.get_param('~frame_id', "/imu")
        self.gps_frame_id = rospy.get_param('~gps_frame_id', "/gps")
        self.throttle_rate = rospy.get_param('~throttle_rate', 10000)
        self.reset_mag = rospy.get_param('~reset_mag', False)
        self.reset_accel = rospy.get_param('~reset_accel', False)
        self.mag_zero_x = rospy.get_param('~mag_zero_x', False)
        self.mag_zero_y = rospy.get_param('~mag_zero_y', False)
        self.mag_zero_z = rospy.get_param('~mag_zero_z', False)
        rospy.loginfo("serial port: %s"%(self.port))
        
        self.link = rospy.get_param('~link', 'imu_link')
        rospy.loginfo("tf link: {}".format(self.link))

        self.imu_data = Imu()
        self.imu_data = Imu(header=rospy.Header(frame_id=self.link))
        
        # These covariance calculations are based on those bound in the existing
        # UM7 ROS package at:
        #   https://github.com/ros-drivers/um7
        linear_acceleration_stdev = rospy.get_param("~linear_acceleration_stdev", 4.0 * 1e-3 * 9.80665)
        angular_velocity_stdev = rospy.get_param("~angular_velocity_stdev", np.deg2rad(0.06))

        linear_acceleration_cov = linear_acceleration_stdev * linear_acceleration_stdev
        angular_velocity_cov = angular_velocity_stdev * angular_velocity_stdev

        # From the UM7 datasheet for the dynamic accuracy from the EKF.
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
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=1)

        # Set up the Roll-Pitch-Yaw (rpy) message
        self.rpy_data = Vector3Stamped()
        self.rpy_pub = rospy.Publisher('imu/rpy', Vector3Stamped, queue_size=1)

        # Set up the magnometer message
        self.mag_data = Vector3Stamped()
        self.mag_pub = rospy.Publisher('imu/mag', Vector3Stamped, queue_size=1)

        # what data to get from the UM7
        self.statevars = ['health', 
                          'roll', 'pitch', 'yaw', 
                          'mag_proc_x', 'mag_proc_y', 'mag_proc_z', 
                          'accel_proc_x', 'accel_proc_y', 'accel_proc_z', 
                          'gyro_proc_x', 'gyro_proc_y', 'gyro_proc_z', 
                          'quat_a', 'quat_b', 'quat_c', 'quat_d',
                          'gps_latitude', 'gps_longitude', 'gps_altitude']
        
        # Masks for parsing out the health data from the IMU
        self.NUM_SATS_USED =    0b11111100000000000000000000000000
        self.HDOP =             0b00000011111111110000000000000000 # Not used as of 05/23/19
        self.NUM_SATS_IN_VIEW = 0b00000000000000001111110000000000
        
         
        # Set up the GPS NavSatFix publisher
        self.fix_data = NavSatFix()
        self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)

        imu_connected = False
        
        while not rospy.is_shutdown() and not imu_connected:
            try:
                self.driver = um7.UM7('s', self.port, self.statevars, baud=115200)
                imu_connected = True
                rospy.loginfo("Imu initialization completed")

                self.received = -1

                if self.reset_mag:
                    self.driver.set_mag_reference()

                if self.reset_accel:
                    cmd_seq.append(Um6Drv.CMD_SET_ACCEL_REF)

                if self.mag_zero_x and self.mag_zero_y and self.mag_zero_z:
                    rospy.loginfo("Magnetometer calibration: %.3f %.3f %.3f",
                                  self.mag_zero_x, self.mag_zero_y, self.mag_zero_z)
                                  

            except SerialException:
                rospy.logwarn("Serial error communicating with IMU. Will retry in 2.0 seconds.")
                rospy.sleep(2.0)


#     def um6_cmd_cb(self, cmd, result):
#         if (cmd == Um6Drv.UM6_COMMUNICATION):
#             rospy.loginfo("Set quaternion output: %s"%(result))
#         if (cmd == Um6Drv.UM6_MISC):
#             rospy.loginfo("Configured EKF: %s"%(result))
#         if (cmd == Um6Drv.CMD_RESET_EKF):
#             rospy.loginfo("Reset EKF: %s"%(result))
#         if (cmd == Um6Drv.CMD_ZERO_GYROS):
#             rospy.loginfo("Zero Gyros: %s"%(result))
#         if (cmd == Um6Drv.CMD_SET_MAG_REF):
#             rospy.loginfo("Set Magnetometer Reference: %s"%(result))
#         if (cmd == Um6Drv.CMD_SET_ACCEL_REF):
#             rospy.loginfo("Set Accelerometer Reference: %s"%(result))
#         if result:
#             self.received = cmd
# 
#     def reset_service_cb(self,req):
#         cmd_seq = []
#         if req.zero_gyros:
#             cmd_seq.append(Um6Drv.CMD_ZERO_GYROS)
#         if req.reset_ekf:
#             cmd_seq.append(Um6Drv.CMD_RESET_EKF)
#         if req.set_mag_ref:
#             cmd_seq.append(Um6Drv.CMD_SET_MAG_REF)
#         if req.set_accel_ref:
#             cmd_seq.append(Um6Drv.CMD_SET_ACCEL_REF)
#         self.received = -1
#         while (not rospy.is_shutdown()) and (len(cmd_seq)>0):
#             cmd = cmd_seq[0]
#             self.driver.sendCommand(cmd, self.um6_cmd_cb);
#             start = rospy.Time.now()
#             while (rospy.Time.now() - start).to_sec() < 0.5:
#                 rospy.sleep(0.01)
#                 if self.received == cmd:
#                     break
#             if self.received == cmd:
#                 self.received = -1
#                 cmd_seq = cmd_seq[1:]
#         rospy.loginfo("Imu initialisation completed")
#         return ResetResponse()


    def gather_and_publish(self):
        """ 
        Method that runs while ROS is up. It fetches, then processes the data
        from the IMU. It only publishes the data when there are subscribers 
        """
        
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if (now.to_sec() - self.imu_data.header.stamp.to_sec())*self.throttle_rate < 1.0:
                # Ignore data at this rate (ok for a boat)
                return
            else:
                self.driver.catchallsamples(self.statevars, 0.1)

            self.imu_data.header.stamp = now
                    
            # If someone is subscribed to these messages, then process the data and
            # publish it. This avoids spending CPU cycles to process data no other
            # nodes need
            if self.imu_pub.get_num_connections() != 0:
                self.imu_data.orientation = Quaternion()
            
                # IMU outputs [w,x,y,z] NED, convert to [w, x, -y, -z] ENU in world frame
                self.imu_data.orientation.w = self.driver.state['quat_a'] 
                self.imu_data.orientation.x = self.driver.state['quat_b'] 
                self.imu_data.orientation.y = -self.driver.state['quat_c']
                self.imu_data.orientation.z = -self.driver.state['quat_d']

                # convert to radians from degrees and scale according to values 
                # in the UM7 datasheet
                # again note NED to ENU conversion
                self.imu_data.angular_velocity.x = np.deg2rad(self.driver.state['roll_rate'] / 16)
                self.imu_data.angular_velocity.y = np.deg2rad(-self.driver.state['pitch_rate'] / 16)
                self.imu_data.angular_velocity.z = np.deg2rad(-self.driver.state['yaw_rate'] / 16)
            
                # again note NED to ENU conversion
                self.imu_data.linear_acceleration.x = self.driver.state['accel_proc_x'] * GRAVITY  # data['DATA_LINEAR_ACCEL'][1]
                self.imu_data.linear_acceleration.y = -self.driver.state['accel_proc_y'] * GRAVITY   # data['DATA_LINEAR_ACCEL'][0]
                self.imu_data.linear_acceleration.z = -self.driver.state['accel_proc_z'] * GRAVITY  # -(data['DATA_LINEAR_ACCEL'][2])

                self.imu_pub.publish(self.imu_data)

            # If someone is subscribed to these messages, then process the data and
            # publish it. This avoids spending CPU cycles to process data no other
            # nodes need
            if self.rpy_pub.get_num_connections() != 0:
                self.rpy_data.header = self.imu_data.header
                self.rpy_data.vector.x = np.deg2rad(self.driver.state['roll'])
                self.rpy_data.vector.y = np.deg2rad(-self.driver.state['pitch'])
                self.rpy_data.vector.z = np.deg2rad(-self.driver.state['yaw'])

                self.rpy_pub.publish(self.rpy_data)

            # If someone is subscribed to these messages, then process the data and
            # publish it. This avoids spending CPU cycles to process data no other
            # nodes need
            if self.mag_pub.get_num_connections() != 0:
                self.mag_data.header = self.imu_data.header
                self.mag_data.vector.x = self.driver.state['mag_proc_y']  
                self.mag_data.vector.y = self.driver.state['mag_proc_x']  
                self.mag_data.vector.z = -self.driver.state['mag_proc_z']

                self.mag_pub.publish(self.mag_data)

            # Now, gather and process the GPS data
            # If someone is subscribed to these messages, then process the data and
            # publish it. This avoids spending CPU cycles to process data no other
            # nodes need
            if self.fix_pub.get_num_connections() != 0:
                self.fix_data.header.stamp = rospy.get_rostime()
                self.fix_data.header.frame_id = self.gps_frame_id
            
                self.fix_data.status.service = NavSatStatus.SERVICE_GPS

                # Mask the returned health register to determine the number of
                # satellites in use.
                self.number_of_sats_used = (self.NUM_SATS_USED & self.driver.state['health']) >> 26
                self.number_of_sats_inView = (self.NUM_SATS_IN_VIEW & self.driver.state['health']) >> 10
            
                # GPS needs at least 4 satellites to have a reliable fix
                # So, if we don't have 4, we publish that we don't have a fix
                if self.number_of_sats_used < 4:
                    self.fix_data.status.status = NavSatStatus.STATUS_NO_FIX
                    rospy.logwarn("No GPS fix.")
                else:
                    self.fix_data.status.status = NavSatStatus.STATUS_FIX
            
                # Publish the GPS data
                # TODO: 05/23/19 - JEV - Should be publish NaN or 0 if we don't
                #                        have a fix?
                self.fix_data.latitude = self.driver.state['gps_latitude']
                self.fix_data.longitude = self.driver.state['gps_longitude'] 
                self.fix_data.altitude = self.driver.state['gps_altitude']
                self.fix_data.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            
                self.fix_pub.publish(self.fix_data)


if __name__ == '__main__':
    node = ImuUm7Node()
    node.gather_and_publish()