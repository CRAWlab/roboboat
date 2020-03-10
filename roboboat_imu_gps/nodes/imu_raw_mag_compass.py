#!/usr/bin/env python
import roslib; roslib.load_manifest("imu_um6")
import rospy
import time
from geometry_msgs.msg import Vector3Stamped, Quaternion
from sensor_msgs.msg import Imu
from math import atan2, pi
from numpy import mean, zeros, array, any, nditer
import tf

class ImuRawMagCompassNode:
    def __init__(self):
        rospy.init_node('imu_raw_mag_compass')

        self.mag_zero_x = rospy.get_param('~mag_zero_x')
        self.mag_zero_y = rospy.get_param('~mag_zero_y')
        self.mag_zero_z = rospy.get_param('~mag_zero_z')
        #self.mag_zero_radius = rospy.get_param('~mag_zero_radius')
        self.max_age = rospy.Duration.from_sec(rospy.get_param('~max_age_seconds', 0.11))

        self.imu_pub = rospy.Publisher("imu/data_compass", Imu)
        self.compass_pub = rospy.Publisher("imu/compass", Vector3Stamped)
        self.compass_msg = Vector3Stamped()

        rospy.Subscriber("imu/mag", Vector3Stamped, self._mag_cb)
        rospy.Subscriber("imu/data", Imu, self._imu_cb)

        self.yaw_vals = zeros(rospy.get_param('~smoothing_samples', 10))
        self.yaw_index = 0

    def _mag_cb(self, data):
        data.vector.x -= self.mag_zero_x
        data.vector.y -= self.mag_zero_y
        data.vector.z -= self.mag_zero_z

        # Fixed for now. Later may determine up-vector from accelerometer.
        self.yaw_vals[self.yaw_index] = -atan2(data.vector.y, -data.vector.x)
        self.yaw_index += 1
        if self.yaw_index >= len(self.yaw_vals):
          self.yaw_index = 0
       
        # Detect wraparound condition.
        # print any(self.yaw_vals > 1.5), any(self.yaw_vals < -1.5), self.yaw_vals
        if any(self.yaw_vals > 1.5) and any(self.yaw_vals < -1.5):
          yaw = mean([(x + 2*pi if x < -1.5 else x) for x in self.yaw_vals])
          if yaw > pi:
            yaw -= 2*pi
        else:
          yaw = mean(self.yaw_vals)

        self.compass_msg.vector.z = yaw 
        self.compass_msg.header.stamp = data.header.stamp
        self.compass_msg.header.frame_id = "imu_link"
        self.compass_pub.publish(self.compass_msg)

    def _imu_cb(self, data):
        age = data.header.stamp - self.compass_msg.header.stamp
        if age > self.max_age:
            return

        # Retain roll and pitch from original message, but insert pure mag yaw value.
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([getattr(data.orientation, f) for f in data.orientation.__slots__])
        orient = Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, self.compass_msg.vector.z));
        data.orientation = orient

        self.imu_pub.publish(data)
        
    def spin(self):
        rospy.spin()

        
if __name__ == '__main__':
    ImuRawMagCompassNode().spin()

