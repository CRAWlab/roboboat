#!/usr/bin/env python
import roslib; roslib.load_manifest("imu_um6")
import rospy
import time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu 

class ImuPoseNode:

    def __init__(self):
        rospy.init_node('imu_pose')
        self.pub = rospy.Publisher('imu_pose', PoseStamped)
        self.sub = rospy.Subscriber("imu_data", Imu, self.handle_imu)
        
        self.link = rospy.get_param('~link', 'imu_link')
        
        rospy.spin()

    def handle_imu(self, imu_data):
    
        if self.pub.get_num_connections() != 0:
    
          self.pose = PoseStamped()
          self.pose.header.frame_id = self.link
          self.pose.header.stamp = rospy.Time.now()
          self.pose.pose.position.x = 0
          self.pose.pose.position.y = 0
          self.pose.pose.position.z = 0
          self.pose.pose.orientation = imu_data.orientation
          self.pub.publish(self.pose)

if __name__ == '__main__':
    poser = ImuPoseNode()
