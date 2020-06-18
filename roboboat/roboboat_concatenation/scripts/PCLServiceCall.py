#!/usr/bin/env python
import roslib; roslib.load_manifest('laser_assembler')
import rospy; from laser_assembler.srv import *

from sensor_msgs.msg import PointCloud, PointCloud2

# Initialize the node
rospy.init_node("CombinedPCLPub")
rospy.wait_for_service("assemble_scans")

# How often do we want to call the service?
rate = rospy.Rate(10) # (Hz)

point_pub = rospy.Publisher("/pcl/combined", PointCloud, queue_size=1)

PCL_msg = PointCloud()
PCL2_msg = PointCloud2()

# While roscore is active:
while not rospy.is_shutdown():

  try:
    assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
    resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())

    #print "Got cloud with %u points" % len(resp.cloud.points)

    PCL_msg.header = resp.cloud.header
    PCL_msg.points = resp.cloud.points
    PCL_msg.channels = resp.cloud.channels

    # Publish the point cloud
    point_pub.publish(PCL_msg)

  except rospy.ServiceException, e:
    print "Service call to PCL_concatenation failed: %s"%e

  rate.sleep()