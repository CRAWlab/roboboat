#! /usr/bin/env python

###############################################################################
# roboboat_navio_gps.py
#
# Package that provides uses the Navio2 Python API to read the GPS and publish
# the information on standard ROS messages
#
# Created: 05/13/21
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
from sensor_msgs.msg import NavSatFix, NavSatStatus

class NavioGPSNode(object):
    
    def __init__(self):
        rospy.init_node('navio_gps')
        self.gps_frame_id = rospy.get_param('~gps_frame_id', "/gps")
        
        # Set up the GPS NavSatFix publisher
        self.fix_data = NavSatFix()
        self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)

    def gather_and_publish(self):
        """ 
        Method that runs while ROS is up. It fetches, then processes the data
        from the IMU. It only publishes the data when there are subscribers 
        """
        
        while not rospy.is_shutdown():
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
                self.number_of_sats_used = #
                self.number_of_sats_inView = #
            
                # GPS needs at least 4 satellites to have a reliable fix
                # So, if we don't have 4, we publish that we don't have a fix
                if self.number_of_sats_used < 4:
                    self.fix_data.status.status = NavSatStatus.STATUS_NO_FIX
                    rospy.logwarn("No GPS fix.")
                else:
                    self.fix_data.status.status = NavSatStatus.STATUS_FIX
            
                # Publish the GPS data
                # TODO: 05/19/21 - JEV - Should be publish NaN or 0 if we don't
                #                        have a fix?
                self.fix_data.latitude = #
                self.fix_data.longitude = #
                self.fix_data.altitude = #
                self.fix_data.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            
                self.fix_pub.publish(self.fix_data)


if __name__ == "__main__":
    gps_node = NavioGPSNode()
    gps_node.gather_and_publish()