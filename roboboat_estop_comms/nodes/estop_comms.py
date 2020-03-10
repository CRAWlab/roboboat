#! /usr/bin/env python

###############################################################################
# estop_comms.py
#
# Node to communicate the status of the boat to the LED-strip microncontroller. 
# It will also read status messages from the microcontroller. 
# The microcontroller should be connected to the computer running this node via
# a serial port (very likely through a USB<->Serial converter).
#
# The possible (as of 06/17/19) colors to communicate to the LED microcontroller
# are green, yellow, blue, white, and red. It expects the data to be a 
# lowercase string matching the color name and terminated by a 
# carriage return and new line (\r\n)
#
# The micrcontroller will send a message
#
#
# Created: 06/17/19
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * 
#
# TODO:
#   * 06/18/19 - Change port, bps, etc to rosparameters
###############################################################################

import serial

# ROS related imports
import rospy
from std_msgs.msg import String

class LED_status(object):
    """ 
    Class to encapsulate the code to read and write LED and E-stop status 
    information to and from the LoRa-based E-stop system's microcontroller
    """
    
    def __init__(self, port, bps=115200, timeout=0.1):
        # Open the serial port
        self.ser = serial.Serial(port, bps, timeout=timeout)

        # Initialize the node
        rospy.init_node('LED_status', anonymous=True)
    
        # Set up the ROS subscriber for mode and publisher for mode
        # The publisher is latched - 
        self.mode_pub = rospy.Publisher('/mode', String, queue_size=1, latch=True)
        mode_sub = rospy.Subscriber("/mode", String, self.process_mode_message)

        self.LED_color = 'yellow'
        
        # Rate at which we'll check for data from the microcontroller (Hz)
        self.rate = rospy.Rate(10) 


    def process_mode_message(self, status_message):
        """ 
        Callback function for the message from the boat on the /mode
        topic. You should not need to call this directly. It gets called each 
        time a message is received.

        We'll use it to report set the color that we should change the LED to
    
        Arguments:
          status_message : the string message received
      
        Returns:
            True is successfully processed
            False if not
        """ 
    
        print(status_message.data.upper())
        # We convert to all uppercase to make it case insensitive
        if status_message.data.upper() == 'REMOTE':
            self.LED_color = 'yellow'
    
        elif status_message.data.upper() == 'AUTONOMOUS':
            self.LED_color = 'green'
        
        elif status_message.data.upper() == 'STOPPED':
            self.LED_color = 'red'
        
        # terminate the string with a newline
        LED_color_string = '{}\r\n'.format(self.LED_color).encode('utf-8')
        
        # Then, send it to the microcontroller
        self.ser.write(LED_color_string)
        

    def indefinite_loop_through_status(self):
        try:
            while (True):
                # We're assuming here that we get properly terminated lines 
                # over the serial connection. If we are not, either because they
                # are not properly terminated or we aren't receiving any data at
                # all, then this call will block indefinitely.
                line = self.ser.readline()
                line = line.decode('utf-8')
            
                # Check the line we get. If we get a message that we are e-stopped,
                # then we report that as the mode. Other nodes will handle the 
                # reporting of teleop or autonomous
                if line.upper() == '$ESTOP,STOPPED':
                    rospy.loginfo('Microcontroller is E-stopped')
                    self.mode_pub.pulish('STOPPED')
                
                elif line.upper() == '$ESTOP,OK':
                    rospy.logdebug('Got $ESTOP,OK from microcontroller')
                
#                 else:
#                     rospy.loginfo('Got {} from microcontroller'.format(line))

                self.rate.sleep()

        except (KeyboardInterrupt, SystemExit):
            rospy.loginfo('Quit e-stop comms.')
            raise
        
        finally:
            rospy.logwarn('Communication with LoRa controller stopped.')
            
            # Close the serial port
            self.ser.close()
                


if __name__ == "__main__":
    # Define the serial port setup
    # NOTE: Serial port will have to change based on configuration
    PORT = '/dev/ttyACM0'

    # define the serial communication parameters, 8 bits, no parity, 1 stop bit
    BPS = 115200

    # Define a Timeout for serial communication
    TIMEOUT = 0.01
    
    status_processing = LED_status(PORT, bps=BPS, timeout=TIMEOUT)
    status_processing.indefinite_loop_through_status()
