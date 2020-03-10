#! /usr/bin/env python

###############################################################################
# TD_TCP_statusServer.py
#
# Script to make a TCP connection with the Technical Director's (TD) network for 
# the 2019 International RoboBoat Competition. It should be run as a node on 
# a shore-based computer running ROS on the same network as the TD server. 
# 
# The protocol is based on the NMEA sentences described at:
#   https://robonationforum.vbulletin.net/filedata/fetch?id=2030
#
# NOTE: Any plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 06/18/16
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   *
#
###############################################################################


# Imports for "pure" TCP communcation with TD server
import socket
import SocketServer
import threading

# ROS related imports
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix

# datetime import to use the computer's time as that reported
# Make sure it is on EST to meet the protocol requirements
import datetime

# Change this address and port to match those provided by the technical directors
# TODO: 03/10/20 - JEV - Change these to ros parameters
TD_IP_ADDRESS = 'ROBOT.SERVER'
TD_PORT = 9000


class ThreadedTCPRequestHandler(SocketServer.StreamRequestHandler):
    """
    The RequestHandler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        # self.rfile is a file-like object created by the handler;
        # we can now use e.g. readline() instead of raw recv() calls
        self.data = self.rfile.readline().strip()
        print("{} wrote:".format(self.client_address[0]))
        print(self.data)
        # Likewise, self.wfile is a file-like object used to write back
        # to the client
        #self.wfile.write(self.data.upper())

class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    pass


class TD_communication(object):
    """ 
    Class to handled forming and sending of status packets 
    """

    def __init__(self, td_ip_address, td_port):
        """
        Initialization function
        
        Arguments:
          ip_address : the IP address to send the data, as a string
          port : the port to send the data to, as an integer
        """

        self.td_ip_address = td_ip_address
        self.td_port = td_port
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Data to be included in messages. 
        # All initialized as either an empty string or to safest default value
        # TODO: 06/14/19 - JEV - should we change the GPS to empty too?
        self.latitude = 29.151098    # Center of the pond according to Google Maps
        self.longitude = -81.016561  # Center of the pond according to Google Maps
        self.NS = 'N'                # We'll always be in the Northern hemisphere
        self.EW = 'W'                # We'll always be in the Western hemisphere
        self.TEAM_ID = "ULL"         # Update this to reflect ID given by TDs
        self.mode = 1               # 1=Remote, 2=Autonomous, 3=E-stop
        self.dock_number = ""        # Number of dock identified
        self.flag_number = ""        # Number of flag seen
        
        # Initialize the node
        rospy.init_node('td_status_sender', anonymous=True)
        
        # Set up the fix subscriber and register the callback
        rospy.Subscriber("/fix", NavSatFix, self.process_fix_message)
        
        # Set up the mode subscriber and register the callback
        rospy.Subscriber("/mode", String, self.process_mode_message)
        
        # Set up the dock subscriber and register the callback
        rospy.Subscriber("/dock", Int32, self.process_dock_message)
        
        # Set up the flag subscriber and register the callback
        rospy.Subscriber("/flag", Int32, self.process_flag_message)

    def process_fix_message(self, fix_message):
        """ 
        Callback function for the NavSatFIX message from the GPS on the /fix
        topic. You should not need to call this directly. It gets called each 
        time a message is received.
        
        If we have a fix, we save the latitude and longitude. Otherwise, we
        fill those two values with None

        We'll use it to report the necessary location information in the 
        heartbeat message
        
        Arguments:
          fix_message : the NavSatFix message received
          
        Returns:
            Nothing. Data is just saved in instance variables
        """
        
        # We have a valid fix if the status is >= 0 
        # From:
        #   http://docs.ros.org/api/sensor_msgs/html/msg/NavSatStatus.html

        if fix_message.status >= 0:
            # TODO: To use this more generally, we would need to look for the sign
            #       of the latitude returned and determine N/S from it
            self.latitude = fix_message.latitude
        
            # Negative sign to correct for W
            # TODO: To use this more generally, we would need to look for the sign
            #       fo the longitude returned and determine E/W from it
            self.longitude = -fix_message.longitude 

        else: # We don't have a fix so fill the parameters with an empty string
            self.latitude = ""
            self.longitude = ""


    def process_mode_message(self, mode_message):
        """ 
        Callback function for the message from the boat on the /mode
        topic. You should not need to call this directly. It gets called each 
        time a message is received.

        We'll use it to report the mode in the heartbeat message and the data
        require for each message
        
        Arguments:
          mode_message : the string message received 
          
        Returns:
            True is successfully processed
            False if not
        """ 
        
        # Read the message and convert it to the integer representation 
        # we need for the TD protocol, 
        #    0 = remote
        #    1 = autunomous
        #    2 = E-stopped
        #
        # We convert to all uppercase to make it case insensitive
        if mode_message.upper() == 'REMOTE':
            self.mode = 0
        
        elif mode_message.upper() == 'AUTONOMOUS':
            self.mode = 1
            
        elif mode_message.upper() == 'STOPPED':
            self.mode = 2
            
        else:
            # If we don't match a known mode type, then return false to 
            # indicate we didn't get a proper message
            return False
            
        return True


    def process_dock_message(self, dock_message):
        """ 
        Callback function for the message from the boat on the /dock
        topic. You should not need to call this directly. It gets called each 
        time a message is received.

        We'll use it to report the dock in the message
        
        Arguments:
          dock_message : the message received, should be an Int32 ros message
          
        Returns:
            True is successfully processed and sent
            False if not
        """ 
        
        try: 
            self.dock = dock_message
            
        except:
            # If we cant process the dock number, then return false to 
            # indicate we didn't get a proper message
            return False
            
        # Now, send the dock message
        self.send_dock_message()
        
        # TODO: Add generating and sending the message. since its infrequent snding here is okay
        return True


    def process_flag_message(self, flag_message):
        """ 
        Callback function for the message from the boat on the /flag
        topic. You should not need to call this directly. It gets called each 
        time a message is received.

        We'll use it to report the dock in the message
        
        Arguments:
          flag_message : the message received, should be an Int32 ros message
          
        Returns:
            True is successfully processed
            False if not
        """ 
        
        try: 
            self.flag = flag_message
            
        except:
            # If we cant process the flag number, then return false to 
            # indicate we didn't get a proper message
            return False
        
        # Now, send the flag message
        self.send_flag_message()

        # TODO: Add generating and sending the message. since its infrequent snding here is okay
        return True


    def send_data_and_wait(self, message):
        # TODO: 06/14/19 - JEV - Should we just stay connected and manage 
        #                        disconnecting at the class level rather than
        #                        connecting and disconnecting  each time this 
        #                        method is called?
        self.sock.connect((self.td_ip_address, self.td_port))

        try:
            self.sock.sendall(message)
            rospy.loginfo('Sending to {}:{} \t Message: {}'.format(self.td_ip_address, 
                                                                   self.td_ip_address, 
                                                                   message))

            # Now, wait for the acknowledge message
            # TODO: Do we even need to worry about this outside of debugging?
            response = sock.recv(1024)
            rospy.loginfo("Received: {}".format(response))
            
            # Check the return message for success
            if (response.split(',')[3].startswith('Success')):
                rospy.loginfo("Message acknolwedged by server.")
            else:
                error_message = response.split(',')[4]
                rospy.logwarn("Server Reported Error: {}".format(error_message))

        except (socket.error):
            rospy.logerror("Could not send data to TD server.")
            return False

        finally:
            self.sock.close()
        
        return True


    def send_heartbeat_message(self):
        """ Generates, then sends a heartbeat message """

        message = self.generate_heartbeat_message()

        self.send_data_and_wait(message)


    def send_dock_message(self):
        """ Generates, then sends a dock message """

        message = self.generate_dock_message()

        self.send_data_and_wait(message)


    def send_flag_message(self):
        """ Generates, then sends a flag message """

        message = self.generate_flag_message()

        self.send_data_and_wait(message)


    def calculate_checksum(self, message):
        """ 
        Calculate the checksum based on the: 

            the bitwise exclusive OR of ASCII codes 
            of all characters between the $ and *
        
        Arguments:
            message: the message to calculate the checksum for
        
        Returns:
            A string containing the hexidecimal representation of checksum
        """
        checksum = 0
        
        for character in message:
            checksum ^= ord(character)
        
        return "{:2x}".format(checksum)


    def generate_heartbeat_message(self):
        """ 
        Generates a heartbeat message. This message should be sent at 1Hz
        
        The message contains these elements in a comma separated form:
        
          $RDHRB    |  protocol header
          ddmmyy    |  date in EST
          hhmmss    |  24 hour formatted time in EST
          lat       |  Latitude in decimal degrees
          N/S       |  North/South indicator for latitude
          long      |  longitude in decimal degrees
          E/W       |  East/West indicator for longitude
          TEAMID    |  5 character team ID assgined to us
          MODE      |  System mode 1=Remote, 2=Autonomous, 3=E-stop
          *         |  Indicate end of message
          Checksum  |  Bitwise XOR of message between $ and *
          \r\n      |  Carriage returna and linefeed end message
          
          
        Example Messages: 
            $RBHRB,101218,161229,21.31198,N,157.88972,W,AUVSI,2*01\r\n
            $RBHRB,101218,161230,21.31198,N,157.88972,W,AUVSI,2*09\r\n

        Arguments:
          None, it operates on instance variables processed in callbacks from
          the ROS topic messages
          
        Returns:
          The message to send as a string
        """
        
        now = datetime.datetime.now()
        
        header = "RBHRB"
        date = now.strftime("%m%d%y")
        time = now.strftime("%H%M%S")
        lat = "{},{}".format(self.latitude, self.NS)
        long = "{},{}".format(self.longitude, self.EW) 
        ID = self.TEAM_ID
        mode = "{:d}".format(self.mode)
        
        core_message = "{},{},{},{},{},{},{}".format(header,
                                                     date,
                                                     time,
                                                     lat,
                                                     long,
                                                     ID,
                                                     mode)
        
        checksum = self.calculate_checksum(core_message)
        
        message = "${}*{}\r\n".format(core_message, checksum)
        
        return message


    def generate_dock_message(self):
        """ 
        Generates a Identify the Dock message. This message should be sent 
        during identify the dock challenge, reporting the dock in which the 
        acoustic pinger is located
        
        The message contains these elements in a comma separated form: 
        
          $RDDOK    |  protocol header
          ddmmyy    |  date in EST
          hhmmss    |  24 hour formatted time in EST
          TEAMID    |  5 character team ID assgined to us
          DOCKNUM   |  Number of the dock with the pinger
          *         |  Indicate end of message
          Checksum  |  Bitwise XOR of message between $ and *
          \r\n      |  Carriage returna and linefeed end message
          
        Example Message:
            $RBDOK,101218,161229,AUVSI,2*3E\r\n
            $RBDOK,101218,161231,AUVSI,2*37\r\n
        
        Arguments:
          None, it operates on instance variables processed in callbacks from
          the ROS topic messages
          
        Returns:
          The message to send as a string
        """
        
        now = datetime.datetime.now()
        
        header = "RBDOK"
        date = now.strftime("%m%d%y")
        time = now.strftime("%H%M%S")
        ID = self.TEAM_ID
        dock_num = self.dock_number
        
        core_message = "{},{},{},{},{}".format(header,
                                               date,
                                               time,
                                               ID, 
                                               dock_num)
                                               
        checksum = self.calculate_checksum(core_message)
        
        message = "${}*{}\r\n".format(core_message, checksum)
        
        return message
    
    
    def generate_flag_message(self):
        """ 
        Generates a Identify Flag message. This message should be sent 
        during identify the flag challenge, reporting the flag number
        
        The message contains these elements in a comma separated form: 
        
          $RBFLG    |  protocol header
          ddmmyy    |  date in EST
          hhmmss    |  24 hour formatted time in EST
          TEAMID    |  5 character team ID assgined to us
          FLAGNUM   |  Number of the flag
          *         |  Indicate end of message
          Checksum  |  Bitwise XOR of message between $ and *
          \r\n      |  Carriage returna and linefeed end message
          
        Example Message:
            $RBFLG,101218,161229,AUVSI,3*32\r\n
            $RBFLG,101218,161232,AUVSI,3*38\r\n
            
        Arguments:
          None, it operates on instance variables processed in callbacks from
          the ROS topic messages
          
        Returns:
          The message to send as a string
        """
    
        now = datetime.datetime.now()
        
        header = "RBFLG"
        date = now.strftime("%m%d%y")
        time = now.strftime("%H%M%S")
        ID = self.TEAM_ID
        flag_num = self.flag_number
        
        core_message = "{},{},{},{},{}".format(header,
                                               date,
                                               time,
                                               ID, 
                                               flag_num)
                                               
        checksum = self.calculate_checksum(core_message)
        
        message = "${}*{}\r\n".format(core_message, checksum)
        
        return message



if __name__ == "__main__":
    # Depending the setup of your computer and the network, you may need to 
    # change the IP address and port below
    # Port 0 means to select an arbitrary unused port
    # TODO: 03/10/20 - JEV - Change these to ROS parameters
    LOCAL_IP, LOCAL_PORT = "0.0.0.0", 0

    server = ThreadedTCPServer((LOCAL_IP, LOCAL_PORT), ThreadedTCPRequestHandler)
    ip, port = server.server_address

    # Start a thread with the server -- that thread will then start one
    # more thread for each request
    server_thread = threading.Thread(target=server.serve_forever)
    # Exit the server thread when the main thread terminates
    server_thread.daemon = True
    server_thread.start()
    rospy.logdebug("TD server communication loop running in thread:", server_thread.name)

    
    # Now create and instance of the TD_communication class to use to manage
    # the messages
    TD_comm = TD_communication(TD_IP_ADDRESS, TD_PORT)
    
    rate = rospy.Rate(1) # We'll run the heartbeat sender at 1Hz

    try:
        while not rospy.is_shutdown():
            TD_comm.send_heartbeat_message()
            
            rate.sleep()

    except (KeyboardInterrupt, SystemExit):
        rospy.logerror("TD communication node exited.")
        server.shutdown()
