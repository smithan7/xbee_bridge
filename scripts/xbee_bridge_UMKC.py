#!/usr/bin/env python

import rospy
import sys, termios, tty, select, os
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import serial

class XBee(object):
  initialized = False # Make sure the XBee is setup before trying to use it!
  
  def __init__(self, com_port, baud_rate, com_type, fake_agent):
    
    self.fake_agent = fake_agent # Fake agent is only used when I am NOT using the XBEE but am instead publishing to a topic in ROS instead, useful for early debuggin
    self.com_type = com_type # am I the broadcaster  (i.e., the agent transmitting their location) or receiver (i.e., the one recieving the broadcaster's location)

    if self.fake_agent: # I am a fake agent, setup publisher/subscriber instead of Xbee
        if self.com_type == 'receiver':
            self.pub_chatter = rospy.Publisher('/xbee/chatter', String, queue_size=10) # Publish over a topic instead of out over xbee
        elif self.com_type == 'broadcaster':    
            self.sub_chatter = rospy.Subscriber('/xbee/chatter', String, self.xbee_chatter_callback) # Sub to topic
        else:
            while True: # the com type was invalid
                rospy.logerr("XBee Bridge:: Improper Communication Type Recieved, no communication established: " + com_type)
                rospy.sleep(rospy.Duration(3.0))
    else: # I am using the Xbee, set it up instead
        self.ser = serial.Serial(com_port, baud_rate)#'/dev/ttyUSB0',9600)  # open serial port
        self.xbee_broadcast('hello') # write a string as a test

    if self.com_type == 'receiver': # I am the receiver, setup my publisher that forwards what my XBee recieves so I can publish it to ROS
        ### Setup publishers, in reality this is what the ground station subscribes to
        self.pub_odom = rospy.Publisher('/target_odom', NavSatFix, queue_size=10) # Location of the the broadcasting agent
        
    elif self.com_type == 'broadcaster': # I am the broadcaster, setup my subscriber that recieves messages from ROS and sends them out over the XBee
        ## Setup Subscriber, in reality, this is what the agent publishes
        self.check_xbee = rospy.Timer(rospy.Duration(0.1), self.xbee_timer_callback) # check the Xbee for messages ever X seconds ~ x = Duration specified in seconds
        self.sub_odom = rospy.Subscriber('/dji_sdk/global_position', NavSatFix, self.odom_callback) # agent location
    
    else:
        while True: # the com type was invalid
            rospy.logerr("XBee Bridge:: Improper Communication Type Recieved, no communication established: " + com_type)
            rospy.sleep(rospy.Duration(3.0))
    
  def xbee_broadcast(self, msg):
    if self.fake_agent: # Fake agent, publish to the ros topic specified
        self.pub_chatter.publish(msg)

    else: # I am broadcasting over the XBee
        try: # error handling
            msg = msg + '\n' # append End Of Line (EOL, eol) to msg
            if not self.ser.is_open: # if ser is not open, try to open it
                self.ser.open()
                if not self.ser.is_open: # Still can't open, must be a problem, print error msg
                    rospy.logerr("Xbee_Bridge:: Still unable to open Serial Port %s", self.ser.name)
                    rospy.sleep(rospy.Duration(3.0)) # wait a few so you don't spam the screen with error msgs
                    return
            
            else: # ser opened, let people know
                if not self.initialized and self.ser.is_open:
                    self.initialized = True
                    rospy.loginfo("XBee Bridge:: initialized serial port on %s", self.ser.name)

            self.ser.write(msg); # write the message over the serial port
        except: # somehow the above code broke, tell user
            rospy.logerr("XBee_Bridge::xbee_broadcast broke")


  def xbee_chatter_callback(self, msg): # I recieved a msg (msg is a string containing the msg) over the fake xbee ros topic, do something with it
    ################################
    #
    # Example message is "$l123456789,123456789\n"
    #   $ - is the start charact, tells you where the msg begins
    #   l - this is the msg identifier, implies location. Later is you want to have multiple msg types you can do it easily by changing this character
    #   123456789 - this is the latiude * 1000.0 as an int
    #   , - separate lat and lon
    #   123456789 - this is the longitude * 1000.0 as an int
    #   \n - is the eol marker
    #
    ################################

    #rospy.loginfo("chatter msg in: " + str(msg.data)) # this prints the msg, useful for debugging
    msg = str(msg.data) # ensure msg is a string
    if msg[0] == '$': # ensure it starts with header
        msg = msg[1:] # strip header from msg
        tp = msg[0] # get message type
        msg = msg[1:] # strip message type from msg

        ### Location Message
        if tp == 'l': # message type is location
            self.read_location_chatter(msg) # do something with location msg
        else: # type is not specified, give warning msg
            rospy.logwarn("XBee Bridge::Bad message type: " + tp)

  def xbee_timer_callback(self, event): # Check the xbee at specified duration for data in xbee buffer
      ################################
    #
    # Example message is "$l123456789,123456789\n"
    #   $ - is the start charact, tells you where the msg begins
    #   l - this is the msg identifier, implies location. Later is you want to have multiple msg types you can do it easily by changing this character
    #   123456789 - this is the latiude * 1000.0 as an int
    #   , - separate lat and lon
    #   123456789 - this is the longitude * 1000.0 as an int
    #   \n - is the eol marker
    #
    ################################
    try: # error handling
        if not self.ser.is_open: # if xbee isn't setup, try and set it up
            self.ser.open() # not setup, try to setup
            if not self.ser.is_open: # still not setup, error
                rospy.logerr("Xbee_Bridge:: Still unable to open Serial Port %s", self.ser.name)
                rospy.sleep(rospy.Duration(3.0))
                return
        else:
            if not self.initialized and self.ser.is_open: # first time setting it up
                self.initialized = True # mark initialized and tell user
                rospy.loginfo("XBee Bridge:: initialized serial port on %s", self.ser.name)
            
            while self.ser.in_waiting > 0: # While there is data in the XBee buffer
                while True: # Eat through data until header is found
                    data = self.ser.read() # pull the first char in the Xbee buffer
                    if data == '$': # did I see the header, this marks the start of the msg
                        data = self.ser.read() # read the next character
                        msg = '' # start an empty string that will hold the msg
                        while True: ## Search for EOL - end of line '/n', this marks the end of the msg
                            if data == "\n": # Found the eol, stop appending to msg, at this point msg contains a complete msg from $ to the char before the eol
                                break
                            else: # did not find eol, append char to msg and read next char in buffer
                                msg = msg + data
                                data = self.ser.read()
                        rospy.loginfo("XBee_Bridge:: msg: " + msg) # print out the complete msg
                        break # found a complete msg, stop search for header
                tp = msg[0] # get msg type
                msg = msg[1:]
                ### Location Message
                if tp == 'l':
                    self.read_location_chatter(msg)
                else:
                    rospy.logwarn("XBee Bridge::Bad message type: " + tp)
    except: # had an error somewhere above, don't break node, just tell user
        rospy.logerr("XBee_Bridge::xbee_timer_callback had a problem")

  def read_location_chatter(self, msg):
    try:
        lat = float(msg[0:12]) / 1000000.0 # convert back to float
        msg = msg[12:] # strip lat off msg
        #comma = msg[0]
        msg = msg[1:] # strip off comma
        lon = float(msg[0:12]) / 1000000.0 # convert back to float
        self.publish_location(lat, lon)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("Recieved bad msg: " + msg)

  def publish_location(self, lat, lon):
    # This tells the coordinator where each agent is
    msg = NavSatFix
    msg.latitude = lat
    msg.longitude = lon
    self.pub_loc.publish(msg)
    
  def broadcast_location_callback(self, msg):
    # Another ros node published to odom, broadcast it out over the XBee
    broadcast = '$l'
    broadcast = broadcast + self.set_string_length(str(int(msg.latitude * 1000000.0)), 12) + ',' # 12 digits should be enough
    broadcast = broadcast + self.set_string_length(str(int(msg.longitude * 1000000.0)), 12) # 12 digits should be enough
    self.xbee_broadcast(broadcast)

  def set_string_length(self, str_in, length):
    # This appends '0' 's to the front of the string to make it the right length
    temp_0 = length - len(str_in)
    for i in range(0, temp_0):
       str_in = '0' + str_in
    return str_in
                   
if __name__ == '__main__':
  rospy.init_node('XBee_Bridge') # initialize the node
  
  com_port = rospy.get_param('com_port') # get com port name, something like dev/USB0
  baud_rate = rospy.get_param('baud_rate') # from xbee ~9600,~57600
  com_type = rospy.get_param('com_type') # this is EITHER 'receiver' OR 'broadcaster'
  fake_agent = rospy.get_param('fake_agent') # am I actually using XBees or am I testing w/o XBees? If I try to launch XBee w/o it being plugged in it will fail

  rospy.loginfo("XBee Bridge::initializing")
  rospy.loginfo(" XBee Bridge::com_port: %s", com_port)
  rospy.loginfo(" XBee Bridge::baud rate: %i", baud_rate)
  rospy.loginfo(" XBee Bridge::com_type: %s", com_type)
  rospy.loginfo(" XBee Bridge::fake_agent: %s", fake_agent)
  xbee = XBee(com_port, baud_rate, com_type, fake_agent)
  rospy.loginfo("xbee Bridge::initialized")
  rospy.spin()
  
