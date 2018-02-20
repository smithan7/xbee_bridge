#!/usr/bin/env python


# useful link : http://pyserial.readthedocs.io/en/latest/shortintro.html

import rospy, math
import numpy as np
from random import *
import sys, termios, tty, select, os
import serial

from custom_messages.msg import DMCTS_Travel_Goal
from custom_messages.srv import Get_A_Star_Path, Get_Kinematic_A_Star_Path
from hector_uav_msgs.srv import EnableMotors
from hector_uav_msgs.msg import PoseAction
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped
from nav_msgs.msg import Odometry, Path

class Xbee(object):

  def __init__(self, com_port, baud_rate):
    
    self.ser = serial.Serial(com_port, baud_rate)  # open serial port
    print(ser.name)         # check which port was really used
    self.ser.write(b'hello')     # write a string
    
    self.check_xbee = rospy.Timer(rospy.Duration(0.01), self.xbee_callback)
    
    #self.pub_twist = rospy.Publisher('/uav' + str(self.index) + '/cmd_vel', Twist, queue_size=10)
    #self.sub_twist = rospy.Subscriber('/cmd_vel_from_move_base', Twist, self.twist_callback)
    #self.sub_odom = rospy.Subscriber('/uav' + str(self.index) + '/ground_truth/state', Odometry, self.odom_callback )
    #self.goal_sub = rospy.Subscriber('/dmcts_' + str(self.index) + '/travel_goal', DMCTS_Travel_Goal, self.goal_callback )
    #self.a_star_client = rospy.ServiceProxy('/dmcts_' + str(self.index) + '/costmap_bridge/a_star_path', Get_A_Star_Path)
    #self.kinematic_client = rospy.ServiceProxy('/dmcts_' + str(self.index) + '/costmap_bridge/kinematic_path', Get_Kinematic_A_Star_Path)

  def xbee_callback(self, event):
    if not self.ser.is_open:
        self.ser.open()
        if not self.ser.is_open:
            rospy.logwarn("Xbee_Bridge:: Still unable to open Serial Port")
            rospy.sleep(rospy..Duration(3.0)
            return
    
    data = self.ser.read(100) # read up to 100 bytes or until the buffer is empty
    print data
    
if __name__ == '__main__':
  rospy.init_node('XBee_Bridge')
  
  com_port = rospy.get_param('com_port')
  baud_rate = rospy.get_param('baud_rate')

  rospy.loginfo("XBee Bridge::initializing")
  rospy.loginfo(" XBee Bridge::com_port: %s", com_port)
  rospy.loginfo(" XBee Bridge::baud rate: %i", baud_rate)
  xbee = XBee(com_port, baud_rate)
  
  rospy.loginfo("xbee Bridge::initialized")
  rospy.spin()
