#!/usr/bin/env python


import rospy, math
import numpy as np
from random import *
import sys, termios, tty, select, os

from custom_messages.msg import DMCTS_Travel_Goal
from custom_messages.srv import Get_A_Star_Path, Get_Kinematic_A_Star_Path
from hector_uav_msgs.srv import EnableMotors
from hector_uav_msgs.msg import PoseAction
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped
from nav_msgs.msg import Odometry, Path

import serial


class XBee(object):
  initialized = False
  
  def __init__(self, com_port, baud_rate):
    
    self.check_xbee = rospy.Timer(rospy.Duration(1.0), self.xbee_callback)
    self.ser = serial.Serial(com_port, baud_rate)#'/dev/ttyUSB0',9600)  # open serial port
    self.ser.write(b'hello')     # write a string
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
            rospy.logerror("Xbee_Bridge:: Still unable to open Serial Port %s", self.ser.name)
            rospy.sleep(rospy.Duration(3.0))
    
    else:
        if not self.initialized and self.ser.is_open:
            self.initialized = True
            rospy.loginfo("XBee Bridge:: initialized serial port on %s", self.ser.name)
        
        
        #data = self.ser.read(100) # read up to 100 bytes or until the buffer is empty
        while True:
            data = self.ser.read()
            #print "data: ", data
            if data == '$':
                #print "FOUND $$$$"
                break
        
        tp = self.ser.read()
        if tp == 'c':
            #print "tp: ", tp
            n_nodes = int(self.ser.read(3)) # 3 digits for number of nodes in the message
            #print "n_nodes: ", n_nodes
            comma = self.ser.read()
            #print "comma: ", comma
            for n in range(0,n_nodes):
                #nn = self.ser.read()
                #print "n: ", nn
                if self.ser.read() == 'n':
                    comma = self.ser.read()
                    n_index = int(self.ser.read(3)) # 3 digits for node index
                    comma = self.ser.read()
                    n_time = float(self.ser.read(5)) / 100.0 # 5 digits for time
                    comma = self.ser.read()
                    n_prob = float( self.ser.read(3)) / 1000.0 # 3 digits for prob
                    print "claiming node ", n_index, " at time ", n_time, " with probability ", n_prob
        else:
            print "tp: ", tp

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
  
