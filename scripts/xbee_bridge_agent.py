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
        ### Coordination Message
        if tp == 'c':
            #print "tp: ", tp
            n_nodes = int(self.ser.read(3)) # 3 digits for number of nodes in the message
            #print "n_nodes: ", n_nodes
            comma = self.ser.read()
            #print "comma: ", comma
            claims = []
            for n in range(0,n_nodes):
                #nn = self.ser.read()
                #print "n: ", nn
                if self.ser.read() == 'n':
                    claim = []
                    comma = self.ser.read()
                    n_index = int(self.ser.read(3)) # 3 digits for node index, allows 0-999 nodes
                    claim.append(n_index)
                    comma = self.ser.read()
                    n_time = float(self.ser.read(5)) / 10.0 # 5 digits for time, allows time 0.0-9999.9 seconds (166 min and 39.9 sec)
                    claim.append(n_time)
                    comma = self.ser.read()
                    n_prob = float( self.ser.read(3)) / 100.0 # 3 digits for prob, allows 0.00 -> 1.00
                    claim.append(n_prob)
                    claims.append(claim)
                    comma = self.ser.read()
                    print "claiming node ", n_index, " at time ", n_time, " with probability ", n_prob
             
             # publish all claims to quadrotor
             self.publish_coord_to_quad(claims)
        
        ### Location Message
        elif tp == 'l':
            index = int(self.ser.read(2)) # 2 digits for agent index, allows 0-99 agents
            comma = self.ser.read()
            x = float(self.ser.read(4)) / 10.0 - 500.0 # 4 digits for xLoc, -500.0->500.0 m
            comma = self.ser.read()
            y = float(self.ser.read(4)) / 10.0 - 500.0 # 4 digits for yLoc, -500.0->500.0 m
            comma = self.ser.read()
            z = float(self.ser.read(3)) / 10.0 # 3 digits for alt, 00.0->99.9 m
            comma = self.ser.read()
            w = float(self.ser.read(2)) / 10.0 # 2 digits for alt, 0.0->9.9 rad
            comma = self.ser.read()
            edge_x = int(self.ser.read(3)) # 3 digits for node index, allows 0-999 nodes
            comma = self.ser.read()
            edge_y = int(self.ser.read(3)) # 3 digits for node index, allows 0-999 nodes
            comma = self.ser.read()
            status = self.ser.read(1) # 1 char for agent and world status, allows all chars as possible status nodes
            self.publish_location(index,x,y,z,w,edge_x,edge_y,status)
            
        ### Pulse message, this is from the ground station providing the clock and number of active nodes
        elif tp == 'p':
            agent_index = int(self.ser.read(2)) # 2 digits for agent index, allows 0-99 agents
            comma = self.ser.read()
            c_time = float(self.ser.read(5)) / 10.0 # 5 digits for time, allows time 0.0-9999.9 seconds (166 min and 39.9 sec)
            comma = self.ser.read()
            n_active_tasks = int(self.ser.read(3)) # 3 digits for node index, allows 0-999 nodes
            self.publish_pulse(agent_index, c_time, n_active_tasks)
        
        ### Complete Work message, this is from an agent to the ground station to request work on a node
        elif tp == 'w':
            
        
        ### Get Task List, request list of active tasks from the coordinator
        elif tp == 't':
        
        else:
            rospy.logwarn("XBee Bridge::Bad Message")

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
  
