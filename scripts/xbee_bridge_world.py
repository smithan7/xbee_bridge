#!/usr/bin/env python


import rospy, math
import numpy as np
from random import *
import sys, termios, tty, select, os

from custom_messages.msg import DMCTS_Pulse, DMCTS_Task_List, DMCTS_Work_Status, DMCTS_Loc, DMCTS_Request_Task_List, DMCTS_Request_Work, DMCTS_Coordination

import serial


class XBee(object):
  initialized = False
  
  def __init__(self, com_port, baud_rate, com_type):
    
    self.check_xbee = rospy.Timer(rospy.Duration(1.0), self.xbee_callback) # check the Xbee for messages
    self.ser = serial.Serial(com_port, baud_rate)#'/dev/ttyUSB0',9600)  # open serial port
    self.ser.write(b'hello')     # write a string
    
    if com_type == 'ground_station':
    
        ### Setup publishers
        self.pub_pulse = rospy.Publisher('/dmcts_master/pulse', Pulse, queue_size=10) # Tell the agents the time
        self.pub_task_list = rospy.publisher('/dmcts_master/task_list', Task_List, queue_size=10) # Ground station tells agents which tasks are active and what their reward structure is
        self.pub_work_status = rospy.Publisher('/dmcts_master/work_status', Work_Status, queue_size=10) # Ground station tells agent if work / task is completed
        
        ### Setup subscribers
        self.sub_loc = rospy.Subscriber('/dmcts_master/loc', Loc, self.loc_callback) # ground station is updated with agent locations
        self.sub_request_task_list = rospy.Subscriber('/dmcts_master/request_task_list', Request_Task_List, self.request_task_list_callback) # agent requests task from ground station
        self.sub_request_work = rospy.Subscriber('/dmcts_master/request_work', Request_Work, self.request_work_callback) # agent attempts to work on a task
        
    elif com_type == 'agent':
        ### Setup Publishers
        self.pub_coordination = rospy.Publisher('/dmcts_master/coordination', Coordination, queue_size=10) # how agents coordinate with eachother
        self.pub_loc = rospy.publisher('/dmcts_master/loc', Location, queue_size=10) # Publish my location, largely for the ground station to plot a display
        self.pub_request_task_list = rospy.publisher('/dmcts_master/request_task_list', Request_Task_List, queue_size=10) # Ask the ground station for the task list
        self.pub_request_work = rospy.Publisher('/dmcts_master/request_work', Request_Work, queue_size=10) # Agent attempts to complete work by notifying ground station

        ## Setup Subscribers
        self.sub_coordination = rospy.Subscriber('/dmcts_master/coordination', Coordination, self.coordination_callback) # how agents coordinate with eachother
        self.sub_pulse = rospy.Subscriber('/dmcts_master/pulse', Pulse, self.pulse_callback) # agent recieves pulse from ground station and other agents notifiyng them of time
        self.sub_task_list = rospy.Subscriber('/dmcts_master/task_list', Task_List, self.task_list_callback) # agent recieves task list from ground station
        self.sub_work_status = rospy.Subscriber('/dmcts_master/work_status', Work_Status, self.work_status_callback) # agent is notified if they completed work succesfully
    
    else:
        while True:
            rospy.logerror("XBee Bridge:: Improper Communication Type Recieved, no communication established: " + com_type)
            rospy.sleep(rospy.Duration(3.0))
            
    
    ### Setup Services, these I should not be able to use probably as it would require waiting for XBee to transmit the message across and get a response, keep here just as reference in case in the future I need to set up a service
    #self.a_star_client = rospy.ServiceProxy('/dmcts_' + str(self.index) + '/costmap_bridge/a_star_path', Get_A_Star_Path)
    
  def xbee_callback(self, event):
    # c = coordination message; agents send this out to claim tasks
    # l = location message;     agents send this out to update their location
    # p = pulse message;        coordinator sends this out to set clock and n_active_tasks
    #                           agents echo it and send along
    # r = request task list;    agents send this out to get the updates task list
    # t = task list             coordinator sends out task list as requested
    # w = work request          agent tells coordinator that it is doing work
    # s = work status          coordinator tells agent if they did work or not
    
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
            n_indices = []
            times = []
            probs = []
            for n in range(0,n_nodes):
                #nn = self.ser.read()
                #print "n: ", nn
                if self.ser.read() == 'n':
                    claim = []
                    comma = self.ser.read()
                    n_index = int(self.ser.read(3)) # 3 digits for node index, allows 0-999 nodes
                    n_indices.append(n_index)
                    comma = self.ser.read()
                    n_time = float(self.ser.read(5)) / 10.0 # 5 digits for time, allows time 0.0-9999.9 seconds (166 min and 39.9 sec)
                    times.append(n_time)
                    comma = self.ser.read()
                    n_prob = float( self.ser.read(3)) / 100.0 # 3 digits for prob, allows 0.00 -> 1.00
                    probs.append(n_prob)
                    comma = self.ser.read()
                    print "claiming node ", n_index, " at time ", n_time, " with probability ", n_prob
             
            # publish all claims to quadrotor
            self.publish_coord_to_quad(n_indices, times, probs)
        
        ### Location Message
        elif tp == 'l':
            index = int(self.ser.read(2)) # 2 digits for agent index, allows 0-99 agents
            comma = self.ser.read()
            x = float(self.ser.read(4)) / 10.0 - 500.0 # 4 digits for xLoc, -500.0->500.0 m
            comma = self.ser.read()
            y = float(self.ser.read(4)) / 10.0 - 500.0 # 4 digits for yLoc, -500.0->500.0 m
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
        
        ### Work Request message, this is from an agent to the ground station to request work on a node
        elif tp == 'w':
            n_index = int(self.ser.read(3)) # 3 digits for node index, allows 0-999 nodes
            comma = self.ser.read()
            a_index = int(self.ser.read(2)) # 1 digit for agent type, 0-9 agent types
            
            self.publish_request_work(n_index, a_index)
            
        ### Work Status message, from the ground station to the agent telling if work was successful or not
        elif tp == 's':
            a_index = int(self.ser.read(2); # 2 digits for agent index, allows 0-99 agents
            comma = self.ser.read()
            n_index = int(self.ser.read(3)) # 3 digits for node index, allows 0-999 nodes
            comma = self.ser.read()
            success = self.ser.read() # 1 digit to indicate sucess, 1 means work was
            
            self.publish_work_status(a_index, n_index, success)
            
        ### Transmitted Task List, recieve list of active tasks from the coordinator
        elif tp == 't':
            n_nodes = int(self.ser.read(3)) # 3 digits for node index, 0-999 nodes
            comma = self.ser.read()
            nodes_indices = []
            rewards = []
            for i in range(0,n_nodes):
                node_indices.append( int(self.ser.read(3)) )
                comma = self.ser.read()
            
            self.publish_task_list(node_indices, rewards)
            
        ### Get Task List, request list of active tasks from the coordinator
        elif tp == 'r':
            self.publish_request_for_task_list()
        
        ### Message was invalid
        else:
            rospy.logwarn("XBee Bridge::Bad Message")

    def publish_coord_to_quad(self, n_indices, times, probs):
        a = 7


    def publish_pulse(self, agent_index, c_time, n_active_tasks):
        # This is the pulse from all of the other agents being echoed back
        msg = DMCTS_Pulse()
        msg.my_index = agent_index
        msg.c_time = c_time
        msg.n_active_tasks = n_active_tasks
        self.pub_pulse.publish(msg)
        
    def broadcast_pulse_callback(self, msg):
        # This sends the pulse out to the agents
        broadcast = '$p'
        for i in range(0,n_nodes):
            broadcast = broadcast + str(msg.node_indices[i]) + ','
            broadcast = broadcast + str(int(reward *100.0)) + ','
        
        self.ser.write(b + msg)     # write a string
    
    def publish_location(self, index, x,y,edge_x,edge_y,status):
        # This tells the coordinator where each agent is
        msg = Recieve_Agent_Locs()
        msg.index = index
        msg.xLoc = x
        msg.yLoc = y
        msg.edge_x = edge_x
        msg.edge_y = edge_y
        msg.status = status
        self.pub_agent_loc.publish(msg)
        
    def broadcast_location_callback(self, msg):
        # This tells the coordinator my location
        broadcast = '$l'
        broadcast = broadcast + self.set_string_length(str(msg.index), 2) + ',' # 2 digits for agent index, allows 0-99 agents
        broadcast = broadcast + self.set_string_length(str(int(msg.x - 10.0 - 500.0)), 4) + ',' # 4 digits for xLoc, -500.0->500.0 m
        broadcast = broadcast + self.set_string_length(str(int(msg.y - 10.0 - 500.0)), 4) + ',' # 4 digits for yLoc, -500.0->500.0 m
        broadcast = broadcast + self.set_string_length(str(msg.edge_x), 3) + ',' # 3 digits for node index, allows 0-999 nodes
        broadcast = broadcast + self.set_string_length(str(msg.edge_y), 3) + ',' # 3 digits for node index, allows 0-999 nodes
        broadcast = broadcast + self.set_string_length(str(msg.status), 1) + ',' # 1 char for agent and world status, allows all chars as possible status nodes
        
        self.ser.write(b + broadcast)     # write a string
    
    def publish_request_work(self, n_index, a_index):
        # This tells the coordinator that an agent is trying to do some work
        msg = Request_Work()
        msg.agent_index = a_index;
        msg.node_index = n_index
        self.pub_request_work.publish(msg)
        
    def broadcast_request_work_callback(self, msg):
        # Agent trying to do work
        broadcast = '$w'
        broadcast = broadcast + self.set_string_length(str(msg.n_index), 3) + ',' # 3 digits for node index, allows 0-999 nodes
        broadcast = broadcast + self.set_string_length(str(msg.a_index), 2) + ',' # 2 digits for agent index, allows 0-99 agents
        self.ser.write(b + broadcast)     # write a string
        
    def publish_work_status(self, a_index, n_index, success):
        # This takes the message from the ground station, through the Xbee, to the agent informing them if they are succesfully working on the task
        msg = Work_Status()
        msg.a_index = a_index
        msg.n_index = n_index
        msg.success = success
        self.pub_work_status.publish(msg)
    
    def broadcast_work_status_callback(self, msg):
        broadcast = '$s'
        broadcast = broadcast + self.set_string_length(str(msg.a_index), 2) + ',' # 2 digits for agent index, allows 0-99 agents
        broadcast = broadcast + self.set_string_length(str(msg.n_index), 3) + ',' # 3 digits for node index, allows 0-999 nodes
        broadcast = broadcast + self.set_string_length(str(msg.success), 1) # 1 digit to indicate sucess, 1 means work was succesful, 0 means it was not
        self.ser.write(b + broadcast)     # write a stringe sucess, 1 means work was
        
        
        
    def publish_request_for_task_list():
        # This tells the coordinator node that an agent has requested the task list
        msg = Request_For_Task_List();
        self.pub_req_task_list(msg)

    def broadcast_request_for_task_list_callback(self, msg):
        broadcast = '#r'
        self.ser.write(b + broadcast)     # write a string    
  

    def broadcast_task_list_callback(self,  msg):
        # This recieves the task list from the coordinator node and then sends it out over the xbee
        # This consists of only the nodes that are active and the reward type
        n_nodes = len(msg.node_indices)
        broadcast = '$t'
        for i in range(0,n_nodes):
            broadcast = broadcast + str(msg.node_indices[i]) + ','
        
        self.ser.write(b + broadcast)     # write a string    
  
  def set_string_length(self, string, length):
    # This appends '0' 's to the string to make it the right length
    temp_0 = length - len(temp_str)
    for i in range(0, temp_0):
       string = '0' + temp_str
       
    return string
              
        
if __name__ == '__main__':
  rospy.init_node('XBee_Bridge')
  
  com_port = rospy.get_param('com_port')
  baud_rate = rospy.get_param('baud_rate')
  comm_type = rospy.get_param('com_type')

  rospy.loginfo("XBee Bridge::initializing")
  rospy.loginfo(" XBee Bridge::com_port: %s", com_port)
  rospy.loginfo(" XBee Bridge::baud rate: %i", baud_rate)
  rospy.loginfo(" XBee Bridge::com_type: %i", com_type)
  xbee = XBee(com_port, baud_rate, com_type)
  rospy.loginfo("xbee Bridge::initialized")
  rospy.spin()
  
