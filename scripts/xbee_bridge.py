#!/usr/bin/env python


import rospy, math
import numpy as np
from random import *
import sys, termios, tty, select, os
from std_msgs.msg import String

from custom_messages.msg import DMCTS_Pulse, DMCTS_Task_List, DMCTS_Work_Status, DMCTS_Loc, DMCTS_Request_Task_List, DMCTS_Request_Work, DMCTS_Coordination

import serial


class XBee(object):
  initialized = False
  
  def __init__(self, com_port, baud_rate, com_type, fake_agent, agent_index):
    
    self.fake_agent = fake_agent
    self.agent_index = agent_index
    self.com_type = com_type

    if self.fake_agent:
        self.pub_chatter = rospy.Publisher('/xbee/chatter', String, queue_size=10) # Publish over a topic instead of out over xbee
        self.sub_chatter = rospy.Subscriber('/xbee/chatter', String, self.xbee_chatter_callback) # Sub to topic
    else:
        self.ser = serial.Serial(com_port, baud_rate)#'/dev/ttyUSB0',9600)  # open serial port
        try:
            self.check_xbee = rospy.Timer(rospy.Duration(1), self.xbee_callback) # check the Xbee for messages
            self.xbee_broadcast('hello') # write a string
        except:
            rospy.logwarn("XBee_Bridge::could not initialize Xbee")
            rospy.sleep(rospy.Duration(3.0))

    if self.com_type == 'ground_station':
        ### Setup publishers, in reality this is what the ground station subscribes to
        self.pub_request_work = rospy.Publisher('/dmcts_master/request_work', DMCTS_Request_Work, queue_size=10) # Agent attempts to complete work by notifying ground station
        self.pub_loc = rospy.Publisher('/dmcts_master/loc', DMCTS_Loc, queue_size=10) # Publish my location, largely for the ground station to plot a display
        self.pub_request_task_list = rospy.Publisher('/dmcts_master/request_task_list', DMCTS_Request_Task_List, queue_size=10) # Ask the ground station for the task list
        
        ### Setup subscribers, in reality this is what the ground station publishes
        self.sub_pulse = rospy.Subscriber('/dmcts_master/pulse', DMCTS_Pulse, self.broadcast_pulse_callback) # agent recieves pulse from ground station and other agents notifiyng them of time
        self.sub_work_status = rospy.Subscriber('/dmcts_master/work_status', DMCTS_Work_Status, self.broadcast_work_status_callback) # agent is notified if they completed work succesfully
        self.sub_task_list = rospy.Subscriber('/dmcts_master/task_list', DMCTS_Task_List, self.broadcast_task_list_callback) # agent recieves task list from ground station
        
    elif self.com_type == 'agent':
        ### Setup Publishers, in reality this is what the agent subscribes to
        self.pub_pulse = rospy.Publisher('/dmcts_master/pulse', DMCTS_Pulse, queue_size=10) # Tell the agents the time
        self.pub_task_list = rospy.Publisher('/dmcts_master/task_list', DMCTS_Task_List, queue_size=10) # Ground station tells agents which tasks are active and what their reward structure is
        self.pub_coordination = rospy.Publisher('/dmcts_master/coordination', DMCTS_Coordination, queue_size=10) # how agents coordinate with eachother
        self.pub_work_status = rospy.Publisher('/dmcts_master/work_status', DMCTS_Work_Status, queue_size=10) # Ground station tells agent if work / task is completed
        
        ## Setup Subscribers, in reality, this is what the agent publishes
        self.sub_request_work = rospy.Subscriber('/dmcts_master/request_work', DMCTS_Request_Work, self.broadcast_request_work_callback) # agent attempts to work on a task
        self.sub_request_task_list = rospy.Subscriber('/dmcts_master/request_task_list', DMCTS_Request_Task_List, self.broadcast_request_for_task_list_callback) # agent requests task from ground station
        self.sub_loc = rospy.Subscriber('/dmcts_master/loc', DMCTS_Loc, self.broadcast_location_callback) # ground station is updated with agent locations
        self.sub_coordination = rospy.Subscriber('/dmcts_master/coordination', DMCTS_Coordination, self.broadcast_coordination_callback) # how agents coordinate with eachother
    
    else:
        while True:
            rospy.logerr("XBee Bridge:: Improper Communication Type Recieved, no communication established: " + com_type)
            rospy.sleep(rospy.Duration(3.0))
            
    
    ### Setup Services, these I should not be able to use probably as it would require waiting for XBee to transmit the message across and get a response, keep here just as reference in case in the future I need to set up a service
    #self.a_star_client = rospy.ServiceProxy('/dmcts_' + str(self.index) + '/costmap_bridge/a_star_path', Get_A_Star_Path)

  def xbee_broadcast(self, msg):
    try:
        if self.fake_agent:
            self.pub_chatter.publish(msg)

        else:
            msg = msg + '\n'
            if not self.ser.is_open:
                self.ser.open()
                if not self.ser.is_open:
                    rospy.logerr("Xbee_Bridge:: Still unable to open Serial Port %s", self.ser.name)
                    rospy.sleep(rospy.Duration(3.0))
                    return
            
            else:
                if not self.initialized and self.ser.is_open:
                    self.initialized = True
                    rospy.loginfo("XBee Bridge:: initialized serial port on %s", self.ser.name)
            
            self.ser.write(msg);
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed in xbee broadcast: " + msg)

  def xbee_chatter_callback(self, msg):
    try:
        #rospy.loginfo("chatter msg in: " + msg.data)
        msg = str(msg.data)
        if msg[0] == '$':
            msg = msg[1:]
            tp = msg[0]
            msg = msg[1:]

            if self.com_type == 'ground_station':
                ### Location Message
                if tp == 'l':
                    self.read_location_chatter(msg)
                ### Work Request message, this is from an agent to the ground station to request work on a node
                elif tp == 'w':
                    self.read_work_request_chatter(msg)
                ### Get Task List, request list of active tasks from the coordinator
                elif tp == 'r':
                    # This is right, the message is empty!!!
                    self.publish_request_for_task_list()
                elif tp == 'c' or tp == 'p' or tp == 's' or tp == 't':
                    a = 7
                    #rospy.loginfo("Xbee_Bridge::ground_station got agent message: " + tp)
                ### Message was invalid
                else:
                    rospy.logwarn("XBee Bridge::Bad ground_station Message type: " + tp)

            elif self.com_type == 'agent':
                ### Coordination Message
                if tp == 'c':
                    self.read_coordination_chatter(msg)
                ### Pulse message, this is from the ground station providing the clock and number of active nodes
                elif tp == 'p':
                    self.read_pulse_chatter(msg)
                ### Work Status message, from the ground station to the agent telling if work was successful or not
                elif tp == 's':
                    self.read_work_status_chatter(msg)
                ### Transmitted Task List, recieve list of active tasks from the coordinator
                elif tp == 't':
                    rospy.loginfo("got task list msg: " + msg)
                    self.read_task_list_chatter(msg)
                ### Message was for agent
                elif tp == 'l' or tp == 'w' or tp == 'r':
                    a = 7
                    #rospy.loginfo("Xbee_Bridge::Agent got ground station message: " + tp)
                ### Message was invalid    
                else:
                    rospy.logwarn("XBee Bridge::Bad agent Message type: " + tp)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed in xbee chatter callback msg " + msg)

  def xbee_callback(self, event):
    try:
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
                rospy.logerr("Xbee_Bridge:: Still unable to open Serial Port %s", self.ser.name)
                rospy.sleep(rospy.Duration(3.0))
                return

        else:
            if not self.initialized and self.ser.is_open:
                self.initialized = True
                rospy.loginfo("XBee Bridge:: initialized serial port on %s", self.ser.name)
            
            # While there is data in the buffer
            while self.ser.in_waiting > 0:
                # Eat through data until header is found
                while True:
                    data = self.ser.read()
                    #print "data: ", data
                    if data == '$':
                        #rospy.logerr("FOUND $$$$")
                        data = self.ser.read()
                        msg = ''
                        while True: ## Search for EOL - end of line '/n'
                            if data == "\n":
                                #rospy.logerr("Found: eo")
                                break
                            else:
                                #rospy.loginfo('Data: ' + data + ' and msg: ' + msg)
                                msg = msg + data
                                data = self.ser.read()
                        rospy.logerr("XBee_Bridge:: msg: " + msg)
                        break
                tp = msg[0]
                msg = msg[1:]
                if self.com_type == 'ground_station':
                    ### Location Message
                    if tp == 'l':
                        self.read_location_chatter(msg)
                    ### Work Request message, this is from an agent to the ground station to request work on a node
                    elif tp == 'w':
                        self.read_work_request_chatter(msg)
                    ### Get Task List, request list of active tasks from the coordinator
                    elif tp == 'r':
                        # This is right, the message is empty!!!
                        self.publish_request_for_task_list()
                    elif tp == 'c' or tp == 'p' or tp == 's' or tp == 't':
                        rospy.logwarn("Xbee_Bridge::ground_station got agent message: " + tp)
                    ### Message was invalid
                    else:
                        rospy.logwarn("XBee Bridge::Bad ground_station Message type: " + tp)

                elif self.com_type == 'agent':
                    ### Coordination Message
                    if tp == 'c':
                        self.read_coordination_chatter(msg)
                    ### Pulse message, this is from the ground station providing the clock and number of active nodes
                    elif tp == 'p':
                        self.read_pulse_chatter(msg)
                    ### Work Status message, from the ground station to the agent telling if work was successful or not
                    elif tp == 's':
                        self.read_work_status_chatter(msg)
                    ### Transmitted Task List, recieve list of active tasks from the coordinator
                    elif tp == 't':
                        rospy.loginfo("got task list msg: " + msg)
                        self.read_task_list_chatter(msg)
                    ### Message was for agent
                    elif tp == 'l' or tp == 'w' or tp == 'r':
                        rospy.loginfo("Xbee_Bridge::Agent got ground station message: " + tp)
                    ### Message was invalid    
                    else:
                        rospy.logwarn("XBee Bridge::Bad agent Message type: " + tp)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed in xbee callback")

  def publish_request_for_task_list(self):
    try:
        # This tells the coordinator node that an agent has requested the task list
        msg = DMCTS_Request_Task_List()
        self.pub_request_task_list.publish(msg)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to publish request for task list")


  def broadcast_request_for_task_list_callback(self, msg):
    try:
        broadcast = '$r'
        self.xbee_broadcast(broadcast)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to broadcast request for task list msg " + msg)


  def read_task_list_msg(self):
    try:
        n_nodes = int(self.ser.read(3)) # 3 digits for node index, 0-999 nodes
        comma = self.ser.read()
        nodes_indices = []
        for i in range(0,n_nodes):
            node_indices.append( int(self.ser.read(3)) )
            comma = self.ser.read()
        self.publish_task_list(node_indices)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to read task list msg")


  def read_task_list_chatter(self, msg):
    try:
        n_nodes = int(msg[0:3]) # 3 digits for node index, 0-999 nodes
        msg = msg[3:]
        comma = msg[0]
        msg = msg[1:]
        node_indices = []
        for i in range(0,n_nodes):
            node_indices.append( int(msg[0:3]) )
            msg = msg[3:]
            comma = msg[0]
            msg = msg[1:]
        self.publish_task_list(node_indices)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to read task list chatter msg " + msg)

  def publish_task_list(self, node_indices):
    try:
        msg = DMCTS_Task_List()
        msg.node_indices = node_indices
        self.pub_task_list.publish(msg)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to publish task list")

  def broadcast_task_list_callback(self,  msg):
    try:
        # This recieves the task list from the coordinator node and then sends it out over the xbee
        # This consists of only the nodes that are active and the reward type
        n_nodes = len(msg.node_indices)
        broadcast = '$t'
        broadcast = broadcast + self.set_string_length(str(n_nodes),3) + ','
        for i in range(0,n_nodes):
            broadcast = broadcast + self.set_string_length(str(msg.node_indices[i]),3) + ','    
        self.xbee_broadcast(broadcast)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to broadcast task list msg " + msg)


  def read_work_status_msg(self):
    try:
        a_index = int(self.ser.read(2)) # 2 digits for agent index, allows 0-99 agents
        comma = self.ser.read()
        n_index = int(self.ser.read(3)) # 3 digits for node index, allows 0-999 nodes
        comma = self.ser.read()
        success = int(self.ser.read()) # 1 digit to indicate sucess, 1 means work was        
        self.publish_work_status(a_index, n_index, success)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to read work status msg")

  def read_work_status_chatter(self, msg):
    try:
        a_index = int(msg[0:2]) # 2 digits for agent index, allows 0-99 agents
        msg = msg[2:]
        comma = msg[0]
        msg = msg[1:]
        n_index = int(msg[0:3]) # 3 digits for node index, allows 0-999 nodes
        msg = msg[3:]
        comma = msg[0]
        msg = msg[1:]
        success = int(msg[0]) # 1 digit to indicate sucess, 1 means work was
        self.publish_work_status(a_index, n_index, success)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to read work status chatter msg " + msg)

  def publish_work_status(self, a_index, n_index, success):
    try:
        # This takes the message from the ground station, through the Xbee, to the agent informing them if they are succesfully working on the task
        msg = DMCTS_Work_Status()
        msg.a_index = a_index
        msg.n_index = n_index
        msg.success = success
        self.pub_work_status.publish(msg)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to publish work status")

  def broadcast_work_status_callback(self, msg):
    try:
        broadcast = '$s'
        broadcast = broadcast + self.set_string_length(str(msg.a_index), 2) + ',' # 2 digits for agent index, allows 0-99 agents
        broadcast = broadcast + self.set_string_length(str(msg.n_index), 3) + ',' # 3 digits for node index, allows 0-999 nodes
        broadcast = broadcast + self.set_string_length(str(msg.success), 1) # 1 digit to indicate sucess, 1 means work was succesful, 0 means it was not
        self.xbee_broadcast(broadcast)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to broadcast work status msg " + msg)

  def read_work_request_msg(self):
    try:
        n_index = int(self.ser.read(3)) # 3 digits for node index, allows 0-999 nodes
        comma = self.ser.read()
        a_index = int(self.ser.read(2)) # 2 digits for agent index, 0-99 agents
        comma = self.ser.read()
        a_type = int(self.ser.read(2)) # 2 digits for agent types, 0-99 agent types
        self.publish_request_work(n_index, a_index, a_type)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to read work request msg")

  def read_work_request_chatter(self, msg):
    try:
        n_index = int(msg[0:3]) # 3 digits for node index, allows 0-999 nodes
        msg = msg[3:]
        comma = msg[0]
        msg = msg[1:]
        a_index = int(msg[0:2]) # 2 digits for agent index, 0-99 agents    
        msg = msg[2:]
        comma = msg[0]
        msg = msg[1:]
        a_type = int(msg[0:2])
        self.publish_request_work(n_index, a_index, a_type)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to read work request chatter msg " + msg)


  def publish_request_work(self, n_index, a_index, a_type):
    try:
        # This tells the coordinator that an agent is trying to do some work
        msg = DMCTS_Request_Work()
        msg.a_index = a_index;
        msg.n_index = n_index
        msg.a_type = a_type;
        self.pub_request_work.publish(msg)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to publish request work")

  def broadcast_request_work_callback(self, msg):
    try:
        # Agent trying to do work
        broadcast = '$w'
        broadcast = broadcast + self.set_string_length(str(msg.n_index), 3) + ',' # 3 digits for node index, allows 0-999 nodes
        broadcast = broadcast + self.set_string_length(str(msg.a_index), 2) + ',' # 2 digits for agent index, allows 0-99 agents
        broadcast = broadcast + self.set_string_length(str(msg.a_type), 2) + ',' # 2 digits for agent type, allows 0-99 agent types
        self.xbee_broadcast(broadcast)        
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to broadcast request work msg " + msg)

  def read_pulse_msg(self):
    try:
        agent_index = int(self.ser.read(2)) # 2 digits for agent index, allows 0-99 agents
        comma = self.ser.read()
        c_time = float(self.ser.read(5)) / 10.0 # 5 digits for time, allows time 0.0-9999.9 seconds (166 min and 39.9 sec)
        comma = self.ser.read()
        n_active_tasks = int(self.ser.read(3)) # 3 digits for node index, allows 0-999 nodes
        self.publish_pulse(agent_index, c_time, n_active_tasks)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to read pulse msg")


  def read_pulse_chatter(self, msg):
    try:
        agent_index = int(msg[0:2]) # 2 digits for agent index, allows 0-99 agents
        msg = msg[2:]
        comma = msg[0]
        msg = msg[1:]
        c_time = float(msg[0:5]) / 10.0 # 5 digits for time, allows time 0.0-9999.9 seconds (166 min and 39.9 sec)
        msg = msg[5:]
        comma = msg[0]
        msg = msg[1:]
        status = int(msg[0:1])
        msg = msg[1:]
        comma = msg[0]
        msg = msg[1:]
        n_active_tasks = int(msg[0:3]) # 3 digits for node index, allows 0-999 nodes
        self.publish_pulse(agent_index, c_time, n_active_tasks, status)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to read pulse chatter msg")

    
  def publish_pulse(self, agent_index, c_time, n_active_tasks, status):
    try:
        # This is the pulse from all of the other agents being echoed back
        msg = DMCTS_Pulse()
        msg.my_index = agent_index
        msg.c_time = c_time
        msg.n_active_tasks = n_active_tasks
        msg.status = status
        self.pub_pulse.publish(msg) # this is from the XBee to the agent
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to publish pulse")

    
  def broadcast_pulse_callback(self, msg):
    try:
        # This sends the pulse out to the agents over Xbee
        broadcast = '$p'
        broadcast = broadcast + self.set_string_length(str(self.agent_index),2) + ','
        broadcast = broadcast + self.set_string_length(str(int(msg.c_time * 10.0)), 5) + ','
        broadcast = broadcast + str(msg.status) + ','
        broadcast = broadcast + self.set_string_length(str(msg.n_active_tasks),3)    
        self.xbee_broadcast(broadcast)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to broadcast pulse msg ")


  def read_location_msg(self):
    try:
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
        status = int(self.ser.read(1)) # 1 char for agent and world status, allows all chars as possible status nodes
        self.publish_location(index,x,y,edge_x,edge_y,status)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to read location msg")

  def read_location_chatter(self, msg):
    try:
        index = int(msg[0:2]) # 2 digits for agent index, allows 0-99 agents
        msg = msg[2:]
        comma = msg[0]
        msg = msg[1:]
        x = float(msg[0:4]) / 10.0 - 500.0 # 4 digits for xLoc, -500.0->500.0 m
        msg = msg[4:]
        comma = msg[0]
        msg = msg[1:]
        y = float(msg[0:4]) / 10.0 - 500.0 # 4 digits for yLoc, -500.0->500.0 m
        msg = msg[4:]
        comma = msg[0]
        msg = msg[1:]
        edge_x = int(msg[0:3]) # 3 digits for node index, allows 0-999 nodes
        msg = msg[3:]
        comma = msg[0]
        msg = msg[1:]
        edge_y = int(msg[0:3]) # 3 digits for node index, allows 0-999 nodes
        msg = msg[3:]
        comma = msg[0]
        msg = msg[1:]
        status = int(msg[0]) # 1 char for agent and world status, allows all chars as possible status nodes
        self.publish_location(index,x,y,edge_x,edge_y,status)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to read location msg: " + msg)


  def publish_location(self, index, x,y,edge_x,edge_y,status):
    try:
        # This tells the coordinator where each agent is
        msg = DMCTS_Loc()
        msg.index = index
        msg.xLoc = x
        msg.yLoc = y
        msg.edge_x = edge_x
        msg.edge_y = edge_y
        msg.status = status
        self.pub_loc.publish(msg)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to publish location")

    
  def broadcast_location_callback(self, msg):
    try:
        # This tells the coordinator my location
        broadcast = '$l'
        broadcast = broadcast + self.set_string_length(str(msg.index), 2) + ',' # 2 digits for agent index, allows 0-99 agents
        broadcast = broadcast + self.set_string_length(str(int((msg.xLoc+500.0) * 10.0)), 4) + ',' # 4 digits for xLoc, -500.0->500.0 m
        broadcast = broadcast + self.set_string_length(str(int((msg.yLoc+500.0) * 10.0)), 4) + ',' # 4 digits for yLoc, -500.0->500.0 m
        broadcast = broadcast + self.set_string_length(str(msg.edge_x), 3) + ',' # 3 digits for node index, allows 0-999 nodes
        broadcast = broadcast + self.set_string_length(str(msg.edge_y), 3) + ',' # 3 digits for node index, allows 0-999 nodes
        broadcast = broadcast + self.set_string_length(str(msg.status), 1) + ',' # 1 char for agent and world status, allows all chars as possible status nodes    
        self.xbee_broadcast(broadcast)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to broadcast location msg ")


  def read_coordination_msg(self):
    try:
        msg = DMCTS_Coordination()
        #print "tp: ", tp
        n_nodes = int(self.ser.read(3)) # 3 digits for number of nodes in the message
        #print "n_nodes: ", n_nodes
        comma = self.ser.read()
        #print "comma: ", comma
        msg.agent_index = int(self.ser.read(2)) # 2 digits for number of agents
        comma = self.ser.read()
        n_indices = []
        times = []
        probs = []
        for n in range(0,n_nodes):
            #nn = self.ser.read()
            #print "n: ", nn
            if self.ser.read() == 'n':
                n_index = int(self.ser.read(3)) # 3 digits for node index, allows 0-999 nodes
                n_indices.append(n_index)
                comma = self.ser.read()
                n_time = float(self.ser.read(5)) / 10.0 # 5 digits for time, allows time 0.0-9999.9 seconds (166 min and 39.9 sec)
                times.append(n_time)
                comma = self.ser.read()
                n_prob = float( self.ser.read(3)) / 100.0 # 3 digits for prob, allows 0.00 -> 1.00
                probs.append(n_prob)
                comma = self.ser.read()

        # publish all claims to quadrotor
        msg.claimed_tasks = n_indices
        msg.claimed_time = times
        msg.claimed_probability = probs
        self.pub_coordination.publish(msg)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to read coordination msg")

  def read_coordination_chatter(self, msg):
    try:
        n_nodes = int(msg[0:3]) # 3 digits for number of nodes in the message
        msg = msg[3:]
        comma = msg[0]
        msg = msg[1:]
        msg_out = DMCTS_Coordination()
        msg_out.agent_index = int(msg[0:2]) # 2 digits for number of agents
        msg = msg[2:]
        comma = msg[0]
        msg = msg[1:]
        n_indices = []
        times = []
        probs = []
        for n in range(0,n_nodes):
            n_index = int(msg[0:3]) # 3 digits for node index, allows 0-999 nodes
            msg = msg[3:]
            n_indices.append(n_index)
            comma = msg[0]
            msg = msg[1:]
            n_time = float(msg[0:5]) / 10.0 # 5 digits for time, allows time 0.0-9999.9 seconds (166 min and 39.9 sec)
            msg = msg[5:]
            times.append(n_time)
            comma = msg[0]
            msg = msg[1:]
            n_prob = float(msg[0:3]) / 100.0 # 3 digits for prob, allows 0.00 -> 1.00
            msg = msg[3:]
            probs.append(n_prob)
            comma = msg[0]
            msg = msg[1:]

        # publish all claims to quadrotor
        msg_out.claimed_tasks = n_indices
        msg_out.claimed_time = times
        msg_out.claimed_probability = probs
        self.pub_coordination.publish(msg_out)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to read coordination chatter")


  def broadcast_coordination_callback(self, msg):
    try:
        broadcast = "$c"
        broadcast = broadcast + self.set_string_length(str(len(msg.claimed_tasks)),3) + ","
        broadcast = broadcast + self.set_string_length(str(msg.agent_index),2) + ","
        for i in range(0,len(msg.claimed_tasks)):
            broadcast = broadcast + self.set_string_length(str(msg.claimed_tasks[i]),3) + ","
            broadcast = broadcast + self.set_string_length(str(int(msg.claimed_time[i]*10.0)),5) + ","
            broadcast = broadcast + self.set_string_length(str(int(msg.claimed_probability[i]*100.0)),3) + ","
        self.xbee_broadcast(broadcast)
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to broadcast coordination msg")


  def set_string_length(self, str_in, length):
    try:
        # This appends '0' 's to the string to make it the right length
        if str_in[0] == '-':
            str_in = str_in[1:]
            temp_0 = length - len(str_in) - 1
            for i in range(0, temp_0):
                str_in = '0' + str_in
                
            str_in = '-' + str_in
            return str_in    
        else:
            temp_0 = length - len(str_in)
            for i in range(0, temp_0):
               str_in = '0' + str_in
               
            return str_in
    except:
        # For some reason the above code broke and failed to work, provide error msg without killing the node, this is normally because a non-number was tried to turn into a float; e.g. float(89,123) or float($l123)
        rospy.logwarn("XBee_Bridge::Failed to set string length")

if __name__ == '__main__':
  rospy.init_node('XBee_Bridge')
  
  com_port = rospy.get_param('com_port')
  baud_rate = rospy.get_param('baud_rate')
  com_type = rospy.get_param('com_type')
  fake_agent = rospy.get_param('fake_agent')
  agent_index = rospy.get_param('agent_index')

  rospy.logwarn("XBee Bridge::initializing")
  rospy.logwarn(" XBee Bridge::com_port: %s", com_port)
  rospy.logwarn(" XBee Bridge::baud rate: %i", baud_rate)
  rospy.logwarn(" XBee Bridge::com_type: %s", com_type)
  rospy.logwarn(" XBee Bridge::fake_agent: %s", fake_agent)
  rospy.logwarn(" XBee_Bridge::agent_index: %i", agent_index)
  xbee = XBee(com_port, baud_rate, com_type, fake_agent, agent_index)
  rospy.logwarn("xbee Bridge::initialized")
  rospy.spin()
  
