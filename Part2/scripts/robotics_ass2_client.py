#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 13:30:00 2018

@author: baozhang
"""

from sklearn.neighbors import KDTree
import rospy
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelStates
from pqp_server.srv import *
from pyquaternion import Quaternion
from gazebo_msgs.msg import ModelState
import numpy as np
import time
from pyquaternion import *
import dijkstra as dj
import ast

class RRT_Sample():
                     
    def sample_rand(self,grid_interval):
        s_rand = []
        p = [s_rand.append( np.random.uniform(x[0],x[1],1)) for x in grid_interval]
        s_rand = [x[0] for x in s_rand]
        return s_rand
    
    def find_s_close(self,s_rand,coordinates):
    	#find the closest point to s_rand from the tree
        s_close = []
        X = coordinates
        tree = KDTree(X, leaf_size=2,metric='manhattan')
        dist, ind = tree.query([s_rand], k=1)
        s_close = X[ind[0][0]]
        return s_close,ind[0][0],dist
    
    def save_robot_state(self,msg):
        #print(msg)
        if "ackermann_vehicle" in msg.name:
            model_idx = msg.name.index("ackermann_vehicle")
            pose = msg.pose[model_idx]
            twist = msg.twist[model_idx]
            self.node_md = [pose,twist]
            #print "ackermann_vehicle"
        else:
            self.node_md = 0
            #print msg.name
        
        
    
        
    def sample_control(self,control_set):
        u = []
        u_v = np.random.choice(control_set[0],size=1,replace=True)
        u_th = np.random.uniform(control_set[1][0],control_set[1][1],size=1)
        u = [u_v[0],u_th[0]]
        return u    
    
    def sample_time(self,time_interval):
        delta_t = np.random.uniform(time_interval[0],time_interval[1],size=1)
        return delta_t[0]
    
    def set_model_state(self,modelstate):
        
        msg = ModelState()
        msg.model_name = "ackermann_vehicle"
        msg.pose = modelstate[0]
        msg.twist = modelstate[1]
        self.pub_set_model_state.publish(msg)
        
    
    def propagate_control(self,u):
    	#publish to ackermann_cmd topic
            #robot moves
        	
        	#print 'move'
        	msg = AckermannDrive()
        	msg.steering_angle = u[1]
        	msg.speed = u[0]
        	self.pub_ackermann_cmd.publish(msg)
            
    def manhattan(self,p1,p2):
        return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])
    
    def manhattan_gred(self,p1,p2):
        return abs(p1[0]-p2[0])
    
    def check_goal(self,s_goal,modelstate):
        epsilon = 4
        #check if s' close to goal
        position = modelstate[0].position
        x = position.x
        y = position.y
        #print 's_new = '+str([x,y])
        
        dis = self.manhattan([x,y],s_goal)
        print 'goal distance = '+str(dis)
        if dis<=epsilon:
            print "reach goal state"
            return True,dis
        else:
        		return False,dis
    def __init__(self, grid_interval,control_set,time_interval,s_start,s_goal):
        #grid_interval[0]=[low,high]
        sample_times = 1
        sample_prob = 0.05
        min_goal_dist = 100
        st = time.time()
        self.grid_interval = grid_interval
        sample_interval = grid_interval
                #control_set=[[a_min,0,a_max],[-pi/2,pi/2]]
        self.control_set = control_set
                #time_interval = [0,t_max]
        self.time_interval = time_interval
        self.graph = []#adjacency list
        self.coordinates = []#coordinates of nodes
        self.ms_all = []
        self.goal_state = False
        rospy.Subscriber('/gazebo/model_states', ModelStates,self.save_robot_state)
        self.pub_set_model_state= rospy.Publisher('/gazebo/set_model_state', ModelState,queue_size=50)
        self.pub_ackermann_cmd= rospy.Publisher('/ackermann_cmd', AckermannDrive,queue_size=50)
        print 'initialization finished'
        while 'node_md' not in self.__dict__:
            continue
        while self.node_md==0:
            continue
        self.ms_all.append(self.node_md)
        self.coordinates.append([self.node_md[0].position.x,self.node_md[0].position.y])
        print self.coordinates
        count = 0
        while self.goal_state==False:
            samples = []
            samples_dist = []            
            count = count+1
            print 'count = '+str(count)
            
            for i in range(sample_times):
                s_rand = self.sample_rand(sample_interval)
                samples.append(s_rand)
                samples_dist.append(self.manhattan(s_rand,s_goal))
            sam_index = samples_dist.index(min(samples_dist))
            s_rand = samples[sam_index]
            if np.random.uniform(0,1)<sample_prob:
                s_rand = s_goal
            s_close,ind,dist = self.find_s_close(s_rand,self.coordinates)
            u = self.sample_control(control_set)
            dt = self.sample_time(time_interval)
            self.set_model_state(self.ms_all[ind])
            self.propagate_control(u)
            #print 'time = '+str(dt)
            #print 'control = '+str(u)
            #print 'sclose = '+str(s_close)
            start_time = time.time()
            while time.time()-start_time<dt:
                continue
            orientation = self.node_md[0].orientation
            s = (1-orientation.w**2)**0.5
            ax = orientation.x/s
            ay = orientation.y/s
            az = orientation.z/s
            angle = 2*np.arccos(orientation.w)/3.1415*180
            print 'az = '+str(az)
            print 'angle = '+str(angle)
            if round(az)==1 and angle>0 and round(ay)==0 and round(ax)==0:
                
                self.ms_all.append(self.node_md)
                self.coordinates.append([self.node_md[0].position.x,self.node_md[0].position.y])
                self.graph.append([ind,len(self.ms_all)-1])
    
                self.goal_state,goal_dis = self.check_goal(s_goal,self.ms_all[-1])
                if min_goal_dist>goal_dis:
                    min_goal_dist = goal_dis
                #print 'goal_dis= '+str(goal_dis)
                if  min_goal_dist>7:
                    sample_prob=0.05
                if min_goal_dist<7 :
                    sample_prob=0.07
                if min_goal_dist<5:
                    sample_prob=0.1
                print sample_prob
                if self.goal_state==True:
                    #use A* to find shortest Path
                    #publish all nodes to set_model_states
                    print str(time.time()-st)
                    print self.graph
                    print self.coordinates
                    
                    index_list = self.graph                    
                    coor_list = self.coordinates
                    e_end = [len(coor_list)-1, len(coor_list)]  # append last edge connect goal to the end point
                    index_list.append(e_end)
                    coor_list.append(s_goal)
                    
                    dij = dj.Dijkstra(index_list, coor_list)
                    dij.find_rout()
                    dij.print_rout()
                    rout = dij.rout
                    dij.print_solution_quality()
                    for i in rout[:-2]:
                        self.set_model_state(self.ms_all[i])
                        rospy.sleep(1)
                                        
                    print 'Reach Goal State' 
                    
                    break
            print 
                
if __name__ == '__main__':
    rospy.init_node('ass2_client')
    s_start = [2,4]
    s_goal  = [9,5.5]
    #grid_interval[0]=[low,high]
    grid_interval = [[-9,10],[-7.5,6.5]]
    #control_set=[[a_min,0,a_max],[-pi/2,pi/2]]
    control_set = [[-3,3],[-3.1416/4,3.1416/4]]
    #time_interval = [0,t_max]
    time_interval = [0,3]
    rrt = RRT_Sample(grid_interval,control_set,time_interval,s_start,s_goal)
    
        
    
    
    
    