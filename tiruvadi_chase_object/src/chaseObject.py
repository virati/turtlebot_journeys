#!/usr/bin/env python

#Vineet chaseObject with actual controller

import rospy
import os
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist

class chaser:
    def __init__(self):
        #Publishing variables
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        
        #Subscribers
        rospy.Subscriber("imageLocation",Point,self.img_callback)
        rospy.Subscriber("objRange",Point,self.move_callback)
        
        self.distance = 30
        self.angl = Point(0,0,0)
        self.targ_lvel = 0
        self.targ_avel = 0
        self.objDist = 0
        
        #what range is sweet spot
        self.sweet_spot = (0.24,0.3)
        self.gain = 0.0005
        self.lidar_miss = 0
        
    #img callback is done for the angle
    def img_callback(self,img_loc):
        self.angl = img_loc
        
    def move_callback(self,objDist):
        self.objDist = objDist.x
    
    def iTwist(self,lx=0,az=0):
        twist = Twist()
        twist.linear.x = -lx
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = az

        return twist    
    
    def ctrl(self):
        
        #Linear controller for distance
        #print(self.objDist)
        if self.objDist >= self.sweet_spot[0] and self.objDist <= self.sweet_spot[1]:
            self.targ_lvel = 0
            #print('In Lin Sweet Spot!')
        elif self.objDist == 9999:
            self.lidar_miss += 1
            if self.lidar_miss > 5:
                self.targ_lvel = 0
                self.lidar_miss = 0
                
            pass
        else:
            #print(self.objDist)
            #are we above or below?
            #dir_sign = float(int(self.objDist > 0.30))
            obj_offset = self.objDist - np.mean(self.sweet_spot)
            #print(obj_offset)
            self.targ_lvel += obj_offset * self.gain
        
        #clip the target_lvel
        #print(self.targ_lvel)
        self.targ_lvel = 5 * np.tanh(self.targ_lvel)
        #print(self.targ_lvel)

        #print('L')
        print(self.targ_lvel)  
        twist = self.iTwist()
        if np.abs(self.targ_lvel) > 0.05:
            self.targ_lvel = np.sign(self.targ_lvel) * 0.05
            
        twist.linear.x = self.targ_lvel
        
        
        # Angular controller now
        #WORKS!!!        
        angl = self.angl.x/320
        self.targ_avel = 0
        if angl != 0.5 and self.angl.x != 99999:
            #angl_sign = np.sign(angl - 0.5)
            #print(angl_sign)
            self.targ_avel += (0.5 - angl)*0.9
        
        #print('A')
        #print(self.targ_avel)
        twist.angular.z = np.tanh(self.targ_avel)
        
        
        #publish the needed twist
        self.pub.publish(twist)
        
if __name__=='__main__':
    try:
        rospy.init_node('chaseObj',anonymous=True)
        print('Starting chaseObject')
        drive = chaser()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #the main controller
            drive.ctrl()
            rate.sleep()
    except rospy.ROSInterruptException:
        print('Except!')
            
            
        
