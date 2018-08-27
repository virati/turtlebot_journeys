#!/usr/bin/env python

import rospy
import geometry_msgs
from geometry_msgs.msg import Point, Twist, PoseStamped
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult
from laser_geometry import LaserProjection

from scipy import stats

import numpy as np

import matplotlib.pyplot as plt

import SG_Class as sgc

from collections import defaultdict

import cv2
from cv_bridge import CvBridge, CvBridgeError

from skimage.filter.rank import entropy
from skimage.morphology import disk

import csv
import sys
import sklearn



class NavMaze:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('NavMaze')
        self.rate = rospy.Rate(30)
        
        self.wall_info = defaultdict(dict)
        print('Training KNN')
        self.knn = sgc.knn_train()
        print('Done Training KNN')
        
        rospy.Subscriber('/raspicam_node/image/compressed',Image,self.cam_CB)
        rospy.Subscriber('/scan',LaserScan,self.lidar_CB)
        
        self.movePub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        
        self.update = True
        self.img_class = 0
        self.new_move = -1
        
    
    def lidar_CB(self,LD):
        #Check how far the front wall is
        darray = list(LD.ranges)
        
        self.wall_info['front'] = darray[0]
        self.wall_info['left'] = darray[int((160-0)*0.194375)]
        self.wall_info['right'] = darray[int((359-(160)*0.194375))]
        
    def cam_CB(self,CD):
        curr_img = self.bridge.imgmsg_to_cv2(CD,"bgr8")

        croppedimg,lrratio = sgc.crop_img(curr_img,plot=False)        
        croppedimg = cv2.resize(croppedimg,(25,20))
        
        cv2.imshow('Callback',croppedimg)
        cv2.waitKey(1)
        
        croppedimg = croppedimg.flatten().reshape(1,25*20)

        
        augci = np.hstack((croppedimg,np.array([lrratio]).reshape(-1,1))).astype(np.float32)
        
        ret,ress,neigh,classdst = self.knn.findNearest(augci,3)
        print('IMG Return Class: ' + str(ret))
        
        self.img_class = ret
        
        
    
    def iTwist(self,lx=0,az=0):
        twist = Twist()

        twist.linear.x = -lx
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = az

        return twist
        
    def run(self):
        while not rospy.is_shutdown():
            twist = self.iTwist()
            
            #Update twist based on the current goal
            
            #Check if we're up against a wall
            if self.wall_info['front'] < 0.5:

                
                #we're going to do a lot here now
                
                #What class are we?
                self.new_move = 1
                if self.img_class == 2:
                    #we're going right
                    pass
                elif self.img_class == 1:
                    #we're going left
                    pass
                elif self.img_class == 3:   
                    #we're doing something or other
                    pass
                elif self.img_class == 0:
                    print('Kinda stuck, not sure where to go next')
                    self.new_move = -1                    
                    pass
                
            else:
                twist.linear.x = 0.1
            
            if self.new_move == -1:
                twist.angular.z = np.random.uniform(-0.05,0.05)    
            
            self.movePub.publish(twist)
            
            self.rate.sleep()
                
    
if __name__ == '__main__':
    nm = NavMaze()
    nm.run()
