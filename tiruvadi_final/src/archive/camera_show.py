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

import SignClassif as SCl

import cv2
from cv_bridge import CvBridge, CvBridgeError

print(cv2.__version__)

class VizSys:
    lastframe = []
    activeMove = 1
    activeImg = True
    up_wall = 0
    #Whether a sign if visible    
    sign_viz = 0
    active_move = []
    img_assess = []

    def waypt_struct(self,vect):
        posestamp = PoseStamped()
        posestamp.header.seq = 0
        posestamp.header.stamp.secs=0
        posestamp.header.stamp.nsecs=0
        
        posestamp.header.frame_id = '/map'
        posestamp.pose.position.x = vect[0]
        posestamp.pose.position.y = vect[1]
        posestamp.pose.position.z = vect[2]
        
        posestamp.pose.orientation.x = vect[3]
        posestamp.pose.orientation.y = vect[4]
        posestamp.pose.orientation.z = vect[5]
        posestamp.pose.orientation.w = vect[6]

        return posestamp
        
    def lidarCallB(self,LD):
        #if we're up against the wall, then activate imgCallB
        dists = list(LD.ranges)
        #check directly in front to see if there is something close by
        #print(dists[0])
        #cloud = self.LaserProj.projectLaser(LD)
        #print(LD[540])
        #print(cloud[540])

        if self.toggle_lidar_calcs:
    
            if dists[0] <= 0.3:
                #if a sign if visible, then we set up the next decision
                self.up_wall = 1
                self.toggle_lidar_calcs = False
            
            if self.up_wall:
                self.activeMove = 0
                self.activeImg = 1
    
    def imgCallB(self,inImg):
        if self.activeImg:
            curr_img = self.bridge.imgmsg_to_cv2(inImg,"bgr8")
            croppedimg = self.crop_img(curr_img)
            
            ret,ress,neigh,classdist = self.knns.findNearest(croppedimg.flatten().reshape(1,25*20).astype(np.float32),3)
            print('IMG Return class: ' + str(ret))
            
            cv2.imshow(str(ret),croppedimg)
            cv2.waitKey(1)
            print('Distance to class: ' + str(classdist))
            #need to make sure that the classifier found something reasonable before we adopt it:
            if classdist.any() < 10:
               
                #let's try to get some consistentcy on this
                if len(self.img_assess) > 10:
                    #self.active_cmd = stats.mode(self.img_assess) #get the mode of the class
                    self.img_assess = []
                    #since we have a consensus on the class, we don't want to restart active imaging yet
                    #for debugging, just comment this to ensure we can just check classification of all signs                    
                    #self.activeImg = 0
                    self.next_class = stats.mode(self.img_assess)
                else:
                    self.img_assess.append(ret)
                    self.next_class = 0
                    
            else:
                #if it wasn't stable/reasonable, then set the list back to zero
                self.img_assess = []
                
            #cv2.destroyAllWindows()
            #self.lastframe = curr_img
    def iTwist(self,lx=0,az=0):
        twist = Twist()

        twist.linear.x = -lx
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = az

        return twist
        
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('CameraShow')

        self.rate = rospy.Rate(30)
        #train our KNN
        print('Training KNN')

        self.knns = SCl.trainKNN()
        #input('Press enter')
        #self.LaserProj = LaserProjection()
        
        rospy.Subscriber("/raspicam_node/image/compressed",Image,self.imgCallB)
        rospy.Subscriber('/scan',LaserScan,self.lidarCallB)    
        #rospy.Subscriber('/move_base/local_costmap/costmap',)
        
        self.navpub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=2,latch=True)
        self.dirmovepub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        
        #current active cmd is NOTHING
        #This SHOULD correspond to the zero classifier label
        self.active_cmd = 0
        self.toggle_lidar_calcs = 1
        self.next_class = 0
        self.nav_goal = 1
        
        self.new_nav = 1
        self.nav_list = [[6.34,0.044,0.00,0.0,0.0,-0.7,0.7],[6.294,-1.17,0,0,0,0.78,0.7]]
        self.done = 0
        
    def run(self):
        self.running = 1
        while not rospy.is_shutdown():
            twist = self.iTwist()
            #Check is lidar says we're AT A WALL and if the image classifier also demonstrates a next command
            if not self.done:
                print('Not Done Yet')
                if self.next_class != 0:
                    print('Next Class is nonzero')
                    if self.next_class == 4:
                        #this is a STOP
                        self.new_nav = 1
                        self.nav_goal += 1
                    elif self.next_class == 5:
                        #we're done!
                        self.done = 1
                        
                    self.new_nav = True
                    
                if self.new_nav:
                    #if we do, we're going to add a navigation in the direction of the sign_viz class
                    #go in the direction of the active command
                    if 0:                    
                        if self.active_cmd == 1:
                            print('Go Left')    
                            nav_waypoint = [-1.8,1.8,0,1.8,1.8,1.8,0]
                        elif self.active_cmd == 2:
                            print('Go Right')                        
                            nav_waypoint = [-1.8,1.8,0,1.8,1.8,1.8,0]
                        elif self.active_cmd == 3:
                            print('Go Forward')
                            nav_waypoint = [-1.8,1.8,0,1.8,1.8,1.8,0]
                        else:
                            nav_waypoint = [-1.8,1.8,0,1.8,1.8,1.8,0]
                    else:
                        nav_waypoint = self.nav_list[self.nav_goal]
                        print('Defaulting to prior nav goal')
                    #set the waypoint as a structure
                    pub_waypoints = self.waypt_struct(vect=nav_waypoint)
                    #publish the nav                    
                    self.navpub.publish(pub_waypoints)
                    #navigation goal has been entered and will 
                    self.new_nav = 0
                    
                elif not self.new_nav and self.next_class == 0:
                    #this is if there is not a new nav AND the next_class is still zero
                    twist.angular.x = 0.1
                    
                else:
                    #this means we're just in a good spot and should keep doing
                    pass
                
            #self.dirmovepub.publish(twist)
            self.rate.sleep()
   
    def crop_img(self,img,plot=False):
        #plt.figure()
        #ify ou want to pre-crop - 
        img = img[:,:,1]
        #if not
        #img = stack[num,:,:,0]
        #cimg = stack[num,:,:,:]
        
        #find the color mask
        #mask = np.abs(cv2.inRange(cimg,np.array([105,100,110]),np.array([130,130,140])))
        
        #plt.figure()
        #plt.imshow(mask)
        #whiteidx = np.where(red.any(axis=1) > 100 and green.any(axis=1) < 120)
        #whiteidx = np.where(np.logical_and(red > 100,red < 120) and np.logical_and(green > 100, green < 120) and np.logical_and(blue > 100, blue < 120))  
        
        img = cv2.resize(img,(60,60),interpolation=cv2.INTER_CUBIC)    
        #img_filt = cv2.GaussianBlur(img,(3,3),0)
        img_filt = cv2.medianBlur(img,5)
        
        
        #different approach with adaptive thresholding    
        th3 = cv2.adaptiveThreshold(img_filt,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,3)    
        #th3 = abs(th3 - 255)    
        
        #conts,heirs = cv2.findContours(th3,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #THIS WORKSconts,heirs = cv2.findContours(th3,5,5)
        _,conts,heirs = cv2.findContours(th3,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        
        #just take the top contour??    
        #cnt = conts[0]
        #cycle into contours
        cnts = sorted(conts,key=cv2.contourArea,reverse=True)[:10]
        
        boxcnt = 0
        for c in cnts:
            peri = cv2.arcLength(c,True)
            approx = cv2.approxPolyDP(c,0.02*peri,True)
            
            if len(approx) == 4:
                boxcnt = approx
                
        if len(cnts) > 0:
            hull = cv2.convexHull(cnts[0])
        
            x,y,w,h = cv2.boundingRect(hull)
            crop = img[y:y+h,x:x+w]
        else:
            #        edges = cv2.Canny(img_filt,100,200)
            #        ct2,heirs2 = cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
            #        cnts2 = sorted(ct2,key=cv2.contourArea,reverse=True)[:10]
            #        hull2 = cv2.convexHull(cnts2[0])
            #        for c in cnts2:
            #            peri = cv2.arcLength(c,True)
            #            approx = cv2.approxPolyDP(c,0.02*peri,True)
            #            if len(approx) == 4:
            #                boxcnt2 = approx
            #                hull2 = cv2.convexHull(boxcnt2)
            #                
            #        x,y,w,h = cv2.boundingRect(hull2)
            y = img.shape[1]/4
            x = img.shape[0]/4
            
            crop = img[y:y+25,x+20]            
        
        
        if plot:
            plt.figure()
            plt.subplot(311)
            plt.imshow(img)
            plt.subplot(312)
            plt.imshow(crop,cmap='gray',interpolation='bicubic')
            plt.subplot(313)
            plt.imshow(th3)
            plt.colorbar()
            #cv2.imshow('Test',fulltrain_rgb[0,:,:,cc])
                
            
            plt.show()
        
        #RETURN THE CROPPED PICTURE!!!
        #maybe even return a flag for the feature?
        #flag the ones that are likely nonsense
        #nonsense = False
        #if crop.shape != (25,20):
        #    nonsense = True
        #sumred = np.sum(crop.flatten())
        crop = cv2.resize(crop,(25,20))
        
        return crop

            

if __name__ == '__main__':
    viz = VizSys()
    viz.run()

