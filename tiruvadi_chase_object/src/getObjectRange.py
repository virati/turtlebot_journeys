#!/usr/bin/env python
#Vineet getObjRange

import rospy
import os
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist

class findRange:
    def __init__(self):
        self.pub = rospy.Publisher('objRange',Point,queue_size=10)
        
        rospy.Subscriber("/scan",LaserScan,self.laser_proc)
        rospy.Subscriber("/imageLocation",Point,self.img_callback)
        self.obj_loc = 0
        
    def img_callback(self,img_cntr):
        self.obj_loc = img_cntr.x
        
    def laser_proc(self,Ldata):
        idx = 160
        distances = list(Ldata.ranges)
        #range-based distance
        dist = 0
        
        #print the minimum value in Ldata
        #print(np.min(distances[distances>0.0]))
        print(self.obj_loc)
        #print(len(distances))
        if self.obj_loc > 160:
            idx = int((359 - (self.obj_loc-160)*0.194375))
            if distances[idx] <= 0.9:
                dist = distances[idx]
        elif self.obj_loc < 160:
            idx = int((160-self.obj_loc) * 0.194375)
            if distances[idx] <= 0.9:
                dist=distances[idx]
        
        #else:
        #    if not distances[0] > 0.6:
        #        dist=distances[0]
            
        #now publish the information
        
        if dist == 0:
            dist = 9999
        print(dist)        
        self.pub.publish(Point(dist,0,0))
    
if __name__ == '__main__':
    rospy.init_node('getObjectRange',anonymous=True)
    print('Starting getObjectRange')
    ranger = findRange()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()
