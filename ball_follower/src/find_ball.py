#!/usr/bin/env python

#Rospy and Sub/Pub Libraries
import rospy
import geometry_msgs
from geometry_msgs.msg import Point
import sensor_msgs
from sensor_msgs.msg import CompressedImage

#Synthetic Data Libraries
import scipy.stats as stats
import numpy as np


#Matplotlib Libraries
import matplotlib.pyplot as plt

#Computer Vision Libraries
import cv2
from cv_bridge import CvBridge, CvBridgeError

import time

class LocateBall:
	x = 0
	y = 0
	z = 0
	circles = [[],False]
	sim = False
	
	def __init__(self,simulator=False):
		#Setup pubs and subs
		self.pub = rospy.Publisher('/ball_loc',Point,queue_size=1)
		if not simulator:
			img = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage,self.ImgCallback)
		else:
			img = rospy.Subscriber("/usb_cam/image/compressed",CompressedImage,locate_ball)
		self.sim = simulator

		#First step is to init bridge class through CV
		self.bridge = CvBridge()

	def ImgCallback(self,CVData):
		curr_img = self.bridge.compressed_imgmsg_to_cv2(CVData, "bgr8")
		
		#processing here
		self.circles[1] = self.LocCircles(curr_img)
		
		
	def LocCircles(self,img):
		#processed image
		pimg = img
		pimg = cv2.medianBlur(pimg,5)
		kernel = np.ones((5,5))/25
		pimg = cv2.filter2D(pimg,-1,kernel)
		#_,pimg = cv2.threshold(pimg,100,255,cv2.THRESH_BINARY)
		pimg = cv2.cvtColor(pimg,cv2.COLOR_BGR2GRAY)
		#pimg = cv2.Canny(pimg,100,150)
		
		
		circs = cv2.HoughCircles(pimg,cv2.HOUGH_GRADIENT,1,20,param1=30,param2=40,minRadius=15,maxRadius=0)
		
		
		if circs is not None:
			print(str(time.time()) + ': Found a circle!')
			clist = np.uint16(np.around(circs))
			
			for ii in circs[0,:]:
				cv2.circle(pimg,(ii[0],ii[1]),ii[2],(255,255,0),2)
				cv2.circle(pimg,(ii[0],ii[1]),2,(255,0,255),3)
			
			#generate a uniform random and only show "every fourth frame"
			runi = np.random.uniform(0,1)
			if runi < -1:
				cv2.imshow('detected circles',pimg)
				cv2.waitKey(1)
				
			coords = clist[0][0]
			h,w = pimg.shape[:2]
			self.x = coords[0]/float(w)
			self.y = coords[1]/float(h)
			self.z = coords[2]/(float(w)*float(h))
				
			foundball = True
		else:
			#print(str(time.time()) + 'No Circles Found')
			#if we don't find a circle, we should just pause where we are
			self.x = 10
			self.y = 10
			self.z = 10
				
			foundball = False
			
		cv2.imshow('InImage',pimg)
		cv2.waitKey(1)
		
		return foundball
			
	def pub_coord(self):
		self.pub.publish(Point(self.x,self.y,self.z))
		
if __name__ == '__main__':
	try:
		mainFind = LocateBall()
		
		rospy.init_node('BallLocate')
		rate = rospy.Rate(30)
		
		while not rospy.is_shutdown():
			if mainFind.circles[1] != False:
				mainFind.pub_coord()
			else:
				pass
				mainFind.pub_coord()
				#print(str(time.time()) + 'Bypass Pub')
			rate.sleep()
		cv2.destroyAllWindows()
	except rospy.ROSInterruptException:
		pass

#Below is not used anymore! But keeping so can move into external library
#Can also integrate with webcam code/simulator to make a synthetic flow
def gen_synth_IMG():
	#define our image
	#ball_img = np.zeros((800,600),dtype=np.uint8)

	ball_img = np.random.normal(10,5,size=(800,600))
	#Add in a noisy sphere
	rcent = np.random.uniform(0,1,size=(2))
	rcent[0] = int(rcent[0] * 800)
	rcent[1] = int(rcent[1] * 600)

	rcent = rcent.astype(int)
	rrad = np.abs(int(np.random.normal(10,30)))
	rr,cc,val = circle_perimeter_aa(rcent[0],rcent[1],rrad)
	rr[rr>800] = 800
	ball_img[rr,cc] = val * 255

	plt.figure()
	plt.imshow(ball_img)
	plt.show()

	return ball_img
