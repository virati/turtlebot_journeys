#!/usr/bin/env python
#Vineet Tiruvadi
#Lab 2 for Intro Robotics

import rospy
from geometry_msgs.msg import Twist, Point
import numpy as np

import sys, select, termios, tty

class Driver:
	sendVel = np.array([0,0,0])
	
	def __init__(self):
		self.ballSub = rospy.Subscriber("/ball_loc",Point,self.mover)
		self.VelPub = rospy.Publisher("/cmd_vel",Twist,queue_size=5)
	
	def mover(self,inPoint):
		
		#INPUT HERE IS A POINT
		#inCoord = np.array(inPoint)
		#Check if the point we're looking for is normalized
		#assert inCoord.any() <= 1
		twist=Twist()
		inX = inPoint.x
		
		print(inX)

		if inX <= 1:
			#Center to the screen
			inX = inX - 0.5
			
			#since we're JUST TURNING FOR NOW, we'll focus on the x coord
			targ = inX
			
			print('X ball: ' + str(targ))
			
			t_av = 0
			c_av = 0
			
			#set target_angular_vel; still just velocity
			#if we want to go to the ball:
			
			t_av -= np.sign(targ) * 0.1
			
			#if we want to be scared of the ball: but can also collapse into single var and multily above
			
			#t_av += np.sign(targ) * 0.1
			
			
			#is target Ang Vel > control ang vel?
			c_av = t_av
			
		else:
			c_av = 0
			
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
		twist.angular.x = 0;twist.angular.y;twist.angular.z = c_av;
			
		print('Publishing ' + str(c_av))
		self.pub_vel(twist)
		
	def pub_vel(self,twist):
		self.VelPub.publish(twist)
		
		
if __name__== "__main__":
	try:
		rospy.init_node('WheelDriver')
		mainDrv = Driver()
		rate = rospy.Rate(30)
		
		while not rospy.is_shutdown():
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
		
