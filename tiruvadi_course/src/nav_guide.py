#!/usr/bin/env python
#Vineet Tiruvadi Code
#Lab 5 node for ECE Intro Robotics

import rospy
from geometry_msgs.msg import Point, Twist, PoseStamped
import numpy as np
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult
import pandas as pd
import time

class NavG:
	pubR = []
	active = 0

	obst_info = []
	active_wp = 0

	speed = 0.09
	wp_just_switch = []
	navstatus = []

	def __init__(self):
		rospy.init_node('tb_driver', anonymous=True)
		#rospy.Subscriber('/odom',Odometry,self.update_odo)
		rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.curr_status)
		self.pubR = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=2,latch=True)

		self.rate = rospy.Rate(30)
		
		self.t_zero = rospy.get_time()
		self.waypoints = self.read_waypoint_file()
		self.wp_just_switch = 1
		self.navstatus = 0
		self.last_stat1 = rospy.get_time()
		self.active = 1
		
	def curr_time(self):
		return rospy.get_time() - self.t_zero

	def curr_status(self,res):
		print(res)
		self.navstatus = 1
		
	def read_waypoint_file(self):
		#file to read in waypoint data
	
		#with open('/home/rosu/global_waypoints.txt','r') as f:
			#reader = csv.reader(f,delimiter=',')
			#print(reader)
			#for a,b,c,d,e,f,g in reader:
				#waypoints.append(np.array([a,b,c,d,e,f,g]))
		#better way
		fname = open('/home/rosu/global_waypoints.txt','r')
		waypoints = np.array(pd.read_csv(fname,sep=',',header=None,engine='python'))/2
		print(waypoints)
		#set the waypoint list
		return waypoints
		
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
		
	def run(self):
		self.active_wp = 0
		while not rospy.is_shutdown() and self.active:
			curr_wp = self.waypoints
			if self.active_wp >= self.waypoints.shape[0]:
				self.active = 0
			#what waypoints are we at?
			#is this the first time we're seeing this waypoints?
			#are we done with this nav goal?
			if self.navstatus == 1 and rospy.get_time() - self.last_stat1 > 10:
				self.active_wp += 1
				self.wp_just_switch = True
				self.navstatus = 0
				self.last_stat1 = rospy.get_time()
				time.sleep(5)
				
			if self.wp_just_switch:
				self.wp_just_switch = False
				print('Publishing Waypoint ' + str(self.active_wp))
				print(self.waypoints)
				pub_waypoints = self.waypt_struct(vect=self.waypoints[self.active_wp,:])
				self.pubR.publish(pub_waypoints)
			
			self.rate.sleep()

if __name__ == '__main__':
	nav = NavG()
	nav.run()
