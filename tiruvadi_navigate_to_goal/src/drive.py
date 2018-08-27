#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Twist, Pose
import numpy as np
from nav_msgs.msg import Odometry
from collections import defaultdict

#Angular controller
#moves a state vector to coincide with the target vector

class angle_ctrl:
	current_angle = 0
	init_angle = 0
	target_angle = 0

	def __init__(self,init_offset):
		#initialize the angle controller
		self.init_angle = init_offset
	
	def actuate(self):
		#difference between current and target angle
		diff_angle = init_angle - current_angle

		#we want to minimize this diff_angle -> 0
		#so we do that by MOVING physically
		
class lin_ctrl:
	zeropt = []

	def __init__(self,init_offset):
		self.zeropt = init_offset
		

class Driver:
	pubR = []
	odom = []
	active = 0

	active_controller = []

	obst_info = []

	active_wp = 0

	speed = 0.1

	def __init__(self):
		rospy.init_node('tb_driver', anonymous=True)
		rospy.Subscriber('/odom',Odometry,self.update_odo)
		#rospy.Subscriber('/obst_info',ObstObj,self.obst_state_assess)

		self.rate = rospy.Rate(30)
		self.pubR = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
		self.t_zero = rospy.get_time()
		self.obst_info = self.init_Obst()
		self.read_waypoint()

	def init_Obst(self):
		#encode our priors here
		#Text reading will be done here in order to establish the overall landscape of the course
		#active measurements will either add onto this or pre-empt it
		pass

	def obst_state_assess(self,obsobj):
		#when this callback is activated we want to update the class's obstacle information
		pass

	def iTwist(self,lx=0,az=0):
		twist = Twist()

		twist.linear.x = lx
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = az

		return twist

	def landscape(self):
		#priors on the landscape being navigated
		
		#break out into sections
		pass

	def state(self):
		#return the state in the context of the landscape
		pass

	def update_odo(self,odobj):
		#if we're at the initialization phase, with the first run of the odo, then we want to set up our basis transformations
		if self.active == 0:
			self.odom_zero = odobj.pose.pose
			self.active = 1
			self.theta_0 = np.arccos(odobj.pose.pose.orientation.w) * 2

		self.odom = odobj.pose.pose


	def get_pos(self):
		#this function takes current odobj pose and subtracts the initial
		
		x = self.odom.position.x - self.odom_zero.position.x
		y = self.odom.position.y - self.odom_zero.position.y

		rot_matr = [[np.cos(self.theta_0),-np.sin(self.theta_0)],[np.sin(self.theta_0),np.cos(self.theta_0)]]

		rot_matr = np.array(rot_matr)
		vect = np.array([x,y])

		rvect = np.dot(rot_matr,vect)
		
		
		return rvect

	def get_ori(self):
		#theta_zero = np.arcsin(self.odom_zero.orientation.w) * 2
		
		theta_now = np.arccos(self.odom.orientation.w) * 2
		theta_oth = np.arcsin(self.odom.orientation.z) * 2

		outsign = 1 * np.sign(theta_now) * np.sign(theta_oth)

		return outsign * (theta_now - self.theta_0)

	def get_dir(self):
		#return a unit vector with the direction wrt to the x'y' basis
		theta = self.get_ori()
		unit = [np.cos(theta),np.sin(theta)]
		return unit

	def curr_time(self):
		return rospy.get_time() - self.t_zero

	def read_waypoint(self):
		#file to read in waypoint data
		
		#set the waypoint list
		self.waypoints = [(2,0),(2,3)]


	def gen_pot(self,waypoint_vec,curr_vec):
		#generate a potential well for the current waypoint
		X = np.arange(-5,5,0.1)
		Y = np.arange(-5,5,0.1)

		xc = waypoint_vec[0]
		yc = waypoint_vec[1]
		var = 10

		self.X,self.Y = np.meshgrid(X,Y)

		#self.pot = -0.5 * np.exp(-((self.X - xc)**2/(2*var**2) + (self.Y - yc)**2/(2*var**2)))
		xdir = 1
		#assess the gradient of the potential at the point we current are

	def run(self):
		while not rospy.is_shutdown():
			state = defaultdict(dict)
			#print('Current Time: ' + str(self.curr_time()))
			#First, let's assess our environment and get an idea of whether our priors are appropriate
			#If we have an obstacle, we then switch out of our intrinsic controller and into obstacle avoidance
			curr_time = self.curr_time()
	
			#this is where we focus on the active controller
			#state['time'] = curr_time
			#state['posit'] = [self.odom.position.x,self.odom.position.y]
			if self.active:

				#Priority is being angled toward the next waypoint
				#dot product of target and current_vel
				current_targ = self.waypoints[self.active_wp]
				vect_to_targ = (current_targ - self.get_pos())
				current_dir = self.get_dir()

				angle_off_targ = np.arccos(np.dot(current_dir,current_targ) / np.linalg.norm(current_targ))
				#print(self.get_ori())

				#If the orientation is above some threshold, then we will prioritize the angular shift
				#compare orientation vector in global frame to vect_to_targ in global frame, if it's above an angle, prioritize rotating
				angle_off = np.arccos(np.dot(self.get_dir(),vect_to_targ) / (np.linalg.norm(self.get_dir()) * np.linalg.norm(vect_to_targ)))
				#print(self.get_dir())
				if np.abs(angle_off) > 0.05:
					print('Rotating and correcting: ' + str(angle_off))
					twist = self.iTwist(az = 1*np.sign(self.get_ori()))

				#print('Pos: ' + str(self.get_pos()) + ' Ori: ' + str(self.get_ori()))
				else:
					#check how far we are from the target
					dist_to_targ = np.linalg.norm(vect_to_targ)
					if dist_to_targ < 0.1:
						#increment the waypoint after a 2 second pause
						self.active_wp += 1
						print('Increment to waypoint ' + str(self.active_wp))
					else:
						#move us closer
						
						print('Distance to Target: ' + str(dist_to_targ))
						twist = self.iTwist(lx = self.speed)

				self.pubR.publish(twist)
			self.rate.sleep()

if __name__ == '__main__':
	dv = Driver()
	dv.run()
