#!/usr/bin/env python
import rospy

#this node is meant to integrate camera and LIDAR data to determine whether there is an obstacle/object in front of the intended path

#Big picture
#This node is always returning the distance to the nearest object IN FRONT OF IT, from the camera
#coregistered with the LIDAR

class ranger:
	def __init__(self):
		rospy.init_node('ranger',anonymous=True)
		rospy.Subscriber('/scan',Scan, self.lidar_scan)
		pass


	def lidar_scan(self,scanObj):
		pass

	def cam_scan(self,camObj):
		pass

	def run(self):
		while not rospy.is_shutdown():
			obst_obj = self.integrated_assess()
			self.pubR.publish(obst_obj)
	
	def integrated_assess(self):
		#first let's register the cam

		#then let's register the lidar

		#finally, we're going to output the obstacle data
		return 0
