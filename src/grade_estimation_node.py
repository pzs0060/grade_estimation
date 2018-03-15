#!/usr/bin/env python
import rospy
import sys # what does this do?
import numpy as np 
from math import pi, atan2
from cacc_msgs.msg import GradeEstimationOutput #From gitlab msg header
from grade_estimation import grade_estimation #Import python class 
from nav_msgs import Odometry # Import GPS Vel msg
from wgs_conversions.srv import WgsConversion # Import xyz2enu_vel

# Declare class for ROS node
class grade_estimation_node():
	def __init__(self):
		
		# Initialize class instance
		self.grade_est = grade_estimation()

		## Parameters

		# Set up Client to use Service
		rospy.wait_for_service('xyz2enu_vel')
		rospy.wait_for_service('xyz2lla')

		# ROS
		gps_topic = rospy.get_param('~gps_topic', '/novatel_node/odom')
		queue_size = rospy.get_param('~queue_size', 1)
	
		# Inialize Publisher and Subscriber
		self.pub = rospy.Publisher("~grade_estimate", GradeEstimationOutput,queue_size = queue_size)
		gps_sub = ropspy.Subscriber(gps_topic, Odometry, self.GpsMeasCallback, queue_size = queue_size)




	

	# Define Callback when msg is recieved from topic we subscribed
	def GpsMeasCallback(self,msg)

		# Extract data from GPS msg
		x_vel = msg.twist.twist.linear.x
		y_vel = msg.twist.twist.linear.y 
		z_vel = msg.twist.twist.linear.z

		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		z = msg.pose.pose.position.z

		# Convert ref from xyz to lla
		xyz2lla = rospy.ServiceProxy('xyz2lla', WgsConversion)	
		rsp = xyz2lla(xyz=(x,y,z))
		(lat,lon,alt)=rsp.lla
		ref_lat = lat
		ref_lon = lon


		# Convert from xyz to enu
		xyz2enu_vel = rospy.ServiceProxy('xyz2enu_vel', WgsConversion)	
		rsp = xyz2enu_vel(xyz=(x_vel,y_vel,z_vel),ref_lla=(ref_lat,ref_lon,ref_alt))
		(ve,vn,vu)=rsp.enu

		grade_estimate = self.grade_est.calcGrade(east_vel = ve, north_vel = vn, up_vel = vu)

		# Create msg to publish
		grade_msg = GradeEstimationOutput()
		grade_msg.grade_estimate = grade_estimate
		grade_msg.header.stamp = rospy.get_rostime()

		# Publish msg
		self.pub.publish(grade_msg)


def main(args):
	rospy.init_node('grade_estimation_node')

	node = grade_estimation_node()

	try:
		ropspy.spin()

	except KeyboardInterrupt:
		print("Grade Estimation is Shutting Down")

	if __name__ == '__main__':
		main(sys.argv)