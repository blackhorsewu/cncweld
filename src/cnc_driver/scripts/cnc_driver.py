#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from cnc_class import cnc

cnc_obj = cnc()

def cmdCallback(msg):

	# rospy.loginfo(rospy.get_name() + ": " + str(msg))
	# print("************************************** Received command with position as below:")
	# print( msg.linear.x, msg.linear.y, msg.linear.z)
	cnc_obj.moveTo(msg.linear.x, msg.linear.y, msg.linear.z, blockUntilComplete=True)
	# print("Finished moveTo")

def stopCallback(msg):
		#stop steppers
	if   msg.data == 's':
		cnc_obj.disableSteppers()
		# fire steppers
	elif msg.data == 'f':
		cnc_obj.enableSteppers()

def main():

	''' create ROS topics '''
	pos_pub     = rospy.Publisher('/grbl_pos',  Twist, queue_size = 10)
	status_pub  = rospy.Publisher('/grbl_status'  , String, queue_size = 10)
	rospy.Subscriber('grbl_cmd' ,  Twist,  cmdCallback)
	rospy.Subscriber('grbl_stop', String, stopCallback)

	rospy.init_node('cnc_driver', anonymous=True)

	port          = rospy.get_param('cnc_driver/port')
	baud          = rospy.get_param('cnc_driver/baudrate')
	acc           = rospy.get_param('cnc_driver/acceleration')  
	min_x 		  = rospy.get_param('cnc_driver/x_min')   
	min_y 		  = rospy.get_param('cnc_driver/y_min')   
	min_z 		  = rospy.get_param('cnc_driver/z_min')
	max_x 		  = rospy.get_param('cnc_driver/x_max')   
	max_y 		  = rospy.get_param('cnc_driver/y_max')   
	max_z 		  = rospy.get_param('cnc_driver/z_max')
	default_speed = rospy.get_param('cnc_driver/default_speed')   
	speed_x  	  = rospy.get_param('cnc_driver/x_max_speed')
	speed_y  	  = rospy.get_param('cnc_driver/y_max_speed')
	speed_z  	  = rospy.get_param('cnc_driver/z_max_speed')
	steps_x 	  = rospy.get_param('cnc_driver/x_steps_mm')
	steps_y 	  = rospy.get_param('cnc_driver/y_steps_mm')
	steps_z 	  = rospy.get_param('cnc_driver/z_steps_mm')

	cnc_obj.startup(port,baud,acc,min_x,min_y,min_z,max_x,max_y,max_z,default_speed,speed_x,speed_y,
					speed_z,steps_x,steps_y,steps_z)
	# rate = rospy.Rate(10)
	rate = rospy.Rate(3.3)

	while not rospy.is_shutdown():
		# get the necessary data and put them into ROS format
		# print("Beginning at top loop")
		status     = cnc_obj.getStatus()
		# print("Gotten status")
		cnc_pose   = cnc_obj.getTwist()
		# print("Gotten position")
		ros_status = String(status)
		# After the data is formatted and ready,
		# then publish them.
		# print("Going to publish position")
		# print(cnc_pose)
		pos_pub.publish(cnc_pose)
		# print("Going to publish status")
		status_pub.publish(ros_status)
		# go around this loop in the right speed of 10 Hz
		rate.sleep() 

	rospy.spin()

main()
