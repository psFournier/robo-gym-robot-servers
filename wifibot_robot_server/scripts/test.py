#!/usr/bin/env python
import rospy
#Importing Twist message: Used to send velocity to Turtlesim
from geometry_msgs.msg import Twist

#Function to move turtle: Linear and angular velocities are arguments
class move_wifibot:
	def __init__(self):
		rospy.init_node('move_wifibot')
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.rate = rospy.Rate(10) # 10hz

	def move(self):
		while not rospy.is_shutdown():
			vel = Twist()
			#Adding linear and angular velocity to the message
			vel.linear.x = 0.2
			vel.linear.y = 0
			vel.linear.z = 0
			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = 0.1
			#Publishing Twist message
			self.pub.publish(vel)
			# rospy.loginfo("Linear Vel = %f: Angular Vel = %f",0.2,0.1)
			self.rate.sleep()

if __name__ == '__main__':
	try:
		mover = move_wifibot()
		mover.move()
	except rospy.ROSInterruptException:
		pass