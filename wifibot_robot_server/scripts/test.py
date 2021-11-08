#!/home/pierre/PycharmProjects/robo-gym/venv/bin/python
import rospy
#Importing Twist message: Used to send velocity to Turtlesim
from geometry_msgs.msg import Twist
import random

#Function to move turtle: Linear and angular velocities are arguments
class move_wifibot:
	def __init__(self):
		rospy.init_node('move_wifibot')
		self.pub = rospy.Publisher('/velocity_controller/cmd_vel', Twist, queue_size=10)
		self.rate = rospy.Rate(10) # 10hz

	def move(self):
		while not rospy.is_shutdown():
			vel = Twist()
			lin_vel = random.uniform(-1,1)
			ang_vel = random.uniform(-1,1)
			vel.linear.x = lin_vel
			vel.angular.z = ang_vel
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