#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
#from beginner_tutorials.msg import Num
from beginner_tutorials.msg import wheelSpeed

#from khan_msgs.msg import Quadrature
class mecanumDrive:

	def __init__(self, pub):
		self._pub = pub

		self._velocityRR = 0
		self._velocityRL = 0
		self._velocityFR = 0
		self._velocityFL = 0

	def update(self, msg):

		forward = msg.linear.x
		right = msg.angular.z

		if forward > 1.0:
			forward = 1.0
		if forward < -1.0:
			forward = -1.0

		if right > 1.0:
			right = 1.0
		if right < -1.0:
			right = -1.0			

		self._velocityRR = forward + right
		self._velocityRL = forward - right
		self._velocityFR = forward - right
		self._velocityFL = forward + right

		ws = wheelSpeed()
		ws.vel_rr = self._velocityRR
		ws.vel_rl = self._velocityRL
		ws.vel_fr = self._velocityFR
		ws.vel_fl = self._velocityFL

		self._pub.publish(ws)
		return 0

	@property
	def velocityRR(self):
		return self._velocityRR
	@property
	def velocityRL(self):
		return self._velocityRL
	@property
	def velocityFR(self):
		return self._velocityFR
	@property
	def velocityFL(self):
		return self._velocityFL
	

def main():
	rospy.init_node('MecanumNode', anonymous=False)
	wheelSpeedPub = rospy.Publisher('/wheel_speeds', wheelSpeed, queue_size=50)
	myMecanum = mecanumDrive(wheelSpeedPub)
	rospy.Subscriber("/direction", Twist, myMecanum.update)

	rospy.spin() #prevents from sleeping


if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException: #if this error is encountered skip it
        pass
