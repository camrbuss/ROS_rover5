#! /usr/bin/env python

import rospy
import math

from sensor_msgs.msg import JointState

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM


class controller:
	def __init__(self, pinoutTable, wheelPos):

		if wheelPos == 'RR':
			self._dir = pinoutTable['rear_right_wheel']['IN1']
			self._pwm = pinoutTable['rear_right_wheel']['PWM']
		if wheelPos == 'RL':
			self._dir = pinoutTable['rear_left_wheel']['IN1']
			self._pwm = pinoutTable['rear_left_wheel']['PWM']
		if wheelPos == 'FR':
			self._dir = pinoutTable['front_right_wheel']['IN1']
			self._pwm = pinoutTable['front_right_wheel']['PWM']
		if wheelPos == 'FL':
			self._dir = pinoutTable['front_left_wheel']['IN1']
			self._pwm = pinoutTable['front_left_wheel']['PWM']

		GPIO.setup(self._dir, GPIO.OUT)
		PWM.start(self._pwm, 0)

		self._velocity = 0

	def update(self, msg):

		self._velocity = msg.velocity

		if self._velocity <= 0:
			GPIO.output(self._RRdir, GPIO.HIGH)
		else:
			GPIO.output(self._RRdir, GPIO.LOW)
		PWM.set_duty_cycle(self._RRpwm, abs(self._velocityRR)*50.0)

		return 0




def main():
	rospy.init_node('ioNode', anonymous=False)

	motorTable = {'front_left_wheel' : {'PWM' : 'P8_13', 'IN1' :'P8_9', 'IN2' : 'P8_7'},
'front_right_wheel' : {'PWM' : 'P8_19', 'IN1' :'P8_26', 'IN2' : 'P8_18'},
'rear_left_wheel' : {'PWM' : 'P9_14', 'IN1' :'P9_15', 'IN2' : 'P9_12'},
'rear_right_wheel' : {'PWM' : 'P9_16', 'IN1' :'P9_23', 'IN2' : 'P9_30'}}

	RRcont = controller(motorTable,'RR')
	RLcont = controller(motorTable,'RL')
	FRcont = controller(motorTable,'FR')
	FLcont = controller(motorTable,'FL')

	rospy.Subscriber('/wheel_speeds', JointState, RRcont.update)


	rospy.spin() #prevents from sleeping


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException: #if this error is encountered skip it
        pass
