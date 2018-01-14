#! /usr/bin/env python

import rospy
import math

from sensor_msgs.msg import JointState

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM

class controller:
	def __init__(self, pinoutTable):
		self._RRdir = pinoutTable['rear_right_wheel']['IN1']
		self._RRpwm = pinoutTable['rear_right_wheel']['PWM']

		self._RLdir = pinoutTable['rear_left_wheel']['IN1']
		self._RLpwm = pinoutTable['rear_left_wheel']['PWM']

		self._FRdir = pinoutTable['front_right_wheel']['IN1']
		self._FRpwm = pinoutTable['front_right_wheel']['PWM']

		self._FLdir = pinoutTable['front_left_wheel']['IN1']
		self._FLpwm = pinoutTable['front_left_wheel']['PWM']

		GPIO.setup(self._RRdir, GPIO.OUT)
		PWM.start(self._RRpwm, 0)
		GPIO.setup(self._RLdir, GPIO.OUT)
		PWM.start(self._RLpwm, 0)
		GPIO.setup(self._FRdir, GPIO.OUT)
		PWM.start(self._FRpwm, 0)
		GPIO.setup(self._FLdir, GPIO.OUT)
		PWM.start(self._FLpwm, 0)


	def update(self, msg):

		self._velocityRR = msg.velocity[0]
		self._velocityRL = msg.velocity[1]
		self._velocityFR = msg.velocity[2]
		self._velocityFL = msg.velocity[3]

		if self._velocityRR <= 0:
			GPIO.output(self._RRdir, GPIO.HIGH)
		else:
			GPIO.output(self._RRdir, GPIO.LOW)
		PWM.set_duty_cycle(self._RRpwm, abs(self._velocityRR)*33.0)

		if self._velocityRL <= 0:
			GPIO.output(self._RLdir, GPIO.HIGH)
		else:
			GPIO.output(self._RLdir, GPIO.LOW)
		PWM.set_duty_cycle(self._RLpwm, abs(self._velocityRL)*33.0)

		if self._velocityFR <= 0:
			GPIO.output(self._FRdir, GPIO.HIGH)
		else:
			GPIO.output(self._FRdir, GPIO.LOW)
		PWM.set_duty_cycle(self._FRpwm, abs(self._velocityFR)*33.0)

		if self._velocityFL <= 0:
			GPIO.output(self._FLdir, GPIO.HIGH)
		else:
			GPIO.output(self._FLdir, GPIO.LOW)
		PWM.set_duty_cycle(self._FLpwm, abs(self._velocityFL)*33.0)

		return 0




def main():
	rospy.init_node('ioNode', anonymous=False)

	motorTable = {'front_left_wheel' : {'PWM' : 'P8_13', 'IN1' :'P8_9', 'IN2' : 'P8_7'},
'front_right_wheel' : {'PWM' : 'P8_19', 'IN1' :'P8_26', 'IN2' : 'P8_18'},
'rear_left_wheel' : {'PWM' : 'P9_14', 'IN1' :'P9_15', 'IN2' : 'P9_12'},
'rear_right_wheel' : {'PWM' : 'P9_16', 'IN1' :'P9_23', 'IN2' : 'P9_30'}}

	myMecanum = controller(motorTable)
	rospy.Subscriber('/wheels/js', JointState, myMecanum.update)

	rospy.spin() #prevents from sleeping


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException: #if this error is encountered skip it
        pass
