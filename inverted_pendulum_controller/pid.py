#!/usr/bin/env python

import rospy
from inverted_pendulum_sim.msg import ControlForce, CurrentState
from math import sin, pi,cos ,tan

class SinusoidalController:
	def __init__(self):
		self.cart_x = 0.0
		self.cart_x_dot=0.0
		self.theta=0.0
		self.amplitude = 2.0
		self.frequency = 1.5
		self.distance_threshold = 200.0
		self.direction = 1
		self.distance_traveled = 0
		self.distance_traveled1 = 0
		self.sinusoidal_force=0
		self.flag = 1
		self.Kp = 550
		self.Ki = 2
		self.Kd = 1
		self.prev_error = 0
		self.integral = 0
		self.max_integral = 4
		rospy.init_node('sinusoidal_controller', anonymous=True)
		self.prev_time = rospy.get_time()
		self.control_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)
		rospy.Subscriber('/inverted_pendulum/current_state', CurrentState, self.state_callback)

		self.rate = rospy.Rate(100)  # Adjust the rate as needed

	def state_callback(self, msg):
		"""
		Callback function for handling the current state information.
		"""
		self.cart_x = round(msg.curr_x, 2)
		self.cart_x_dot = round(msg.curr_x_dot, 2)
		self.theta = round(msg.curr_theta, 2)

	def main(self):
		while not rospy.is_shutdown():
			current_time = rospy.get_time()
			elapsed_time = current_time - self.prev_time
			error=pi-abs(self.theta)
			self.integral += error*elapsed_time 
			derivative = (error - self.prev_error) / current_time
			self.prev_error = error

			self.force= self.Kp * error
			self.prev_time = current_time
			self.integral = max(min(self.integral, self.max_integral), -self.max_integral)

			if self.theta<0:
				self.publish_force(-self.force)
				if self.cart_x<=-10:
					self.publish_force(1.0)
				else:
					self.publish_force(-self.force)

					
			if self.theta>0:
				self.publish_force(self.force)
				if self.cart_x>=10:
					self.publish_force(-1.0)
				else:
					self.publish_force(self.force)

			print("THETHA-------", self.theta)

			self.rate.sleep()

	def publish_force(self, force_value):
		control_msg = ControlForce()
		control_msg.force = force_value
		self.control_pub.publish(control_msg)

if __name__ == '__main__':
	try:
		controller = SinusoidalController()
		controller.main()
	except rospy.ROSInterruptException:
		pass
