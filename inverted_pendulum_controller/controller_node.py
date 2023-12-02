#!/usr/bin/env python

import rospy
from inverted_pendulum_sim.msg import ControlForce, CurrentState
from math import sin, pi,cos ,tan

class SinusoidalController:
	def __init__(self):
		self.cart_x = 0.0
		self.cart_x_dot=0.0
		self.amplitude = 2.0
		self.frequency = 1.5
		self.distance_threshold = 200.0
		self.direction = 1
		self.distance_traveled = 0
		self.distance_traveled1 = 0
		self.sinusoidal_force=0
		self.flag = 1
		self.flag1 = 1
		self.flag2=1
		rospy.init_node('sinusoidal_controller', anonymous=True)
		self.control_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)
		rospy.Subscriber('/inverted_pendulum/current_state', CurrentState, self.state_callback)

		self.rate = rospy.Rate(100)  # Adjust the rate as needed

	def state_callback(self, msg):
		"""
		Callback function for handling the current state information.
		"""
		self.cart_x = round(msg.curr_x, 2)
		self.cart_x_dot = round(msg.curr_x_dot, 2)


	def limit_cart_x(self, value, lower_limit, upper_limit):
		"""
		Ensure that the value is within the specified range.
		"""
		return max(lower_limit, min(value, upper_limit))

	def main(self):
		while not rospy.is_shutdown():
			t = rospy.get_time()
			print("FLAG",self.flag1)
			if self.flag==1 :

				x_desired_1 = 170  # Adjust as needed
				x_desired_2 = 0 # Adjust as needed		
				x_desired_3 = -170  # Adjust as needed
				x_desired_4 = 0  # Adjust as needed

				# PD controller gains
				Kp = 0.2
				Kd = 0.2		
				# Calculate the control force using PD control
				self.sinusoidal_force = self.amplitude * sin(2 * pi )


				if self.cart_x <= 100 :
					error = self.cart_x - 90 # Use the desired position for oscillation
					self.force = (-Kp * error - Kd * self.cart_x_dot)
					self.publish_force(self.force)

				else:
					self.flag=2


			if self.flag==2:
				if self.cart_x >= -100 :
					error = self.cart_x + 90  # Use the desired position for oscillation
					self.force = (-Kp * error - Kd * self.cart_x_dot)
					self.publish_force(self.force)

				else:
					print("IF-----------IF")
					self.flag = 1



			# Limit cart_x to the specified range
			# self.cart_x = self.limit_cart_x(self.cart_x, -100, 100)

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
