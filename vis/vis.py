import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import time

class Visualizer(object):
	def __init__(self):
		print('hi')
		rospy.init_node('vis', anonymous=True)

		rospy.Subscriber('/vehicle/steering_cmd', SteeringCmd, self.steer_cb)
		rospy.Subscriber('/vehicle/throttle_cmd', ThrottleCmd, self.throttle_cb)
		rospy.Subscriber('/vehicle/brake_cmd', BrakeCmd, self.brake_cb)

		rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
		rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

		self.xs = []
		self.ys = []

		self.start =  int(round(time.time() * 1000))
		self.init_plot()
		rospy.spin()

	def steer_cb(self, msg):
		self.steer = msg.steering_wheel_angle_cmd

	def throttle_cb(self, msg):
		self.throttle = msg.pedal_cmd
		millis = int(round(time.time() * 1000))
		self.xs.append(millis-start)
		self.ys.append(msg.pedal_cmd)

	def brake_cb(self, msg):
		self.brake = msg.pedal_cmd

	def twist_cb(self, message):
		self.proposed_velocity = message

	def current_velocity_cb(self, message):
		self.current_velocity = message

	def animate(self, i):
		self.ax1.clear()
		self.ax1.plot(self.xs, self.ys)

	def init_plot(self):
		fig = plt.figure()
		self.ax1 = fig.add_subplot(1,1,1)
		ani = animation.FuncAnimation(fig, self.animate, interval=1000)
		plt.show()



if __name__ == "__main__": Visualizer()