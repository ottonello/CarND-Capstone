import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from styx_msgs.msg import Lane, Waypoint
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped, Pose

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
		rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

		self.follow_car = True
		self.half_view_size = 50
		self.xs = []
		self.ys = []

		self.last_x = 0
		self.last_y = 1000
		self.waypoints_x = []
		self.waypoints_y = []

		self.start =  int(round(time.time() * 1000))
		self.init_plot()
		rospy.spin()

	def steer_cb(self, msg):
		self.steer = msg.steering_wheel_angle_cmd

	def waypoints_cb(self, msg):
		waypoints = msg.waypoints
		self.base_waypoints = [waypoint.pose.pose.position for waypoint in waypoints]
		self.waypoints_x = [pos.x for pos in self.base_waypoints]
		self.waypoints_y = [pos.y for pos in self.base_waypoints]
		self.last_x = self.waypoints_x[0]
		self.last_y = self.waypoints_y[0]
		# fig = plt.figure()
		# self.ax1 = fig.add_subplot(1,1,1)
		# self.ax1.plot(self.waypoints_x, self.waypoints_y)
		# plt.show()

	def throttle_cb(self, msg):
		self.throttle = msg.pedal_cmd
		millis = int(round(time.time() * 1000))

	def brake_cb(self, msg):
		self.brake = msg.pedal_cmd

	def twist_cb(self, message):
		self.proposed_velocity = message

	def pose_cb(self, msg):
		self.current_pose = msg.pose
		self.last_x = msg.pose.position.x
		self.last_y = msg.pose.position.y

		self.xs.append(self.last_x)
		self.ys.append(self.last_y)


	def current_velocity_cb(self, message):
		self.current_velocity = message

	def animate(self, i):
		self.ax2.clear()
		self.ax2.set_xticks([])
		self.ax2.set_yticks([])
		self.ax2.plot(self.waypoints_x, self.waypoints_y, 'o')
		if self.follow_car:
			plt.axis((self.last_x - self.half_view_size, self.last_x+self.half_view_size, self.last_y -self.half_view_size, self.last_y+self.half_view_size))
		try:
			self.ax2.plot(self.xs, self.ys, 'r-')
		except ValueError:
			print('Differing sizes of x and y')
			pass
		# self.ax2.set_position([self.last_x, self.last_y, 1000, 1000])

		# self.ax2.set_position([0, 1000, 1000, 1000])


	def init_plot(self):
		fig = plt.figure(figsize=(20,15))
		self.ax2 = fig.add_subplot(1,1,1)
		self.ax2.set_xticks([])
		self.ax2.set_yticks([])
		ani = animation.FuncAnimation(fig, self.animate, interval=1000)
		plt.tight_layout()
		plt.show()

if __name__ == "__main__": Visualizer()