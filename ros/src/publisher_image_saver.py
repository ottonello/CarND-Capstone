import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import time
from random import randint

def publish():
	""" Publish a single message to the /do_capture topic, so the image capture is taken by image_saver"""
	rospy.init_node('random_number')
	pub=rospy.Publisher('/do_capture', Int32, queue_size=10)
	while not pub.get_num_connections():
		pass
	pub.publish(10)

if __name__=='__main__':
	publish()