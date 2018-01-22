import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import time
from random import randint

#define the random_number Publisher
def random_number_publisher():
	rospy.init_node('random_number')
	pub=rospy.Publisher('/do_capture', Int32, queue_size=10)
	# rate= rospy.Rate(1)
	while not pub.get_num_connections():
		pass
	pub.publish(10)

		# rate.sleep()
    # //generate a random number at every 2 seconds
	# while not rospy.is_shutdown():
		# random_msg=randint(0,5000)
		# rospy.loginfo(random_msg)
		# pub.publish(random_msg)
		# rate.sleep()

if __name__=='__main__':
	random_number_publisher()