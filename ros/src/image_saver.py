import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import PIL.Image
import time

class ImageSaver(object):
	""" 
	In theory you should be able to use rosrun image_view image_view image:=/image_color to 
	view and capture the images published to the topic but it wasn't working for me,
	so I built this small image capture tool.
	It will capture an image from /image_color whenever it receives any message on /do_capture.
	"""
	def __init__(self):
		print('hi')
		rospy.init_node('image_capture', anonymous=True)
		self.camera_image = None
		self.bridge = CvBridge()
		sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
		sub7 = rospy.Subscriber('/do_capture', Int32, self.do_capture_cb)

		rospy.spin()

	def image_cb(self, msg):
		self.camera_image = msg

	def do_capture_cb(self, msg):
		print('received {}', msg)
		if self.camera_image:
			cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
			img = PIL.Image.fromarray(cv_image)
			t = rospy.get_rostime()
			filename = 'test_{}.jpg'.format(t.to_sec())
			rospy.loginfo('Saving to {}'.format(filename))
			img.save(filename)
		else:
			rospy.loginfo('No camera_image received yet')


if __name__ == "__main__": ImageSaver()