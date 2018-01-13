import rospy
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
import scipy.misc
from cv_bridge import CvBridge
import cv2

class Visualizer(object):
    def __init__(self):
        rospy.init_node('cap', anonymous=True)
        self.idx = 0
        self.previous_loop_time = rospy.get_rostime()
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        self.bridge = CvBridge()
        print('hi2')
        rospy.spin()

    def image_cb(self, msg):
        data = msg.data
        current_time = rospy.get_rostime()
        ros_duration = current_time - self.previous_loop_time
        duration_seconds = ros_duration.secs + (1e-9 * ros_duration.nsecs)

        if duration_seconds > 1:
            print('hi')
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # scipy.misc.imsave(str(self.idx )+ '.jpg', data)
            cv2.imwrite(str(self.idx )+ '.jpg', cv_image)
            self.idx += 1
            self.previous_loop_time = current_time


if __name__ == "__main__": Visualizer()
