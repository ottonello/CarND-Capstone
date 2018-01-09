#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np
import sys

class TLDetector(object):
    def __init__(self):
        # Start 'seeing' traffic lights only when we're some meters away!
        # - faster_rcnn starts seeing around 60m away
        self.MIN_DETECTION_DIST = 80.0

        # Detection is considered valid after 'n' repeated observations
        self.DETECTION_THRESHOLD = 3

        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.tl_positions = [Pose(position=Point(x= p[0], y= p[1])) for p in self.config['stop_line_positions']]
    
        self.traffic_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.best_waypoint = 0
        self.camera_image = None

        # Publishes the state for each traffic light, useful to get training data
        self.TrafficLightState = rospy.Publisher('/traffic_light_state', Int32, queue_size=1)

        self.current_state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.lights = []

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        detected_tl_waypoint, state = self.process_traffic_lights()
        
        if self.current_state != state:
            # Reset state
            self.state_count = 0
            self.current_state = state

        elif self.state_count >= self.DETECTION_THRESHOLD:
            # From now on we take it as a good detection
            self.last_state = self.current_state
            
            detected_tl_waypoint = detected_tl_waypoint if state == TrafficLight.RED else -1
            self.traffic_light_pub.publish(Int32(detected_tl_waypoint))
        else:
            self.state_count += 1

    def get_class_name(self, code):
        if code == TrafficLight.RED:
            return "RED"
        elif code == TrafficLight.GREEN:
            return "GREEN"
        elif code == TrafficLight.YELLOW:
            return "YELLOW"
        else:
            return "UNKNOWN"

    def euclidean_distance(self, p1, p2):
        delta_x = p1.x - p2.x
        delta_y = p1.y - p2.y
        return math.sqrt(delta_x*delta_x + delta_y*delta_y)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        best_waypoint = self.best_waypoint
        if self.waypoints is not None:
            waypoints = self.waypoints.waypoints
            min_dist = self.euclidean_distance(pose.position, waypoints[0].pose.pose.position)
            for i, point in enumerate(waypoints):
                dist = self.euclidean_distance(pose.position, point.pose.pose.position)
                if dist < min_dist:
                    best_waypoint = i
                    min_dist = dist
            self.best_waypoint = best_waypoint
            return best_waypoint

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        # Find closest stopping position to current pose
        closest_tl_dist = sys.maxint
        closest_tl_idx = None
        for i in range(len(self.tl_positions)):
            dist = self.euclidean_distance(self.tl_positions[i].position, self.pose.pose.position)
            if dist < closest_tl_dist:
                closest_tl_dist = dist
                closest_tl_idx = i

        # rospy.loginfo('Closest tl: {},  distance={}'.format(closest_tl_idx, closest_tl_dist))

        if (closest_tl_dist >= self.MIN_DETECTION_DIST):
            return -1, TrafficLight.UNKNOWN
        else:
            classification = self.light_classifier.get_classification(cv_image)
            # rospy.loginfo('Image classification={}'.format(self.get_class_name(classification)))

            closest_tl_waypoint_idx = self.get_closest_waypoint(self.tl_positions[closest_tl_idx])

            return closest_tl_waypoint_idx, classification

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
