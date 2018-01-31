from styx_msgs.msg import TrafficLight
import rospy
import rospkg
import numpy as np
import os
import sys
import tensorflow as tf
from collections import defaultdict
from io import StringIO
from util import label_map_util
import time
import datetime

class TLClassifier(object):
    """
     Detect Traffic lights and their boundary boxes in images using Tensorflow Object Detection API.
     Code is based on the inference sample at:
     https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb

     Original idea and(base) training data belong to Anthony Sarkis, here's his blogpost about it:
     https://codeburst.io/self-driving-cars-implementing-real-time-traffic-light-detection-and-classification-in-2017-7d9ae8df1c58
    """
    INFERENCE_SCORE_THRESHOLD = .5

    def __init__(self):
        # Load categories
        curr_dir = os.path.dirname(os.path.realpath(__file__))
        labels_file = curr_dir + '/label_map.pbtxt'
        num_classes = 14
        label_map = label_map_util.load_labelmap(labels_file)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=num_classes,
                                                                    use_display_name=True)        
        self.category_index = label_map_util.create_category_index(categories)

        # Load inference graph
        frozen_graph_file = curr_dir + '/inference_graph/frozen_inference_graph.pb'
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(frozen_graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
     
            self.sess = tf.Session(graph=self.detection_graph, config=config)

        # Input and output tensors
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        self.debug = False

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image_expanded = np.expand_dims(image, axis=0)

        a = datetime.datetime.now()

        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores,
                 self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_expanded})


        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        # This implementation returns the last detected traffic light with enough confidence!
        detection = TrafficLight.UNKNOWN
        class_name = None
        detection_score = 0.0
        for i in range(boxes.shape[0]):
            if scores is None or scores[i] > self.INFERENCE_SCORE_THRESHOLD:
                class_name = self.category_index[classes[i]]['name']

                if class_name == 'Red':
                    detection = TrafficLight.RED
                elif class_name == 'Green':
                    detection = TrafficLight.GREEN
                elif class_name == 'Yellow':
                    detection = TrafficLight.YELLOW
                detection_score = scores[i]

        b = datetime.datetime.now()
        millis = (b - a).total_seconds() * 1000

        if self.debug:
            rospy.loginfo('TrafficLight found: {} in {} ms. - confidence={}'.format(class_name, millis, detection_score))

        return detection
