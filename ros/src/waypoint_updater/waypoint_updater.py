#!/usr/bin/env python

import rospy
import copy
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import sys
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
PUBLISH_RATE = 20

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)


        # Add State variables
        self.base_waypoints = []  # List of waypoints, as received from /base_waypoints
        self.base_wp_orig_v = []  # Original velocities of the waypoints
        self.next_waypoint = None # Next waypoint in car direction
        self.current_pose = None # Car pose
        self.red_light_waypoint = None # Waypoint index of the next red light
        self.msg_seq = 0 # Sequence number of /final_waypoints message
        self.stop_distance = 20.0
        self.accel = rospy.get_param('~target_brake_accel', -1.)     # Target brake acceleration
        
        # Launch periodic publishing into /final_waypoints
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            self.update_and_publish()
            rate.sleep()

    def update_and_publish(self):
        if self._find_next_waypoint() is not None:
            num_base_wp = len(self.base_waypoints)
            waypoint_idx = [idx % num_base_wp for idx in range(self.next_waypoint,self.next_waypoint+LOOKAHEAD_WPS)]
            final_waypoints = [self.base_waypoints[wp] for wp in waypoint_idx]
            
            try:
                red_idx = waypoint_idx.index(self.red_light_waypoint)
                if self.red_light_waypoint:
                    final_waypoints = self._decelerate(final_waypoints, red_idx, self.stop_distance)
            except ValueError:
                # No red light available: self.red_light_waypoint is None or not in final_waypoints
                red_idx = None

            # Publish final waypoints
            self.publish_msg(final_waypoints)

    def _decelerate(self, waypoints, stop_index, stop_distance):
        """
        Decelerate a list of wayponts so that they stop on stop_index
        """
        if stop_index <= 0:
            return waypoints

        new_waypoints = copy.deepcopy(waypoints)
        
        dist = self.distance(new_waypoints, 0, stop_index)
        step = dist / stop_index
        # Generate waypoint velocity by traversing the waypoint list backwards:
        #  - Everything beyond stop_index will have velocity = 0
        #  - Before that, constant (de)cceleration is applied until reaching
        #    previous waypoint velocity.
        # We assume constant distance between consecutive waypoints for simplicity
        v = 0.
        d = 0.
        for idx in reversed(range(len(new_waypoints))):
            if idx < stop_index:
                d += step
                if d > self.stop_distance:
                    v = math.sqrt(2*abs(self.accel)*(d-stop_distance))
            if v < self.get_waypoint_velocity(new_waypoints, idx):
                self.set_waypoint_velocity(new_waypoints, idx, v)

        return new_waypoints

    def _find_next_waypoint(self):
        if not self.current_pose or not self.base_waypoints:
            return

        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        wp = None
        dist = sys.maxint
        
        idx_offset = self.next_waypoint or 0
        
        # Find closest waypoint
        n_waypoints = len(self.base_waypoints)
        for i in range(n_waypoints):
            wp_d = dl(self.current_pose.position, self.base_waypoints[i].pose.pose.position)

            if wp_d < dist:
                dist = wp_d
                wp = i

        self.next_waypoint = wp
        return wp

    def publish_msg(self, final_waypoints):
        waypoint_msg = Lane()
        waypoint_msg.header.seq = self.msg_seq
        waypoint_msg.header.stamp = rospy.Time.now()
        waypoint_msg.header.frame_id = '/world'
        waypoint_msg.waypoints = final_waypoints
        self.final_waypoints_pub.publish(waypoint_msg)
        self.msg_seq += 1

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, msg):
        waypoints = msg.waypoints
        num_wp = len(waypoints)

        self.base_wp_orig_v = [self.get_waypoint_velocity(waypoints, idx) for idx in range(num_wp)]
        self.base_waypoints = waypoints


    def traffic_cb(self, msg):
        # rospy.logwarn("Got traffic waypoiny: %s", str(msg.data))
        prev_red_light_waypoint = self.red_light_waypoint
        self.red_light_waypoint = msg.data if msg.data else None
        
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoints, waypoint):
        return waypoints[waypoint].twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        """ Calculates total distance between the waypoints starting in index wp1 and ending in wp2+1  """
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
