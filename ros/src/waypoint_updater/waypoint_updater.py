#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane, Waypoint

import math
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
max_local_distance = 20.0      # Max waypoint distance we admit for a local minimum (m)

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add State variables
        self.base_waypoints = []  # List of waypoints, as received from /base_waypoints
        self.base_wp_orig_v = []  # Original velocities of the waypoints
        self.next_waypoint = None # Next waypoint in car direction
        self.current_pose = None # Car pose
        self.msg_seq = 0 # Sequence number of /final_waypoints message

        self.accel = rospy.get_param('~target_brake_accel', -1.)     # Target brake acceleration
        self.stop_distance = rospy.get_param('~stop_distance', 5.0)  # Distance (m) where car will stop before red light
        self.force_stop_on_last_waypoint = rospy.get_param('~force_stop_on_last_waypoint', True)   # Enable/disable stopping on last waypoint
        
        # Launch periodic publishing into /final_waypoints
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            self.update_and_publish()
            rate.sleep()


    def _update_next_waypoint(self):
        """
        Update next_waypoint based on base_waypoints and current_pose.
        @return True if a valid waypoint has been updated, False otherwise
        """
        if not self.base_waypoints:
            #rospy.logwarn("Waypoints not updated: base_waypoints not available yet.")
            return False

        if not self.current_pose:
            #rospy.logwarn("Waypoints not updated: current_pose not available yet.")
            return False

        # Get ego car variables
        ego_x = self.current_pose.position.x
        ego_y = self.current_pose.position.y
        ego_theta = math.atan2(self.current_pose.orientation.y, self.current_pose.orientation.x)

        # If we do have a next_waypoint, we start looking from it, and we stop looking
        # as soon as we get a local minimum. Otherwise we do a full search across the whole track
        t = time.time()
        wp = None
        yaw = 0
        dist = 1000000 # Long number
        if self.next_waypoint:
            idx_offset = self.next_waypoint
            full_search = False
        else:
            idx_offset = 0
            full_search = True
        num_base_wp = len(self.base_waypoints)

        for i in range(num_base_wp):
            idx = (i + idx_offset)%(num_base_wp)
            wp_x = self.base_waypoints[idx].pose.pose.position.x
            wp_y = self.base_waypoints[idx].pose.pose.position.y
            wp_d = math.sqrt((ego_x - wp_x)**2 + (ego_y - wp_y)**2)

            if wp_d < dist:
                dist = wp_d
                wp = idx
            elif not full_search:
                # Local minimum. If the waypoint makes sense, just use it and break
                if dist < max_local_distance:
                    break; # We found a point
                else:
                    # We seem to have lost track. We search again
                    rospy.logwarn("Waypoint updater lost track (local min at %.1f m after %d waypoints). Going back to full search.", dist, i+1)
                    full_search = True

        if wp is None:
            rospy.logwarn("Waypoint updater did not find a valid waypoint")
            return False

        self.next_waypoint = wp
        return True


    def update_and_publish(self):
        """
        - Update next_waypoint based on current_pose and base_waypoints
        - Generate the list of the next LOOKAHEAD_WPS waypoints
        - Update velocity for them
        - Publish them to "/final_waypoints"
        """
        # 1. Find next_waypoint based on ego position & orientation
        if self._update_next_waypoint():

            # 2. Generate the list of next LOOKAHEAD_WPS waypoints
            num_base_wp = len(self.base_waypoints)
            last_base_wp = num_base_wp-1
            waypoint_idx = [idx % num_base_wp for idx in range(self.next_waypoint,self.next_waypoint+LOOKAHEAD_WPS)]
            final_waypoints = [self.base_waypoints[wp] for wp in waypoint_idx]
                
            # 3b. If we are close to the end of the circuit, make sure that we stop there
            if self.force_stop_on_last_waypoint or self.base_wp_orig_v[-1] < 1e-5:
                try:
                    last_wp_idx = waypoint_idx.index(last_base_wp)
                    self.decelerate(final_waypoints, last_wp_idx, 0)
                except ValueError:
                    # Last waypoint is not one of the next LOOKAHEAD_WPS
                    pass

            # 4. Publish waypoints to "/final_waypoints"
            self.publish_msg(final_waypoints)


    def publish_msg(self, final_waypoints):
            waypoint_msg = Lane()
            waypoint_msg.header.seq = self.msg_seq
            waypoint_msg.header.stamp = rospy.Time.now()
            waypoint_msg.header.frame_id = '/world'
            waypoint_msg.waypoints = final_waypoints
            self.final_waypoints_pub.publish(waypoint_msg)
            self.msg_seq += 1

    def pose_cb(self, msg):
        """
        Receive and store ego pose
        """
        self.current_pose = msg.pose

    def waypoints_cb(self, msg):
        """
        Receive and store the whole list of waypoints.
        """
        waypoints = msg.waypoints
        num_wp = len(waypoints)

        if self.base_waypoints and self.next_waypoint is not None:
            # Normally we assume that waypoint list doesn't change (or, at least, not
            # in the position where the car is located). If that happens, just handle it.
            if not self.is_same_waypoint(self.base_waypoints[self.next_waypoint],
                                         waypoints[self.next_waypoint]):
                self.next_waypoint = None # We can't assume previous knowledge of waypoint
                self.base_waypoints = None # Just for debugging. Will be updated later
                rospy.logwarn("Base waypoint list changed")
        else:
            # No change. We could probably return here.
            pass

        self.base_wp_orig_v = [self.get_waypoint_velocity(waypoints, idx) for idx in range(num_wp)]
        self.base_waypoints = waypoints


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass


    def restore_velocities(self, indexes):
        """
        Restore original velocities of points
        """
        for idx in indexes:
            self.set_waypoint_velocity(self.base_waypoints, idx, self.base_wp_orig_v[idx])

    def decelerate(self, waypoints, stop_index, stop_distance):
        """
        Decelerate a list of wayponts so that they stop on stop_index
        """
        if stop_index <= 0:
            return
        dist = self.distance(waypoints, 0, stop_index)
        step = dist / stop_index
        # Generate waypoint velocity by traversing the waypoint list backwards:
        #  - Everything beyond stop_index will have velocity = 0
        #  - Before that, constant (de)cceleration is applied until reaching
        #    previous waypoint velocity.
        # We assume constant distance between consecutive waypoints for simplicity
        v = 0.
        d = 0.
        for idx in reversed(range(len(waypoints))):
            if idx < stop_index:
                d += step
                if d > self.stop_distance:
                    v = math.sqrt(2*abs(self.accel)*(d-stop_distance))
            if v < self.get_waypoint_velocity(waypoints, idx):
                self.set_waypoint_velocity(waypoints, idx, v)

    def get_waypoint_velocity(self, waypoints, waypoint):
        return waypoints[waypoint].twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


    def is_same_waypoint(self, wp1, wp2, max_d=0.5, max_v=0.5):
        """
        Compare two waypoints to see whether they are the same
        (within 0.5 m and 0.5 m/s)
        """
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        ddif = dl(wp1.pose.pose.position, wp2.pose.pose.position)
        if ddif < max_d:
           return True
        return False


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
