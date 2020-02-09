#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

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
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # rospy.logdebug('MM - WaypointUpdater.__init__(...)')

        # TODO: Add a subscriber for /obstacle_waypoint below
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.pose = None
        self.base_lane = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.loop()

    def loop(self):
        # rospy.logdebug('MM - WaypointUpdater.loop()')

        rate = rospy.Rate(50) # TODO: Try with ~30Hz-35Hz
        while not rospy.is_shutdown():
            if self.pose and self.base_lane and self.waypoint_tree:
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoint(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        # rospy.logdebug('MM - WaypointUpdater.get_closest_waypoint_idx()')

        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_idx)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoint(self, closest_idx):
        # rospy.logdebug('MM - WaypointUpdater.publish_waypoint(...)')

        lane = self.generate_lane()
        self.final_waypoints_pub.publish(lane)

    def generate_lane(self):
        # TODO: Make sure it works in both tracks.
        # When the ego-car is arriving to the end of the track, it stops.
        # Use Modulo maybe?

        # rospy.logdebug('MM - WaypointUpdater.generate_lane()')

        lane = Lane()
        lane.header = self.base_lane.header

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_lane_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_lane_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_lane_waypoints, closest_idx)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        # rospy.logdebug('MM - WaypointUpdater.decelerate_waypoints(...)')

        decelerated_waypoints = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) # Two waypoints back from line so front of car stops at line
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.0:
                vel = 0.0

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            decelerated_waypoints.append(p)

        return decelerated_waypoints

    def pose_cb(self, msg):
        # rospy.logdebug('MM - WaypointUpdater.pose_cb(...)')

        self.pose = msg

    def waypoints_cb(self, waypoints):
        # rospy.logdebug('MM - WaypointUpdater.waypoints_cb(...)')

        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # rospy.logdebug('MM - WaypointUpdater.traffic_cb(...)')

        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later

        # rospy.logdebug('MM - WaypointUpdater.obstacle_cb(pass)')

        pass

    def get_waypoint_velocity(self, waypoint):
        # rospy.logdebug('MM - WaypointUpdater.get_waypoint_velocity(...)')

        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        # rospy.logdebug('MM - WaypointUpdater.set_waypoint_velocity(...)')

        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        # rospy.logdebug('MM - WaypointUpdater.distance(...)')

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