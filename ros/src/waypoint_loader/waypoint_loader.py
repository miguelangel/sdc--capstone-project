#!/usr/bin/env python

import csv
import math
import os

import tf
import rospy

from geometry_msgs.msg import Quaternion
from styx_msgs.msg import Lane, Waypoint

CSV_HEADER = ['x', 'y', 'z', 'yaw']
MAX_DECEL = 1.0

class WaypointLoader(object):
    def __init__(self):
        rospy.init_node('waypoint_loader', log_level=rospy.DEBUG)

        rospy.logdebug('MM - WaypointLoader.__init__(...)')

        self.pub = rospy.Publisher('/base_waypoints', Lane, queue_size=1, latch=True)

        self.velocity = self.kmph2mps(rospy.get_param('~velocity'))
        self.new_waypoint_loader(rospy.get_param('~path'))
        rospy.spin()

    def new_waypoint_loader(self, path):
        rospy.logdebug('MM - WaypointLoader.new_waypoint_loader(...)')

        if os.path.isfile(path):
            waypoints = self.load_waypoints(path)
            self.publish(waypoints)
            rospy.loginfo('Waypoint Loded')
        else:
            rospy.logerr('%s is not a file', path)

    def quaternion_from_yaw(self, yaw):
        # TODO: Delete me
        rospy.logdebug('MM - WaypointLoader.quaternion_from_yaw(...)')

        return tf.transformations.quaternion_from_euler(0., 0., yaw)

    def kmph2mps(self, velocity_kmph):
        rospy.logdebug('MM - WaypointLoader.kmph2mps(...)')

        return (velocity_kmph * 1000.) / (60. * 60.)

    def load_waypoints(self, fname):
        rospy.logdebug('MM - WaypointLoader.load_waypoints(...)')

        waypoints = []
        with open(fname) as wfile:
            reader = csv.DictReader(wfile, CSV_HEADER)
            for wp in reader:
                p = Waypoint()
                p.pose.pose.position.x = float(wp['x'])
                p.pose.pose.position.y = float(wp['y'])
                p.pose.pose.position.z = float(wp['z'])
                q = self.quaternion_from_yaw(float(wp['yaw']))
                p.pose.pose.orientation = Quaternion(*q)
                p.twist.twist.linear.x = float(self.velocity)

                waypoints.append(p)

        return self.decelerate(waypoints)

    def distance(self, p1, p2):
        rospy.logdebug('MM - WaypointLoader.distance(...)')

        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def decelerate(self, waypoints):
        rospy.logdebug('MM - WaypointLoader.decelerate(...)')

        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:-1][::-1]:
            dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)

        return waypoints

    def publish(self, waypoints):
        rospy.logdebug('MM - WaypointLoader.publish(...)')

        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')
