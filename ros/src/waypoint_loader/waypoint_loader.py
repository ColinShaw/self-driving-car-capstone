#!/usr/bin/env python

import os
import csv
import math
import tf
import rospy
from   geometry_msgs.msg import Quaternion
from   styx_msgs.msg     import Lane, Waypoint


CSV_HEADER = ['x', 'y', 'z', 'yaw']
MAX_DECEL  = 1.0


class WaypointLoader(object):

    def __init__(self):
        rospy.init_node('waypoint_loader', log_level=rospy.DEBUG)
        self.pub      = rospy.Publisher('/base_waypoints', Lane, queue_size=1, latch=True)
        self.velocity = rospy.get_param('~velocity')
        self.new_waypoint_loader(rospy.get_param('~path'))
        self.publish()
        rospy.spin()


    def publish(self):
        lane                 = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp    = rospy.Time(0)
        lane.waypoints       = self.waypoints
        self.pub.publish(lane)


    def new_waypoint_loader(self, path):
        if os.path.isfile(path):
            self.waypoints = self.load_waypoints(path)
            rospy.loginfo('Waypoint Loaded')
        else:
            rospy.logerr('%s is not a file', path)


    def quaternion_from_yaw(self, yaw):
        return tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)


    def load_waypoints(self, fname):
        waypoints = []
        with open(fname) as wfile:
            reader = csv.DictReader(wfile, CSV_HEADER)
            for wp in reader:
                p                       = Waypoint()
                p.pose.pose.position.x  = float(wp['x'])
                p.pose.pose.position.y  = float(wp['y'])
                p.pose.pose.position.z  = float(wp['z'])

                q = self.quaternion_from_yaw(float(wp['yaw']))
                p.pose.pose.orientation = Quaternion(*q)
                p.twist.twist.linear.x  = float(self.velocity * 0.27778)
                waypoints.append(p)
        return self.decelerate(waypoints)


    def distance(self, p1, p2):
        x = p1.x - p2.x
        y = p1.y - p2.y
        z = p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)


    def decelerate(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.0
        for wp in waypoints[:-1][::-1]:
            dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
            vel  = math.sqrt(2 * MAX_DECEL * dist) * 3.6
            if vel < 1.0:
                vel = 0.0
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints



if __name__ == '__main__':
    try:
        WaypointLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')

