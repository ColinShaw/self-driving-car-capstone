#!/usr/bin/env python

import rospy
import math
from   geometry_msgs.msg import PoseStamped
from   styx_msgs.msg     import Lane, Waypoint
from   std_msgs.msg      import Int32


LOOKAHEAD_WPS = 200 


class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose',      PoseStamped, self.current_pose_cb)
        rospy.Subscriber('/base_waypoints',    Lane,        self.base_waypoints_cb)
        rospy.Subscriber('/traffic_waypoint',  Int32,       self.traffic_waypoint_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32,       self.obstacle_waypoint_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


    def loop(self):
        if hasattr(self, 'base_waypoints') and hasattr(self, 'current_pose'):
            lane                 = Lane()
            lane.header.stamp    = rospy.Time().now()
            lane.header.frame_id = '/world'

            p    = self.current_pose.pose.position
            wpts = self.base_waypoints.waypoints

            nearest_index    = None
            nearest_distance = float('inf')

            # Compute nearest waypoint
            for i in range(len(wpts)):
                q = wpts[i].pose.pose.position
                d = math.sqrt((p.x-q.x)**2 + (p.y-q.y)**2 + (p.z-q.z)**2)
                if d < nearest_distance:
                    nearest_index    = i
                    nearest_distance = d

            # Create forward list of waypoints 
            for i in range(nearest_index, nearest_index + LOOKAHEAD_WPS):
                index = i % len(wpts)
                lane.waypoints.append(wpts[index])

            # Set velocity for waypoints
            # Needs to bring car to stop and resume driving based on light
            for i in range(len(lane.waypoints)):
                lane.waypoints[i].twist.twist.linear.x = 10.0

            self.final_waypoints_pub.publish(lane)


    def current_pose_cb(self, msg):
        self.current_pose = msg


    def base_waypoints_cb(self, msg):
        self.base_waypoints = msg


    def traffic_waypoint_cb(self, msg):
        self.traffic_waypoint = msg.data


    def obstacle_waypoint_cb(self, msg):
        self.obstacle_waypoint = msg.data


    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x


    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity


    def distance(self, waypoints, wp1, wp2):
        dist = 0.0
        dl   = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1   = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
