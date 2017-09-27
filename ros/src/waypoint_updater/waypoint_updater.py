#!/usr/bin/env python

import rospy
import math
import tf
from   geometry_msgs.msg import PoseStamped, TwistStamped
from   styx_msgs.msg     import Lane, Waypoint
from   std_msgs.msg      import Int32, Float32
from   copy              import deepcopy


LOOKAHEAD_WPS = 200
MAX_DECEL     = 1.0
STOP_BUFFER   = 4.0


class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose',      PoseStamped, self.current_pose_cb)
        rospy.Subscriber('/base_waypoints',    Lane,        self.base_waypoints_cb)
        rospy.Subscriber('/traffic_waypoint',  Int32,       self.traffic_waypoint_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32,       self.obstacle_waypoint_cb)
        rospy.Subscriber('/current_velocity',  TwistStamped,self.current_velocity_cb)

        self.current_velocity = 0.0
        self.traffic_waypoint = -1
        self.braking = False

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

            pose = self.current_pose
            wpts = self.base_waypoints.waypoints

            next_wp = self.get_next_waypoint(pose, wpts)
            traffic_wp = self.traffic_waypoint


            min_stopping_dist = self.current_velocity**2 / (2 * MAX_DECEL)
            max_stopping_dist = (2 * self.current_velocity**2) / (2 * MAX_DECEL)

            if traffic_wp != -1:
                self.braking = True
                lane.waypoints = self.get_final_waypoints(wpts, next_wp, traffic_wp)
            else:
                self.braking = False
                lane.waypoints = self.get_final_waypoints(wpts, next_wp, next_wp+LOOKAHEAD_WPS)

            self.final_waypoints_pub.publish(lane)


    def get_final_waypoints(self, waypoints, start_wp, end_wp):
        final_waypoints = []
        for i in range(start_wp, end_wp):
            index = i % len(waypoints)
            wp = Waypoint()
            wp.pose.pose.position.x  = waypoints[index].pose.pose.position.x
            wp.pose.pose.position.y  = waypoints[index].pose.pose.position.y
            wp.pose.pose.position.z  = waypoints[index].pose.pose.position.z
            wp.pose.pose.orientation = waypoints[index].pose.pose.orientation
            if self.braking:
                wp.twist.twist.linear.x  = min(self.current_velocity,
                                               waypoints[index].twist.twist.linear.x)
            else:
                wp.twist.twist.linear.x = waypoints[index].twist.twist.linear.x

            final_waypoints.append(wp)

        if self.braking:
            # Find the traffic_wp index in final_waypoints to pass to decelerate
            tl_wp = len(final_waypoints)
            # If we are braking set all waypoints passed traffic_wp within LOOKAHEAD_WPS to 0.0
            for i in range(end_wp, start_wp + LOOKAHEAD_WPS):
                index = i % len(waypoints)
                wp = Waypoint()
                wp.pose.pose.position.x  = waypoints[index].pose.pose.position.x
                wp.pose.pose.position.y  = waypoints[index].pose.pose.position.y
                wp.pose.pose.position.z  = waypoints[index].pose.pose.position.z
                wp.pose.pose.orientation = waypoints[index].pose.pose.orientation
                wp.twist.twist.linear.x  = 0.0
                final_waypoints.append(wp)
            final_waypoints = self.decelerate(final_waypoints, tl_wp)

        return final_waypoints


    def decelerate(self, waypoints, tl_wp):
        last = waypoints[tl_wp]
        last.twist.twist.linear.x = 0.0
        for wp in waypoints[:tl_wp][::-1]:
            dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
            dist = max(0.0, dist-STOP_BUFFER)
            vel  = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.0:
                vel = 0.0
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints


    def distance(self, p1, p2):
        x = p1.x - p2.x
        y = p1.y - p2.y
        z = p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)


    def current_pose_cb(self, msg):
        self.current_pose = msg


    def base_waypoints_cb(self, msg):
        self.base_waypoints = msg


    def traffic_waypoint_cb(self, msg):
        self.traffic_waypoint = msg.data


    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x


    def obstacle_waypoint_cb(self, msg):
        self.obstacle_waypoint = msg.data


    def get_closest_waypoint(self, pose, waypoints):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_dist = float('inf')
        closest_wp = 0
        for i in range(len(waypoints)):
            dist = math.sqrt((pose.pose.position.x-waypoints[i].pose.pose.position.x)**2 \
                            +(pose.pose.position.y-waypoints[i].pose.pose.position.y)**2 \
                            +(pose.pose.position.z-waypoints[i].pose.pose.position.z)**2)
            if dist < closest_dist:
                closest_dist = dist
                closest_wp = i

        return closest_wp


    def get_next_waypoint(self, pose, waypoints):
        """Identifies the closest path waypoint that's ahead of the given position

        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the next waypoint in self.waypoints

        """
        closest_wp = self.get_closest_waypoint(pose, waypoints)
        wp_x = waypoints[closest_wp].pose.pose.position.x
        wp_y = waypoints[closest_wp].pose.pose.position.y
        heading = math.atan2( (wp_y-pose.pose.position.y), (wp_x-pose.pose.position.x) )
        x = pose.pose.orientation.x
        y = pose.pose.orientation.y
        z = pose.pose.orientation.z
        w = pose.pose.orientation.w
        euler_angles_xyz = tf.transformations.euler_from_quaternion([x,y,z,w])
        theta = euler_angles_xyz[-1]
        angle = math.fabs(theta-heading)
        if angle > math.pi/4:
            closest_wp += 1

        return closest_wp


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
