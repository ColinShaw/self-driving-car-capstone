#!/usr/bin/env python

import rospy
import math
import tf
from   geometry_msgs.msg import PoseStamped, TwistStamped
from   styx_msgs.msg     import Lane, Waypoint
from   std_msgs.msg      import Int32, Float32


LOOKAHEAD_WPS = 200


class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose',      PoseStamped, self.current_pose_cb)
        rospy.Subscriber('/base_waypoints',    Lane,        self.base_waypoints_cb)
        rospy.Subscriber('/traffic_waypoint',  Int32,       self.traffic_waypoint_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32,       self.obstacle_waypoint_cb)
        rospy.Subscriber('/current_velocity',  TwistStamped,self.current_velocity_cb)
        # For testing and manual topic control
        rospy.Subscriber('/set_speed', Float32, self.set_speed_cb)

        # Set speed to default 4.47m/s (~10mph)
        self.speed = 4.47
        self.current_velocity = 0.0
        self.decel_rate = -5.0
        self.traffic_waypoint = -1

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

            closest_wp = self.get_next_waypoint(pose, wpts)

            # Create forward list of waypoints
            for i in range(closest_wp, closest_wp + LOOKAHEAD_WPS):
                index = i % len(wpts)
                lane.waypoints.append(wpts[index])

            if hasattr(self, 'set_speed'):
                self.speed = self.set_speed

            traffic_wp = self.traffic_waypoint
            stopping_dist = self.current_velocity**2 / (2*abs(self.decel_rate))
            if traffic_wp != -1:
                if stopping_dist < self.distance(wpts, closest_wp, traffic_wp):
                    initial_velocity = self.current_velocity
                    j = 0
                    for i in range(closest_wp, closest_wp + LOOKAHEAD_WPS):
                        dist = self.distance(wpts, i, traffic_wp)
                        if dist > stopping_dist:
                            self.set_waypoint_velocity(lane.waypoints, j, self.speed)
                            j += 1
                        else:
                            dist_traveled = stopping_dist - dist
                            vel = math.sqrt(abs(initial_velocity**2+2*self.decel_rate*dist_traveled))
                            self.set_waypoint_velocity(lane.waypoints, j, vel)
                            j += 1
                else:
                    for i in range(len(lane.waypoints)):
                        self.set_waypoint_velocity(lane.waypoints, i, self.speed)
            else:
                for i in range(len(lane.waypoints)):
                    self.set_waypoint_velocity(lane.waypoints, i, self.speed)

            self.final_waypoints_pub.publish(lane)


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


    def set_speed_cb(self, msg):
        self.set_speed = msg.data


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
