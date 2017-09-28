#!/usr/bin/env python

import rospy
import math
from   std_msgs.msg      import Bool
from   dbw_mkz_msgs.msg  import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from   geometry_msgs.msg import TwistStamped
from   twist_controller  import Controller


class DBWNode(object):

    def __init__(self):
        rospy.init_node('dbw_node')

        self.steer_pub    = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub    = rospy.Publisher('/vehicle/brake_cmd',    BrakeCmd,    queue_size=1)

        self.controller  = Controller()

        self.dbw_enabled = False
        self.current_velocity = None
        self.twist_cmd = None

        rospy.Subscriber('/current_velocity',    TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/twist_cmd',           TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool,         self.dbw_enabled_cb)

        rate = rospy.Rate(50) 
        while not rospy.is_shutdown():
            throttle, brake, steering = self.controller.control(self.twist_cmd,
                                                                self.current_velocity,
                                                                self.dbw_enabled)
            if self.dbw_enabled:
                self.publish(throttle, brake, steering)
            rate.sleep()


    def publish(self, throttle, brake, steer):
        t_cmd                = ThrottleCmd()
        t_cmd.enable         = True
        t_cmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        t_cmd.pedal_cmd      = throttle
        self.throttle_pub.publish(t_cmd)

        b_cmd                = BrakeCmd()
        b_cmd.enable         = True
        b_cmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        b_cmd.pedal_cmd      = brake
        self.brake_pub.publish(b_cmd)

        s_cmd                = SteeringCmd()
        s_cmd.enable         = True
        s_cmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(s_cmd)


    def current_velocity_cb(self, msg):
        self.current_velocity = msg


    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg


    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data


if __name__ == '__main__':
    DBWNode()

