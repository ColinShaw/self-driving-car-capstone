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

        vehicle_mass    = rospy.get_param('~vehicle_mass',    1736.35)
        fuel_capacity   = rospy.get_param('~fuel_capacity',   13.5)
        brake_deadband  = rospy.get_param('~brake_deadband',  0.1)
        decel_limit     = rospy.get_param('~decel_limit',    -5.0)
        accel_limit     = rospy.get_param('~accel_limit',     1.0)
        wheel_radius    = rospy.get_param('~wheel_radius',    0.2413)
        wheel_base      = rospy.get_param('~wheel_base',      2.8498)
        steer_ratio     = rospy.get_param('~steer_ratio',     14.8)
        max_lat_accel   = rospy.get_param('~max_lat_accel',   3.0)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.0)

        rospy.Subscriber('/twist_cmd',           TwistStamped, self.twist_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool,         self.dbw_enabled_cb)

        self.steer_pub    = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub    = rospy.Publisher('/vehicle/brake_cmd',    BrakeCmd,    queue_size=1)

        self.dbw_enabled = False
        self.controller  = Controller()

        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


    def loop(self):
        if hasattr(self, 'twist') and self.dbw_enabled:
            l = self.twist.twist.linear
            a = self.twist.twist.angular
            params = {
                'pos_x': l.x,
                'pos_y': l.y,
                'pos_z': l.z,
                'ang_x': a.x,
                'ang_y': a.y,
                'ang_z': a.z
            }
            throttle, brake, steer = self.controller.control()
            self.publish(throttle, brake, steer)


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


    def twist_cb(self, msg):
        self.twist = msg


    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data


if __name__ == '__main__':
    DBWNode()

