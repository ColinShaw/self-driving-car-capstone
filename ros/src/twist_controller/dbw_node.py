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

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.controller  = Controller(vehicle_mass, fuel_capacity, brake_deadband,
                                      decel_limit, accel_limit, wheel_radius, wheel_base,
                                      steer_ratio, max_lat_accel, max_steer_angle)

        self.dbw_enabled = False
        self.current_velocity = None
        self.twist_cmd = None

        rospy.Subscriber('/current_velocity',    TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/twist_cmd',           TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool,         self.dbw_enabled_cb)

        self.loop()


    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
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
