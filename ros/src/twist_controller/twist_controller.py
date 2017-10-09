import rospy
import math
from   yaw_controller import YawController
from   pid            import PID
from   lowpass        import LowPassFilter
from   std_msgs.msg   import Float32


GAS_DENSITY         =  2.858
THROTTLE_MAX        =  0.75
THROTTLE_CONST      =  0.6
MAX_THROTTLE_CHANGE =  0.005
BRAKE_MAX           =  0.6
BRAKE_CONST         =  0.3
BRAKE_SKIP          = -1.1
BRAKE_TAU           =  0.5
MIN_BRAKING         =  100.0
RESET_SPEED         =  0.5


class Controller(object):

    def __init__(self):
        self.vehicle_mass    = rospy.get_param('~vehicle_mass')
        self.fuel_capacity   = rospy.get_param('~fuel_capacity')
        #self.brake_deadband  = rospy.get_param('~brake_deadband')
        self.decel_limit     = rospy.get_param('~decel_limit')
        self.accel_limit     = rospy.get_param('~accel_limit')
        self.wheel_radius    = rospy.get_param('~wheel_radius')
        self.wheel_base      = rospy.get_param('~wheel_base')
        self.steer_ratio     = rospy.get_param('~steer_ratio')
        self.max_lat_accel   = rospy.get_param('~max_lat_accel')
        self.max_steer_angle = rospy.get_param('~max_steer_angle')

        self.total_vehicle_mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY
        self.max_brake_torque   = BRAKE_MAX * self.total_vehicle_mass * abs(self.decel_limit) * self.wheel_radius

        self.last_time     = None
        self.last_throttle = 0.0
        self.pid_control   = PID(5.0, 0.05, 0.0)
        self.brake_lpf     = LowPassFilter(BRAKE_TAU, 0.02)
        self.yaw_control   = YawController(wheel_base      = self.wheel_base,
                                           steer_ratio     = self.steer_ratio,
                                           min_speed       = 0.0,
                                           max_lat_accel   = self.max_lat_accel,
                                           max_steer_angle = self.max_steer_angle)


    def control(self, twist_cmd, current_velocity, dbw_enabled):

        throttle = 0.0
        brake    = 0.0
        steering = 0.0

        if not all((twist_cmd, current_velocity)):
            return throttle, brake, steering

        if self.last_time is None:
            self.last_time = rospy.get_time()
            return throttle, brake, steering

        time           = rospy.get_time()
        delta_t        = time - self.last_time
        self.last_time = time

        desired_linear_velocity  = twist_cmd.twist.linear.x
        desired_angular_velocity = twist_cmd.twist.angular.z

        current_linear_velocity  = current_velocity.twist.linear.x
        current_angular_velocity = current_velocity.twist.angular.z

        if abs(desired_linear_velocity) < RESET_SPEED:
            self.pid_control.reset()

        if dbw_enabled:
            velocity_error = desired_linear_velocity - current_linear_velocity
            control        = self.pid_control.update(velocity_error, delta_t)

            if control >= 0.0:
                throttle = self.soft_scale(control, THROTTLE_MAX, THROTTLE_CONST)
                if throttle - self.last_throttle > MAX_THROTTLE_CHANGE:
                    throttle = self.last_throttle + MAX_THROTTLE_CHANGE
                self.last_throttle = throttle
                brake              = 0.0
            elif control >= BRAKE_SKIP:
                throttle           = 0.0
                self.last_throttle = 0.0
                brake              = 0.0
            else:
                throttle           = 0.0
                self.last_throttle = 0.0
                brake              = self.soft_scale(-control, self.max_brake_torque, BRAKE_CONST) 

            brake = self.brake_lpf.filter(brake)
            if brake < MIN_BRAKING:
                brake = 0.0

            #rospy.logwarn('Error:    {: 04.2f}'.format(velocity_error))
            #rospy.logwarn('Control:  {: 04.2f}'.format(control))
            #rospy.logwarn('Throttle: {: 04.2f}'.format(throttle))
            #rospy.logwarn('Brake:    {: 06.2f}'.format(brake))
            #rospy.logwarn('Speed:    {: 04.2f}'.format(current_linear_velocity))
            #rospy.logwarn('')

            steering = self.yaw_control.get_steering(desired_linear_velocity,
                                                     desired_angular_velocity,
                                                     current_linear_velocity)

            return throttle, brake, steering

        else:
            self.pid_control.reset()
            return throttle, brake, steering


    def soft_scale(self, value, scale, stretch):
        if value <= 0.0:
            return 0.0
        else:
            return scale * math.tanh(value * stretch)

