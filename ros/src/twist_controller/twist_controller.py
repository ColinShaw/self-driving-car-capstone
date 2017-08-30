import rospy
from   pid import PID 


GAS_DENSITY = 2.858
ONE_MPH     = 0.44704


class Controller(object):

    def __init__(self):
        self.steering_pid = PID(1.0, 0.0, 0.0)
        self.throttle_pid = PID(1.0, 0.0, 0.0)
        self.brake_pid    = PID(1.0, 0.0, 0.0)
        self.last_time    = None


    def control(self, **kwargs):
        twist_cmd        = kwargs['twist_cmd']
        current_velocity = kwargs['current_velocity']

        tc_l = twist_cmd.twist.linear
        tc_a = twist_cmd.twist.angular

        cv_l = current_velocity.twist.linear
        cv_a = current_velocity.twist.angular
            
        velocity_error = tc_l.x - cv_l.x
        heading_error  = tc_a.z - cv_a.z

        if self.last_time is not None:
            time           = rospy.get_time()
            delta_t        = time - self.last_time
            self.last_time = time
            throttle       = self.throttle_pid.update(velocity_error, delta_t)
            brake          = self.brake_pid.update(velocity_error, delta_t)
            steering       = self.steering_pid.update(heading_error, delta_t)
            return throttle, brake, steering
        else:
            self.last_time = rospy.get_time()
            return 0.0, 0.0, 0.0

