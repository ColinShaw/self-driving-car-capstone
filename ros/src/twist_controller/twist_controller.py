from pid import PID 


GAS_DENSITY = 2.858
ONE_MPH     = 0.44704


class Controller(object):

    def __init__(self):
        self.steering_pid = PID(1.0, 0.0, 0.0)
        self.throttle_pid = PID(1.0, 0.0, 0.0)
        self.brake_pid    = PID(1.0, 0.0, 0.0)


    def control(self, **kwargs):
        throttle_error = 0.0
        brake_error    = 0.0
        steering_error = 0.0
        time           = 0.1

        throttle = self.throttle_pid.update(throttle_error, time)
        brake    = self.brake_pid.update(brake_error, time)
        steering = self.steering_pid.update(steering_error, time)

        return 50.0, 0.0, 1.0
        #return throttle, brake, steering

