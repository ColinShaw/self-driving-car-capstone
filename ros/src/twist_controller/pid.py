MIN_NUM = float('-inf')
MAX_NUM = float('inf')
# import rospy

class PID(object):

    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp  = kp
        self.ki  = ki
        self.kd  = kd

        self.min = mn
        self.max = mx

        self.reset()
        self.last_error = 0.0


    def reset(self):
        self.int_val      = 0.0
        self.last_int_val = 0.0


    def update(self, error, sample_time):
        self.last_int_val = self.int_val

        integral          = self.int_val + error * sample_time;
        derivative        = (error - self.last_error) / sample_time;

        y                 = self.kp * error + self.ki * self.int_val + self.kd * derivative;
        val               = max(self.min, min(y, self.max))
        # rospy.logwarn('++++++++++++++++++++++++++++++++++++++++++=')
        # rospy.logwarn('proport : '+str(self.kp * error))
        # rospy.logwarn('differen: '+str(self.kd * derivative))
        # rospy.logwarn('integral: '+str(self.ki * self.int_val))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val
