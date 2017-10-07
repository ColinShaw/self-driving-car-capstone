class LowPassFilter(object):

    def __init__(self, tau, ts):
        self.a        = 1.0 / (tau / ts + 1.0)
        self.b        = tau / ts / (tau / ts + 1.0);
        self.last_val = 0.0


    def filter(self, val):
        val = self.a * val + self.b * self.last_val
        self.last_val = val
        return val

