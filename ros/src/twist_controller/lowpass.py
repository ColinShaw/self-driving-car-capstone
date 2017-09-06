class LowPassFilter(object):


    def __init__(self, tau, ts):
        self.a = 1.0 / (tau / ts + 1.0)
        self.b = tau / ts / (tau / ts + 1.0);

        self.last_val = 0.0
        self.ready    = False


    def filter(self, val):
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val
