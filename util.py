import numpy
import time

# a simple PID controller with optional integral windup mitigation
# TODO bumpless parameter changing
class PID:
    def __init__(self, p, i, d, anti_windup=None):
        # initialize controller parameters
        self.p = p
        self.i = i
        self.d = d
        self.u = 0
        self.anti_windup = anti_windup

        # initialize controller state
        self.last_t = time.time()
        self.last_e = 0
        self.tot_e = 0

    def update(self, e):
        # get current time
        t = time.time()

        # update integral term
        # TODO simpson's rule
        self.tot_e += e * (t - self.last_t)

        # calculate controller output
        p_term = self.p * e
        i_term = self.i * self.tot_e
        d_term = self.d * (e - self.last_e) / (t - self.last_t)
        self.u = p_term + i_term + d_term

        # back-calculate integral term for anti-windup
        if self.anti_windup and self.u >= self.anti_windup[0]:
            self.u = self.anti_windup[0]
            if self.i != 0:
                self.tot_e = (self.u - p_term - d_term) / self.i
        elif self.anti_windup and self.u <= self.anti_windup[1]:
            self.u = self.anti_windup[1]
            if self.i != 0:
                self.tot_e = (self.u - p_term - d_term) / self.i

        # remember time and error
        self.last_t = t
        self.last_e = e
        return self.u

    def set_params(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d

# estimate current aircraft state from discrete updates
# TODO linear propagation?
class Estimator:
    def __init__(self):
        self.last_x = (0, 0, 0)
        self.x = (0, 0, 0)
        self.v = (0, 0, 0)
        self.last_r = numpy.zeros(3, 3)
        self.r = numpy.zeros(3, 3)
        self.w = numpy.zeros(3, 3)

    def update_state(self, state):
        # TODO infer velocity vector
        pass

    def update_indicators(self, indicators):
        # TODO calculate rotation matrix from indicators
        pass

    def update_map(self, map_obj):
        pass