import math

class PID_Controller():

    def __init__(self, p , i ,d , max, min, dt):
        self.p = p
        self.i = i
        self.d = d
        self.max = max
        self.min = min
        self.dt = dt

        # init 0.0
        self.err_old = 0.0
        self.int = 0.0

    def calc_pid(self, is_value, should_value, max_range, min_range):

        is_value = min(is_value, max_range)
        is_value = max(is_value , min_range)
        error = should_value - min(is_value, max_range)
        
        #print("is: ", is_value)
        #print("should: ", should_value)
        #print("error: ", error)

        #P
        p_error = self.p * error
        #print("P", p_error)

        #I
        self.int += error * self.dt
        i_error = self.i * self.int
        #print("I", i_error)

        #D
        d_error = self.d * (error - self.err_old) / self.dt
        #print("D", d_error)

        #PID
        pid = p_error + i_error + d_error
        #print("Pid", pid)

        #negative = 45 / (1 + math.e ** (-0.25 * (pid + 30))) - 45
        #positive = 45 / (1 + math.e ** (-0.25 * (pid - 30)))

        #steering_angle = positive + negative

        self.err_old = error
        #print("Return ", pid)
        return pid

