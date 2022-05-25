
class PID_Controller():

    def __init__(self, p , i ,d , max, dt):
        self.p = p
        self.i = i
        self.d = d
        self.max = max
        self.dt = dt

        # init 0.0
        self.err_old = 0.0
        self.int = 0.0

    def calc_pid(self, is_value, should_value, max_range):

        error = should_value - min(is_value, max_range)

        p_error = self.p * error
        print("P", p_error)

        self.int += error * self.dt
        i_error = self.i * self.int
        print("I", i_error)

        d_error = self.d * (error - self.err_old) / self.dt
        print("D", d_error)

        out = p_error + i_error + d_error
        print("Range", is_value)
        print("Pid", out)

        if self.max < out:
            out = self.max
        elif out < -self.max:
            out = -self.max

        self.err_old = error
        print("Return ", out)
        return out


