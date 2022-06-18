
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
        error = should_value - is_value
        #P
        p_error = self.p * error
        print("P", p_error)

        #I
        self.int += error * self.dt
        i_error = self.i * self.int
        print("I", i_error)

        #D
        d_error = self.d * (error - self.err_old) / self.dt
        print("D", d_error)

        #PID
        pid = p_error + i_error + d_error
        print("Range", is_value)
        print("Pid", pid)

        if self.max < pid:
            pid = self.max
        elif pid < self.min:
            pid = self.min

        self.err_old = error
        print("Return ", pid)
        return pid


