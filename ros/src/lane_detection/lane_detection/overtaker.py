import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from . import pid
import time


class Overtaker(Node):
    def __init__(self):
        super().__init__('overtaker')

        # small front tof
        self.subFront2 = self.create_subscription(Range, '/tof_Front2', self.is_road_blocked, 1)

        self.pub_update_LD = self.create_publisher(Bool, 'stop_lane_based_steer', 1)

        self.subLeft = self.create_subscription(Range, 'tof_Front_Left', self.look_left, 1)
        self.subRight = self.create_subscription(Range, 'tof_Front_Right', self.update_right_dist, 1)

        self.right_dist = 0.0
        self.pid_controller = pid.PID_Controller(100, 0.001, 1, 45, -45, (1 / 5))
        # steering
        self.pub_steering = self.create_publisher(Float64, '/steering', 1)

        self.overtaker_mode = False
        self.left_lane_free = True

        #self.start_time = 0.0

        #self.delay = 6.5

    def is_road_blocked(self, msg):
        range = min(msg.max_range, msg.range)
        if range < 0.75 and self.left_lane_free:
            # shut up lane detection
            out = Bool()
            out.data = True
            self.pub_update_LD.publish(out)

            #self.start_time = time.time()
            self.overtaker_mode = True
        else:
            print("nop")


    def update_right_dist(self, msg):
        print("overtaker mode:", self.overtaker_mode)
        if not self.overtaker_mode:
            return
        elif self.overtaker_mode and msg.max_range <= msg.range:
            steer = Float64()
            steer.data = -15.0
            print(steer)
            self.pub_steering.publish(steer)
        else:
            # should_value = 0.12
            # out = self.pid_controller.calc_pid(msg.range, should_value, msg.max_range, msg.min_range)
            # msg_out = Float64()
            # msg_out.data = float(-out)
            # self.pub_steering.publish(msg_out)
            # print(self.start_time, " ", self.delay, "   ", time.time())

        if self.start_time + self.delay < time.time():
            self.overtaker_mode = False
            lane_out = Bool()
            lane_out.data = False
            self.pub_update_LD.publish(lane_out)
        print(msg.range)

    def look_left(self, msg):
        self.left_lane_free = 0.5 < msg.range
        print("left lane:", self.left_lane_free)


def main(args=None):
    rclpy.init(args=args)

    overtaker = Overtaker()
    rclpy.spin(overtaker)
    overtaker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
