import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64, Bool, Int64MultiArray
from . import pid
import time
import math


class Overtaker(Node):
    def __init__(self):
        super().__init__('overtaker')

        # small front tof
        self.subFront2 = self.create_subscription(Range, '/tof_Front2', self.is_road_blocked, 1)

        self.pub_update_LD = self.create_publisher(Bool, 'stop_lane_based_steer', 1)
        self.pub_adjust_RoI = self.create_publisher(Int64MultiArray, 'adjust_region_of_interest', 1)
        self.pub_adjust_factor = self.create_publisher(Float64, 'adjust_driveway_factor', 1)

        self.subLeft = self.create_subscription(Range, 'tof_Front_Left', self.look_left, 1)
        self.subRight = self.create_subscription(Range, 'tof_Front_Right', self.look_right, 1)

        self.subSpeed = self.create_subscription(Float64, '/speed', self.get_current_speed, 1)
        self.speed = 0
        self.lc_time = 0

        self.right_dist = 0.0
        self.pid_controller = pid.PID_Controller(100, 0.001, 1, 45, -45, (1 / 5))

        # steering
        self.pub_steering = self.create_publisher(Float64, '/steering', 1)

        self.overtaker_mode = False
        self.left_lane_free = True
        self.right_lane_free = True

        self.current_phase = 0

    def is_road_blocked(self, msg):
        range = min(msg.max_range, msg.range)
        if range < 2 and self.left_lane_free and self.current_phase == 0:

            # shut up lane detection
            out = Bool()
            out.data = True
            self.pub_update_LD.publish(out)

            print("---INITITATE OVERTAKE---")
            self.current_phase = 1
            print("---SET CURRENT PHASE TO 1---")
        else:
            self.overtake()
            print("nop")

    def overtake(self):
        # Phase 1 - Switch Lanes
        match self.current_phase:
            case 1:
                out = Float64()
                out.data = -25.0
                self.pub_steering.publish(out)
                self.lc_time = self.calc_lc_time()
                time.sleep(self.lc_time * 0.9)
                out.data = 25.0
                self.pub_steering.publish(out)
                time.sleep(self.lc_time * 0.9)
                out.data = 0.0
                self.pub_steering.publish(out)
                self.current_phase = 2
                print("---SET CURRENT PHASE TO 2---")
            # Phase 2 - Switch RoI and Hold Lane
            case 2:
                new_roi_msg = Int64MultiArray()
                new_roi_msg.data = [260, 500, 100, 40]
                new_factor_msg = Float64()
                new_factor_msg.data = 0.85
                self.pub_adjust_RoI.publish(new_roi_msg)
                self.pub_adjust_factor.publish(new_factor_msg)
                msg_ld = Bool()
                msg_ld.data = False
                self.pub_update_LD.publish(msg_ld)
                if self.right_lane_free == False:
                    self.current_phase = 3
                    print("---SET CURRENT PHASE TO 3---")
            # Phase 3 - Look Right to check for box
            case 3:
                if self.right_lane_free == True:
                    self.current_phase = 4
                    print("---SET CURRENT PHASE TO 4---")
            # Phase 4 - Hold Lane when box is gone
            case 4:
                msg_ld = Bool()
                msg_ld.data = True
                self.pub_update_LD.publish(msg_ld)
                new_roi_msg = Int64MultiArray()
                new_roi_msg.data = [140, 380, 100, 40]
                new_factor_msg = Float64()
                new_factor_msg.data = 1.25
                self.pub_adjust_RoI.publish(new_roi_msg)
                self.pub_adjust_factor.publish(new_factor_msg)
                self.current_phase = 5
                print("---SET CURRENT PHASE TO 5---")
            # Phase 5 - Switch Lanes back
            case 5:
                out = Float64()
                out.data = 25.0
                self.pub_steering.publish(out)
                self.lc_time = self.calc_lc_time()
                time.sleep(self.lc_time * 0.9)
                out.data = -25.0
                self.pub_steering.publish(out)
                time.sleep(self.lc_time * 0.9)
                out.data = 0.0
                self.pub_steering.publish(out)
                msg_ld = Bool()
                msg_ld.data = False
                self.pub_update_LD.publish(msg_ld)
                self.overtaker_mode = False
                print("---OVERTAKE COMPLETE---")
            case _:
                return

    def calc_lc_time(self):
        gamma = 1.57079  # 90 degrees in rad
        beta = 0.43633  # 25 degrees in rad
        b = 0.25
        c = (b * math.sin(gamma)) / math.sin(beta)  # calculation of effective distance via sine rule
        time = c / self.speed  # calculation of needed time via distance / speed
        return time

    def get_current_speed(self, msg):
        self.speed = msg.data

    def look_left(self, msg):
        self.left_lane_free = 0.5 < msg.range
        print("left lane:", self.left_lane_free)

    def look_right(self, msg):
        self.right_lane_free = 0.5 < msg.range
        print("right lane:", self.right_lane_free)


def main(args=None):
    rclpy.init(args=args)

    overtaker = Overtaker()
    rclpy.spin(overtaker)
    overtaker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
