import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64, Bool, Int64MultiArray
import time
import math


class Overtaker(Node):
    def __init__(self):
        super().__init__('overtaker')
        self.blocked = False
        self.left_lane_free = True
        self.right_lane_free = True
        self.current_phase = 0

        # small front tof
        self.sub_front2 = self.create_subscription(Range, '/tof_Front2', self.is_road_blocked, 1)

        # other ToFs
        self.sub_left = self.create_subscription(Range, 'tof_Front_Left', self.look_left, 1)
        self.sub_right = self.create_subscription(Range, 'tof_Front_Right', self.look_right, 1)

        # controls for lane based steer
        self.pub_block_lbs = self.create_publisher(Bool, 'block_lane_based_steer', 1)
        self.pub_adjust_RoI = self.create_publisher(Int64MultiArray, 'adjust_region_of_interest', 1)
        self.pub_adjust_factor = self.create_publisher(Float64, 'adjust_driveway_factor', 1)

        # speed
        self.sub_speed = self.create_subscription(Float64, '/speed', self.get_current_speed, 1)
        self.speed = 0
        self.lc_time = 0

        # steering
        self.pub_steering = self.create_publisher(Float64, '/steering', 1)

        # parking
        self.pub_block_parking = self.create_publisher(Bool, 'block_parking',
                                                       1)

        # for praking mode
        self.sub_block_overtaker = self.create_subscription(Bool, 'block_lane_based_steer', self.update_self_blocked, 1)

    def update_self_blocked(self, msg_in):
        self.blocked = msg_in.data

    def is_road_blocked(self, msg):
        range = min(msg.max_range, msg.range)
        if range < 1 and self.left_lane_free and self.current_phase == 0 and not self.blocked:

            # shut up lane detection
            out = Bool()
            out.data = True
            self.pub_block_lbs.publish(out)
            self.pub_block_parking.publish(out)

            print("---INITITATE OVERTAKE---")
            self.current_phase = 1
            print("---SET CURRENT PHASE TO 1---")
        else:
            self.overtake()
            print("nop")

    def overtake(self):

        # Phase 1 - Switch Lanes
        if self.current_phase == 1:
            self.change_lane(-25.0, 0.9)
            self.current_phase = 2
            print("---SET CURRENT PHASE TO 2---")
        # Phase 2 - Switch RoI and Hold Lane
        elif self.current_phase == 2:
            new_roi_msg = Int64MultiArray()
            new_roi_msg.data = [260, 500, 100, 40]
            new_factor_msg = Float64()
            new_factor_msg.data = 0.85
            self.pub_adjust_RoI.publish(new_roi_msg)
            self.pub_adjust_factor.publish(new_factor_msg)
            msg_ld = Bool()
            msg_ld.data = False
            self.pub_block_lbs.publish(msg_ld)
            if self.right_lane_free == False:
                self.current_phase = 3
                print("---SET CURRENT PHASE TO 3---")
        # Phase 3 - Look Right to check for box
        elif self.current_phase == 3:
            if self.right_lane_free == True:
                self.current_phase = 4
                print("---SET CURRENT PHASE TO 4---")
        # Phase 4 - Hold Lane when box is gone
        elif self.current_phase == 4:
            msg_ld = Bool()
            msg_ld.data = True
            self.pub_block_lbs.publish(msg_ld)
            new_roi_msg = Int64MultiArray()
            new_roi_msg.data = [140, 380, 100, 40]
            new_factor_msg = Float64()
            new_factor_msg.data = 1.25
            self.pub_adjust_RoI.publish(new_roi_msg)
            self.pub_adjust_factor.publish(new_factor_msg)
            self.current_phase = 5
            print("---SET CURRENT PHASE TO 5---")
        # Phase 5 - Switch Lanes back
        elif self.current_phase == 5:
            self.change_lane(25.0, 0.9)
            msg_ld = Bool()
            msg_ld.data = False
            self.pub_block_lbs.publish(msg_ld)
            self.pub_block_parking.publish(msg_ld)
            print("---OVERTAKE COMPLETE---")
            self.current_phase = 0 # reset for next lap
        else:
            return

    def change_lane(self, starting_angle, correction_factor):
            out = Float64()
            out.data = starting_angle
            self.pub_steering.publish(out)
            self.lc_time = self.calc_lc_time()
            time.sleep(self.lc_time * correction_factor)
            out.data = starting_angle * -1
            self.pub_steering.publish(out)
            time.sleep(self.lc_time * correction_factor)
            out.data = 0.0
            self.pub_steering.publish(out)

    def calc_lc_time(self):
        if self.speed != 0:
            gamma = 1.57079  # 90 degrees in rad
            beta = 0.43633  # 25 degrees in rad
            b = 0.25
            c = (b * math.sin(gamma)) / math.sin(beta)  # calculation of effective distance via sine rule
            time = c / self.speed  # calculation of needed time via distance / speed
            return time
        else:
            return 0

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
