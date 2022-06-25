import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64, Bool
import time
import math

class Parker(Node):
    def __init__(self):
        super().__init__('parker')
        self.blocked = False
        self.scan_blocked = False
        self.reverse_speed = -0.2
        self.general_speed = 0.4
        self.back_distance = 10.0
        self.current_scan_phase = 0
        self.current_parking_phase = 0

        # ToF
        self.sub_right = self.create_subscription(Range, 'tof_Front_Right', self.look_for_parking, 1)
        #self.sub_left = self.create_subscription(Range, 'tof_Front_Left', self.look_left, 1)
        #self.sub_front_1 = self.create_subscription(Range, 'tof_Front', self.look_right, 1)
        self.sub_back = self.create_subscription(Range, 'tof_Back', self.get_back_distance, 1)

        # for overtaker mode
        self.sub_block_parking = self.create_subscription(Bool, 'block_parking', self.update_self_blocked, 1)

        # speed
        self.pub_speed = self.create_publisher(Float64, '/speed', 1)
        # steering
        self.pub_steering = self.create_publisher(Float64, '/steering', 1)

        self.pub_block_lbs = self.create_publisher(Bool, 'block_lane_based_steer', 1)
        self.pub_block_overtaker = self.create_publisher(Bool, 'block_overtaker', 1)

    def update_self_blocked(self, msg_in):
        self.blocked = msg_in.data

    def look_for_parking(self, msg_in):
        if self.scan_blocked == False:
            print("phase", self.current_scan_phase)
            print(msg_in.range < msg_in.max_range)
            if self.current_scan_phase == 0:
                # looking for box on the left
                if msg_in.range < msg_in.max_range and not self.blocked:
                    out = Bool()
                    out.data = True
                    self.pub_block_overtaker.publish(out)
                    self.current_scan_phase = 1

            elif self.current_scan_phase == 1:
                # have passed a box
                if msg_in.max_range <= msg_in.range:
                    self.current_scan_phase = 2
            elif self.current_scan_phase == 2:
                # found second box
                if msg_in.range < msg_in.max_range:
                    out = Bool()
                    out.data = True
                    self.pub_block_lbs.publish(out)
                    self.current_scan_phase = 3
            elif self.current_scan_phase == 3:
                #start parking
                msg_out = Float64()
                msg_out.data = 0.0
                self.pub_speed.publish(msg_out)
                print("jetzt wuerde ich einparken")
                self.scan_blocked = True
                print("---INITITATE PARKING---")
                self.current_parking_phase = 1
                print("---SET CURRENT PHASE TO 1---")
            else:
                return
        else:
            self.parking()

    def get_back_distance(self, msg_in):
        self.back_distance = min(msg_in.max_range, msg_in.range)

    def parking(self):
        # Phase 1 - Determine reverse speed, allignment
        if self.current_parking_phase == 1:
            # move forward
            msg_out = Float64()
            msg_out.data = self.general_speed
            self.pub_speed.publish(msg_out)
            time.sleep(1)
            # back-up and steer
            msg_out = Float64()
            msg_out.data = self.reverse_speed
            self.pub_speed.publish(msg_out)
            self.change_lane(25.0, 1, self.reverse_speed)
            print("---SET CURRENT PHASE TO 2---")
            self.current_parking_phase = 2
        # Phase 2 - closing in on parked car and stopping
        elif self.current_parking_phase == 2:
            if self.back_distance < 0.25:
                msg_out = Float64()
                msg_out.data = 0.0
                self.pub_speed.publish(msg_out)
                print(self.back_distance)
                print("---SET CURRENT PHASE TO 3---")
                self.current_parking_phase = 3
            else:
                print("Warte")
        # Phase 3 - forward lane switch and unblock
        elif self.current_parking_phase == 3:
            msg_out = Float64()
            msg_out.data = self.general_speed
            self.pub_speed.publish(msg_out)
            self.change_lane(-25.0, 1, self.general_speed)
            print("---PARKING COMPLETE---")
            self.current_parking_phase = 0
            out = Bool()
            out.data = False
            self.pub_block_overtaker.publish(out)
            self.pub_block_lbs.publish(out)

        else:
            return

    def change_lane(self, starting_angle, correction_factor, speed):
        out = Float64()
        out.data = starting_angle
        self.pub_steering.publish(out)
        self.lc_time = self.calc_lc_time(speed)
        time.sleep(self.lc_time * correction_factor)
        out.data = starting_angle * -1
        self.pub_steering.publish(out)
        time.sleep(self.lc_time * correction_factor)
        out.data = 0.0
        self.pub_steering.publish(out)

    def calc_lc_time(self, speed):
        if abs(speed) != 0:
            gamma = 1.57079  # 90 degrees in rad
            beta = 0.43633  # 25 degrees in rad
            b = 0.25
            c = (b * math.sin(gamma)) / math.sin(beta)  # calculation of effective distance via sine rule
            time = c / abs(speed)  # calculation of needed time via distance / speed
            return time
        else:
            return 0


def main(args=None):
    rclpy.init(args=args)
    parker = Parker()
    rclpy.spin(parker)
    overtaker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
