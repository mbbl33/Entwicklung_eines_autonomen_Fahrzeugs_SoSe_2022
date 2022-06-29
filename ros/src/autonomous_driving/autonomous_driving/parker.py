import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64, Bool
import time
import math


class Parker(Node):
    def __init__(self):
        super().__init__('parker')
        
        # Parking variables
        self.blocked = False
        self.scan_blocked = False
        self.current_scan_phase = 0
        self.current_parking_phase = 0
        self.reverse_speed = -0.2  # TO BE CHANGED - VELOCITY CONTROL
        self.general_speed = 0.4  # TO BE CHANGED - VELOCITY CONTROL
        self.speed = 0
        self.back_distance = 10.0

        # ToF subscriptions
        self.sub_right = self.create_subscription(Range, 'tof_Front_Right', self.look_for_parking, 1)
        self.sub_back = self.create_subscription(Range, 'tof_Back', self.get_back_distance, 1)

        # Blocking publications for lbs and overtaker
        self.pub_block_lbs = self.create_publisher(Bool, 'block_lane_based_steer', 1)
        self.pub_block_overtaker = self.create_publisher(Bool, 'block_overtaker', 1)

        # Blocking subscription from overtaker
        self.sub_block_parking = self.create_subscription(Bool, 'block_parking', self.update_self_blocked, 1)

        # Speed and Steering publication
        self.sub_speed = self.create_subscription(Float64, '/speed', self.get_current_speed, 1)
        self.pub_speed = self.create_publisher(Float64, '/speed', 1)
        self.pub_steering = self.create_publisher(Float64, '/steering', 1)


    def update_self_blocked(self, msg_in):
        self.blocked = msg_in.data

    def look_for_parking(self, msg_in):
        if self.scan_blocked == False:
            if self.current_scan_phase == 0:
                # Phase 0 - Looking for box on the left
                if msg_in.range < msg_in.max_range and not self.blocked:
                    out = Bool()
                    out.data = True
                    self.pub_block_overtaker.publish(out)
                    self.current_scan_phase = 1

            elif self.current_scan_phase == 1:
                # Phase 1 - First box passed, looking for second box
                if msg_in.max_range <= msg_in.range:
                    self.current_scan_phase = 2
            elif self.current_scan_phase == 2:
                # Phase 2 - Second box passed, disable lane based steering
                if msg_in.range < msg_in.max_range:
                    out = Bool()
                    out.data = True
                    self.pub_block_lbs.publish(out)
                    self.current_scan_phase = 3
            elif self.current_scan_phase == 3:
                # Phase 3 - Initiate Parking maneuver
                msg_out = Float64()
                msg_out.data = 0.0
                self.pub_speed.publish(msg_out)
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

    def get_current_speed(self, msg):
        self.speed = msg.data
        print("PARKER: ", self.speed)

    def parking(self):
        # Phase 1 - Determine reverse speed, allignment
        if self.current_parking_phase == 1:
            # move forward
            msg_out = Float64()
            msg_out.data = self.general_speed
            self.pub_speed.publish(msg_out)
            time.sleep(0.5)
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
                print("---SET CURRENT PHASE TO 3---")
                self.current_parking_phase = 3
        # Phase 3 - forward lane switch and unblock
        elif self.current_parking_phase == 3:
            msg_out = Float64()
            msg_out.data = self.general_speed
            self.pub_speed.publish(msg_out)
            self.change_lane(-25.0, 1, self.general_speed)
            print("---PARKING COMPLETE---")
            self.next_lap_prep()
            print("---VARIABLES RESET FOR NEXT LAP---")

        else:
            return

    def next_lap_prep(self):
        # Unblock overtaker and lbs
        out = Bool()
        out.data = False
        self.pub_block_overtaker.publish(out)
        self.pub_block_lbs.publish(out)
        # Reset phases
        self.current_parking_phase = 0
        self.current_scan_phase = 0
        # Wait a bit to unblock scan
        time.sleep(20)
        self.scan_blocked = False

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
            c = (b * math.sin(gamma)) / math.sin(beta)  # Calculation of effective distance via sine rule
            time = c / abs(speed)  # Calculation of needed time via distance / speed
            return time
        else:
            return 0


def main(args=None):
    rclpy.init(args=args)
    parker = Parker()
    rclpy.spin(parker)
    parker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
