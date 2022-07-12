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
        self.right_lane_free = True
        self.right_box_found = False
        self.reverse_speed = -0.5  # TO BE CHANGED - VELOCITY CONTROL
        self.general_speed = 1.0  # TO BE CHANGED - VELOCITY CONTROL
        self.speed = 0
        self.back_distance = 10.0

        # ToF subscriptions
        self.sub_right = self.create_subscription(Range, 'tof_Front_Right', self.look_for_parking, 1)
        self.sub_back = self.create_subscription(Range, 'tof_Back', self.update_back_distance, 1)

        # Blocking publications for lbs and overtaker and velocity_controller
        self.pub_block_lbs = self.create_publisher(Bool, 'block_lane_based_steer', 1)
        self.pub_block_overtaker = self.create_publisher(Bool, 'block_overtaker', 1)
        self.pub_block_velocity_controller = self.create_publisher(Bool, 'block_velocity_controller', 1)

        # Blocking subscription from overtaker
        self.sub_block_parking = self.create_subscription(Bool, 'block_parking', self.update_self_blocked, 1)

        # Speed and Steering publication
        self.sub_speed = self.create_subscription(Float64, '/speed', self.update_current_speed, 1)
        self.pub_speed = self.create_publisher(Float64, '/speed', 1)
        self.pub_steering = self.create_publisher(Float64, '/steering', 1)

        self.sub_box_detect = self.create_subscription(Bool, 'early_box_detection', self.find_second_box, 1)


    def update_self_blocked(self, msg_in):
        self.blocked = msg_in.data

    def find_second_box(self, msg_in):
        self.right_box_found = msg_in.data

    def look_for_parking(self, msg_in):
        self.look_right(msg_in)
        if self.scan_blocked == False:
            if self.current_scan_phase == 0:
                # Phase 0 - Looking for box on the left
                if self.right_lane_free == False and not self.blocked:
                    block = Bool()
                    block.data = True
                    self.pub_block_overtaker.publish(block)
                    self.pub_block_velocity_controller.publish(block)
                    self.current_scan_phase = 1
                    print("---SCAN: PHASE TO 1---")

            elif self.current_scan_phase == 1:
                # Phase 1 - First box passed, looking for second box
                if self.right_lane_free == True:
                    self.current_scan_phase = 2
                    print("---SCAN: PHASE TO 2---")

            elif self.current_scan_phase == 2:
                # Phase 2 - Second box passed
                if self.right_box_found == True:
                    self.current_scan_phase = 3
                    print("---SCAN: PHASE TO 3---")

            elif self.current_scan_phase == 3:
                # Phase 3 - Initiate Parking maneuver
                speed = Float64()
                speed.data = 0.0
                self.pub_speed.publish(speed)
                self.scan_blocked = True
                print("---INITITATE PARKING---")
                self.current_parking_phase = 1
                print("---SET CURRENT PHASE TO 1---")
            else:
                return
        else:
            self.parking()

    def update_back_distance(self, msg_in):
        self.back_distance = msg_in.range

    def update_current_speed(self, msg):
        self.speed = msg.data

    def parking(self):
        # Phase 1 - reversing until parking lane is free
        if self.current_parking_phase == 1:
            speed = Float64()
            speed.data = self.reverse_speed
            self.pub_speed.publish(speed)
            if self.right_lane_free == True:
                block_lbs = Bool()
                block_lbs.data = True
                self.pub_block_lbs.publish(block_lbs)
                self.current_parking_phase = 2
        # Phase 2 - steering into the parking spot
        elif self.current_parking_phase == 2:
            time.sleep(0.4)
            self.change_lane(25.0, 1, self.reverse_speed)
            print("---SET CURRENT PHASE TO 2---")
            self.current_parking_phase = 3
        # Phase 3 - closing in on parked car and stopping
        elif self.current_parking_phase == 3:
            if self.back_distance < 0.25:
                speed = Float64()
                speed.data = 0.0
                self.pub_speed.publish(speed)
                print("---SET CURRENT PHASE TO 3---")
                self.current_parking_phase = 4
        # Phase 4 - forward lane switch and unblock
        elif self.current_parking_phase == 4:
            speed = Float64()
            speed.data = self.general_speed
            self.pub_speed.publish(speed)
            time.sleep(0.2)
            self.change_lane(-25.0, 1, self.general_speed)
            print("---PARKING COMPLETE---")
            speed = Float64()
            speed.data = 2.0
            self.pub_speed.publish(speed)
            self.next_lap_prep()
            print("---VARIABLES RESET FOR NEXT LAP---")

        else:
            return

    def next_lap_prep(self):
        # Unblock overtaker and lbs and velocity_controller
        block = Bool()
        block.data = False
        self.pub_block_overtaker.publish(block)
        self.pub_block_lbs.publish(block)
        self.pub_block_velocity_controller.publish(block)
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

    def look_right(self, msg):
        self.right_lane_free = 0.8 < msg.range


def main(args=None):
    rclpy.init(args=args)
    parker = Parker()
    rclpy.spin(parker)
    parker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()