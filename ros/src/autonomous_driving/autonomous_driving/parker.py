import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64, Bool


class Parker(Node):
    def __init__(self):
        super().__init__('parker')
        self.blocked = False
        self.current_phase = 0

        # ToF
        self.sub_right = self.create_subscription(Range, 'tof_Front_Right', self.look_for_parking, 1)
        #self.sub_left = self.create_subscription(Range, 'tof_Front_Left', self.look_left, 1)
        #self.sub_front_1 = self.create_subscription(Range, 'tof_Front', self.look_right, 1)
        #self.sub_back = self.create_subscription(Range, 'tof_Back', self.look_right, 1)

        # for overtaker mode
        self.sub_block_parking = self.create_subscription(Bool, 'block_parking', self.update_self_blocked, 1)

        # speed
        self.pub_speed = self.create_publisher(Float64, '/speed', 1)

        self.pub_block_lbs = self.create_publisher(Bool, 'block_lane_based_steer', 1)
        self.pub_block_overtaker = self.create_publisher(Bool, 'block_overtaker', 1)

    def update_self_blocked(self, msg_in):
        self.blocked = msg_in.data

    def look_for_parking(self, msg_in):
        print("phase", self.current_phase)
        print(msg_in.range < msg_in.max_range)
        if self.current_phase == 0:
            # looking for box on the left
            if msg_in.range < msg_in.max_range and not self.blocked:
                out = Bool()
                out.data = True
                self.pub_block_overtaker.publish(out)
                self.current_phase = 1

        elif self.current_phase == 1:
            # have passed a box
            if msg_in.max_range <= msg_in.range:
                self.current_phase = 2
        elif self.current_phase == 2:
            # found second box
            if msg_in.range < msg_in.max_range:
                out = Bool()
                out.data = True
                self.pub_block_lbs.publish(out)
                self.current_phase = 3
        elif self.current_phase == 3:
            #start parking
            msg_out = Float64()
            msg_out.data = 0.0
            self.pub_speed.publish(msg_out)
            print("jetzt wuerde ich einparken")
        else:
            return


def main(args=None):
    rclpy.init(args=args)
    parker = Parker()
    rclpy.spin(parker)
    overtaker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
