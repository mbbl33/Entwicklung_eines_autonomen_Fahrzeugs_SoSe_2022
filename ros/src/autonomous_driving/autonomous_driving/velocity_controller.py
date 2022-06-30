import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import time

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.sub_steering = self.create_subscription(Float64, '/steering', self.set_velocity, 1)
        self.sub_speed = self.create_subscription(Float64, '/speed', self.get_current_speed, 1)
        self.publisher_speed = self.create_publisher(Float64, '/speed', 1)

        # for overtaker and parking
        self.sub_block_lbs = self.create_subscription(Bool, 'block_lane_based_steer', self.update_blocked, 1)
        self.current_speed = 0.0
        self.blocked = False
        self.last_acceleration = 0.0

    def update_blocked(self, msg_in):
        self.blocked = msg_in.data

    def set_velocity(self, msg_in):
        if self.blocked:
            return
        msg_out = Float64()
        max_speed = 2.5
        min_speed = 0.5
        delay = 0.5 # in seconds
        step_size = 0.1
        acceleration_threshold = 5  # in degree
        if  acceleration_threshold < abs(msg_in.data):
            #slowdown
            self.current_speed = min_speed
            msg_out.data = self.current_speed
            self.publisher_speed.publish(msg_out)
        elif self.current_speed < max_speed and delay + self.last_acceleration < time.time():
            # acceleration
            self.current_speed += step_sizeS
            self.last_acceleration = time.time()
        msg_out.data = self.current_speed
        print(self.current_speed)
        self.publisher_speed.publish(msg_out)

    def get_current_speed(self, msg_in):
        self.current_speed = msg_in.data

def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()