import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import time

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.sub_steering = self.create_subscription(Float64, '/steering', self.set_velocity, 1)
        self.sub_speed = self.create_subscription(Float64, '/speed', self.update_current_speed, 1)
        self.pub_speed = self.create_publisher(Float64, '/speed', 1)
        self.pub_default_max_speed =  self.create_publisher(Float64, 'default_max_speed', 1)

        # for overtaker and parking
        self.sub_block_lbs = self.create_subscription(Bool, 'block_velocity_controller', self.update_blocked, 1)
        self.current_speed = 0.0
        self.max_speed = 2.5
        self.min_speed = 0.5
        self.counter = 1
        self.blocked = False
        self.last_acceleration = 0.0

    def update_blocked(self, msg_in):
        self.blocked = msg_in.data

    def pup_default_max_speed(self):
        out = Float64()
        out.data = self.default_max_speed
        self.pub_default_max_speed.publish(out)

    def set_velocity(self, msg_in):
        if self.blocked:
            return
        msg_out = Float64()
        delay = 0.5 # in seconds
        step_size = 0.3
        acceleration_threshold = 1.8  # in degree
        if  acceleration_threshold < abs(msg_in.data):
            #slowdown
            self.current_speed = self.min_speed
            msg_out.data = self.current_speed
            self.pub_speed.publish(msg_out)
            self.counter= 1
        elif self.current_speed < self.max_speed and delay + self.last_acceleration < time.time():
            # acceleration
            self.current_speed += (step_size * self.counter)
            self.last_acceleration = time.time()
        msg_out.data = self.current_speed

        self.pub_speed.publish(msg_out)

    def update_current_speed(self, msg_in):
        self.current_speed = msg_in.data
        print(self.current_speed)

def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()