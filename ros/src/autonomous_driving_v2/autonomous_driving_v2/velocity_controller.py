import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import time

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        
        # speed sub/pub
        self.sub_steering = self.create_subscription(Float64, '/steering', self.speed_up, 1)
        self.sub_speed = self.create_subscription(Float64, '/speed', self.update_current_speed, 1)
        self.pub_speed = self.create_publisher(Float64, '/speed', 1)

        # for overtaker and parking
        self.sub_block_lbs = self.create_subscription(Bool, 'block_velocity_controller', self.update_blocked, 1)

        self.sub_parking_pass = self.create_subscription(Bool, 'parking_done', self.update_parker, 1)
        self.sub_overtaking_pass = self.create_subscription(Bool, 'overtaking_done', self.update_overtaker, 1)

        self.target_speed = 2.5
        self.original_speed = 2.0
        self.blocked = False
        self.parking_passed = False
        self.overtaking_passed = False

    def update_blocked(self, msg_in):
        self.blocked = msg_in.data

    def update_parker(self, msg_in):
        self.parking_passed = msg_in.data

    def update_overtaker(self, msg_in):
        self.overtaking_passed = msg_in.data

    def speed_up(self, msg_in):
        if self.blocked:
            return 
        else:
            if self.parking_passed == True and self.overtaking_passed == True:
                print("---HINDERNISSE WURDEN PASSIERT---")
                time.sleep(4)
                print("---BESCHLEUNIGE...---")
                self.set_speed(self.stabilise(msg_in.data))
                time.sleep(1.5)
                print("---VERLANGSAME FÜR ERSTE KURVE---")
                self.set_speed(self.original_speed)
                time.sleep(1.4)
                print("---BESCHLEUNIGE...---")
                self.set_speed(self.stabilise(msg_in.data))
                time.sleep(2)
                print("---BESCHLEUNIGUNG ABGESCHLOSSEN; SETZE WERTE ZURÜCK---")
                self.set_speed(self.original_speed)
                self.overtaking_passed = False
                self.parking_passed = False
            else:
                self.set_speed(self.original_speed)

    def set_speed(self, speed):
        msg_out = Float64()
        msg_out.data = speed
        self.pub_speed.publish(msg_out)

    def stabilise(self, steering_angle):
        speed_diff = self.target_speed - self.original_speed
        factor = speed_diff * (1.0 - (steering_angle / 45.0))
        return 2.0 + factor

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