import rclpy
#import time
from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Float64
import time

from pynput import keyboard
#from pynput.keyboard import Listener, Key, Controller


class Steer_Direction(float,Enum):
    Right = 1.0
    Left = -1.0


class Drive_Direction(float,Enum):
    Forward = 0.1
    Backward = -0.1
    Reset = 0.0

class Change_Lane_Direction(float,Enum):
    Right = 15.0
    Left = -15.0


class Key_Controller(Node):
    def __init__(self):
        super().__init__('key_controller2')

        # speed
        self.current_speed = 0.0
        self.publisher_speed = self.create_publisher(Float64, '/speed', 1)

        # steering
        self.current_steering = 0.0
        self.publisher_steering = self.create_publisher(
            Float64, '/steering', 1)

        # keyboard stuff
        listener = keyboard.Listener(on_press=self.drive)
        listener.start()

    def steer(self, direction):
        self.current_steering += direction.value

    def change_Speed(self, direction):
        # bitte nicht hauen :(
        self.current_speed = (self.current_speed + direction.value, 0.0)[direction == Drive_Direction.Reset]

    keyboard_action_map = {
        keyboard.Key.up: Drive_Direction.Forward,
        keyboard.Key.down: Drive_Direction.Backward,
        keyboard.Key.left: Steer_Direction.Left,
        keyboard.Key.right: Steer_Direction.Right,
        keyboard.Key.page_up: Change_Lane_Direction.Left,
        keyboard.Key.page_down: Change_Lane_Direction.Right,
        keyboard.Key.shift_r: Drive_Direction.Reset
    }

    def drive(self, key):
        msg = Float64()
        try:
            if(type(self.keyboard_action_map[key]) == Drive_Direction):
                self.change_Speed(self.keyboard_action_map[key])
                msg.data = self.current_speed
                self.publisher_speed.publish(msg)
            else:
                
                if key == keyboard.Key.page_up or keyboard.Key.page_down:
                    self.steer(self.keyboard_action_map[key])
                    msg.data = self.current_steering
                    self.publisher_steering.publish(msg)
                    time.sleep(3.33)
                    self.current_steering *= -1
                    msg.data = self.current_steering
                    self.publisher_steering.publish(msg)
                    time.sleep(3.33)
                    self.current_steering = 0.0
                    msg.data = self.current_steering
                    self.publisher_steering.publish(msg)

                else:
                    self.steer(self.keyboard_action_map[key])
                    msg.data = self.current_steering
                    self.publisher_steering.publish(msg)
            print("Taste [%s] \t Geschwindigkeit[%2.1f m/s]\t Winkel[%d]" % (str(key), self.current_speed, self.current_steering))
        except KeyError:
            print("[%s] ist keine gueltige Taste, nutze die Pfeiltasten oder das rechte Shift" % key)

def main(args=None):
    rclpy.init(args=args)
    key_Controller = Key_Controller()
    rclpy.spin(key_Controller)
    key_Controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
