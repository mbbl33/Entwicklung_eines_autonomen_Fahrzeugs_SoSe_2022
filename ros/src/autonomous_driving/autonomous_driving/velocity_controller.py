import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        self.sub_laser = self.create_subscription(LaserScan, 'tof_Top', self.print_laser_info, 1)

    def print_laser_info(self, msg_in):
        print(msg_in.intensities)

def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()