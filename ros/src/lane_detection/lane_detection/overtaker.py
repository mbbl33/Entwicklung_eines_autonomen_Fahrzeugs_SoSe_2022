import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64

class Overtaker(Node):
    def __init__(self):
        print("aa")
        super().__init__('overtaker')
        self.subscription = self.create_subscription(Range, '/tof_Front2', self.callback, 1)

    def callback(self, msg):
        print(msg.range)

def main(args=None):
    rclpy.init(args=args)

    overtaker = Overtaker()
    rclpy.spin(overtaker)
    overtaker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()