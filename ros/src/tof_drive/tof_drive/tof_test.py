import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class Tof_Test(Node):
    def __init__(self):
        super().__init__('tof_test')
        #/tof_0 [sensor_msgs/msg/Range]
        self.subscription = self.create_subscription(Range, '/tof_0', self.printDist, 1)

    def printDist(self, msg):
        print(msg.range)


def main(args=None):
    rclpy.init(args=args)
    tof_Test = Tof_Test()
    rclpy.spin(tof_Test)
    tof_Test.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()