import rclpy
from . import pid
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64


class Tof_circle_driver(Node):
    def __init__(self):
        super().__init__('tof_circle_driver')
        self.pid_controller = pid.PID_Controller(100, 0.001, 1, 45, -45, (1 / 5))

        # /tof_Front_Left [sensor_msgs/msg/Range]
        self.subscription = self.create_subscription(Range, '/tof_Front_Left', self.tof_left_callback2, 1)

        # steering
        self.publisher_steering = self.create_publisher(Float64, '/steering', 1)

    def tof_left_callback2(self, msg):
        should_value = 1.4
        out = self.pid_controller.calc_pid(msg.range, should_value, msg.max_range, msg.min_range)
        msg_out = Float64()
        msg_out.data = float(out)
        self.publisher_steering.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    tof_circle_driver = Tof_circle_driver()
    rclpy.spin(tof_circle_driver)
    tof_circle_driver.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()
