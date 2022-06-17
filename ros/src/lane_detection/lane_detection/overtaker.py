import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from . import pid
class Overtaker(Node):
    def __init__(self):
        super().__init__('overtaker')

        #small front tof
        self.subFront2 = self.create_subscription(Range, '/tof_Front2', self.is_road_blocked, 1)


        self.pub_update_LD = self.create_publisher(Bool, 'stop_lane_based_steer', 1)

        self.subLeft = self.create_subscription(Range , 'tof_Left', self.look_left, 1)
        self.subRight = self.create_subscription(Range , 'tof_Right', self.update_right_dist, 1)

        self.right_dist = 0.0
        self.pid_controller = pid.PID_Controller(100, 0.001, 1, 45, -45, (1 / 5))
        # steering
        self.pub_steering = self.create_publisher(Float64, '/steering', 1)

        self.overtaker_mode = False
        self.left_lane_free = True

    def is_road_blocked(self, msg):
        range = min(msg.max_range, msg.range)
        if range < 0.75 and self.left_lane_free:
            #shut up lane detection
            out = Bool()
            out.data = True
            self.pub_update_LD.publish(out)
            steer = Float64()
            steer.data = -45.0
            print(steer)
            self.pub_steering.publish(steer)
            should_value = 0.15
            out = self.pid_controller.calc_pid(self.right_dist, should_value, 200, 0.1)
            msg_out = Float64()
            msg_out.data = float(out)
            self.publisher_steering.publish(msg_out)

        else:
            #out = Bool()
            #out.data = False
            #self.pub_update_LD.publish(out)
            print("nop")
            #vorlaufig
            #out.append(1.25)
            #out.append(140.0)
            #out.append(380.0)
        #msg_out.data = out
        #self.pubUpdateLD.publish(msg_out)
        print(msg.range)
    def update_right_dist(self, msg):
        self.right_dist = msg.range
    def look_left(self, msg):
        range = min( msg.max_range, msg.range)
        self.left_lane_free = range < 0.5




def main(args=None):
    rclpy.init(args=args)

    overtaker = Overtaker()
    rclpy.spin(overtaker)
    overtaker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()