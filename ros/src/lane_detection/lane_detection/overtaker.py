import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
class Overtaker(Node):
    def __init__(self):
        super().__init__('overtaker')
        self.subFront2 = self.create_subscription(Range, '/tof_Front2', self.is_road_blocked, 1)
        self.pubUpdateLD = self.create_publisher(Float64MultiArray, 'update_lane_detection', 1)
        self.subLeft = self.create_subscription(Range , 'tof_Left', self.look_left, 1)
        self.overtaker_mode = False
        self.left_lane_free = True

    def is_road_blocked(self, msg_in):
        out = []
        msg_out = Float64MultiArray()
        if msg_in.range < 0.75 and self.left_lane_free:
            out.append(0.85)
            out.append(260.0)
            out.append(500.0)
            msg_out.data = out
            self.pubUpdateLD.publish(msg_out)

        else:
            print("nop")
            #vorlaufig
            #out.append(1.25)
            #out.append(140.0)
            #out.append(380.0)
        #msg_out.data = out
        #self.pubUpdateLD.publish(msg_out)
        print(msg_in.range)

    def look_left(self, msg):
        self.left_lane_free = msg.range < 0.5




def main(args=None):
    rclpy.init(args=args)

    overtaker = Overtaker()
    rclpy.spin(overtaker)
    overtaker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()