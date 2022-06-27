import rclpy
from rclpy.node import Node
import threading
from .lane_based_steer import LaneBasedSteer
from .overtaker import Overtaker
from .parker import Parker

def main(args=None):
    rclpy.init(args=args)

    lane_based_steering = LaneBasedSteer()
    overtaker = Overtaker()
    parker = Parker()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(lane_based_steering)
    executor.add_node(overtaker)
    executor.add_node(parker)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = lane_based_steering.create_rate(1)
    rate2 = overtaker.create_rate(1)
    
    while rclpy.ok():
        rate.sleep()
        rate2.sleep()

    lane_based_steering.destroy_node()
    overtaker.destroy_node()
    parker.destroy_node()
    rclpy.shutdown()
    executor_thread.join()


if __name__ == '__main__':
    main()
