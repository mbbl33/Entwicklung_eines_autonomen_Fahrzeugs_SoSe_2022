from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    lbs_node = Node(
        package="autonomous_driving",
        executable="lane_based_steer",
    )
    overtaker_node = Node(
        package="autonomous_driving",
        executable="overtaker"
    )
    parker_node = Node(
        package="autonomous_driving",
        executable="parker"
    )
    ld.add_action(lbs_node)
    ld.add_action(overtaker_node)
    ld.add_action(parker_node)
    
    return ld