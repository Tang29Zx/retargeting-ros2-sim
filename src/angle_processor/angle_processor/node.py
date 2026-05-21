import rclpy
from rclpy.node import Node

from hybrik_msgs.msg import HumanJointAngles

from .processor import (
    AngleProcessor,
    DEFAULT_MAX_JUMP_DEG,
    DEFAULT_MOVING_AVERAGE_WINDOW,
)


class AngleProcessorNode(Node):
    def __init__(self):
        super().__init__('angle_processor')

        self.declare_parameter('input_topic', '/human_joint_angles')
        self.declare_parameter('output_topic', '/human_joint_angles_filtered')
        self.declare_parameter('max_jump_deg', DEFAULT_MAX_JUMP_DEG)
        self.declare_parameter(
            'moving_average_window',
            DEFAULT_MOVING_AVERAGE_WINDOW,
        )

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        max_jump_deg = self.get_parameter('max_jump_deg').value
        moving_average_window = self.get_parameter(
            'moving_average_window'
        ).value

        self.processor = AngleProcessor(
            max_jump_deg=max_jump_deg,
            moving_average_window=moving_average_window,
        )

        self.subscription = self.create_subscription(
            HumanJointAngles,
            input_topic,
            self.angle_callback,
            10,
        )
        self.publisher = self.create_publisher(
            HumanJointAngles,
            output_topic,
            10,
        )

        self.get_logger().info(
            'Angle processor started: '
            f'{input_topic} -> {output_topic}, '
            f'max_jump_deg={self.processor.max_jump_deg}, '
            f'moving_average_window={self.processor.moving_average_window}'
        )

    def angle_callback(self, msg):
        filtered_msg = self.processor.process(msg)
        self.publisher.publish(filtered_msg)


def main():
    rclpy.init()
    node = AngleProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
