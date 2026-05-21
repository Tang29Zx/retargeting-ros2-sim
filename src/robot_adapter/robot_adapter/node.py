import rclpy
from rclpy.node import Node

from hybrik_msgs.msg import HumanJointAngles
from sensor_msgs.msg import JointState

from .g1_adapter import DEFAULT_MAX_STEP_RAD, G1Adapter


class RobotAdapterNode(Node):
    def __init__(self):
        super().__init__('robot_adapter')

        self.declare_parameter('input_topic', '/human_joint_angles_filtered')
        self.declare_parameter('output_topic', '/g1_joint_targets')
        self.declare_parameter('max_step_rad', DEFAULT_MAX_STEP_RAD)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        max_step_rad = self.get_parameter('max_step_rad').value

        self.adapter = G1Adapter(max_step_rad=max_step_rad)

        self.subscription = self.create_subscription(
            HumanJointAngles,
            input_topic,
            self.angle_callback,
            10,
        )
        self.publisher = self.create_publisher(
            JointState,
            output_topic,
            10,
        )

        self.get_logger().info(
            'Robot adapter started: '
            f'{input_topic} -> {output_topic}, '
            f'max_step_rad={self.adapter.max_step_rad}'
        )

    def angle_callback(self, msg):
        human_angles = self.angles_to_dict(msg)
        names, positions = self.adapter.adapt(human_angles)

        output_msg = JointState()
        output_msg.header = msg.header
        output_msg.name = names
        output_msg.position = positions

        self.publisher.publish(output_msg)

    def angles_to_dict(self, msg):
        return {
            angle.name: float(angle.angle)
            for angle in msg.angles
        }


def main():
    rclpy.init()
    node = RobotAdapterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
