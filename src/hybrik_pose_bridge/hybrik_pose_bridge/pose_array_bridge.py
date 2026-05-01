from geometry_msgs.msg import Pose, PoseArray
from hybrik_msgs.msg import Joints3D
import rclpy
from rclpy.node import Node


class PoseArrayBridge(Node):
    """Convert HybrIK joints into a standard PoseArray for visualization."""

    def __init__(self):
        super().__init__('hybrik_pose_array_bridge')

        self.declare_parameter('input_topic', '/hybrik_pose')
        self.declare_parameter('output_topic', '/hybrik_pose_points')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.publisher = self.create_publisher(PoseArray, output_topic, 10)
        self.subscription = self.create_subscription(
            Joints3D,
            input_topic,
            self.pose_callback,
            10,
        )

        self.get_logger().info(
            f'Bridging {input_topic} (hybrik_msgs/Joints3D) '
            f'to {output_topic} (geometry_msgs/PoseArray)'
        )

    def pose_callback(self, msg):
        pose_array = PoseArray()
        pose_array.header = msg.header

        for joint in msg.joints:
            pose = Pose()
            pose.position.x = joint.x
            pose.position.y = joint.y
            pose.position.z = joint.z
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.publisher.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = PoseArrayBridge()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
