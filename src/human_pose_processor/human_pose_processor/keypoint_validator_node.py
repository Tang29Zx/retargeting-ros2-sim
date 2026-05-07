import time

import rclpy
from rclpy.node import Node

from hybrik_msgs.msg import Joint3D, Joints3D


JOINT_COUNT = 29
KEYPOINTS = (
    ("left_hip", "左髋", 1),
    ("right_hip", "右髋", 2),
    ("left_knee", "左膝", 4),
    ("right_knee", "右膝", 5),
    ("left_ankle", "左踝", 7),
    ("right_ankle", "右踝", 8),
    ("left_shoulder", "左肩", 16),
    ("right_shoulder", "右肩", 17),
    ("left_elbow", "左肘", 18),
    ("right_elbow", "右肘", 19),
    ("left_wrist", "左腕", 20),
    ("right_wrist", "右腕", 21),
)


class KeypointValidator(Node):
    """Extract the 12 retargeting keypoints from the processed human pose."""

    def __init__(self):
        super().__init__("keypoint_validator")

        self.declare_parameter("input_topic", "/hybrik_pose_world")
        self.declare_parameter("output_topic", "/retarget_keypoints")
        self.declare_parameter("print_period", 1.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.print_period = float(self.get_parameter("print_period").value)
        self.last_print_time = 0.0

        self.publisher = self.create_publisher(Joints3D, output_topic, 10)
        self.subscription = self.create_subscription(
            Joints3D,
            input_topic,
            self.pose_callback,
            10,
        )

        keypoint_summary = ", ".join(
            f"{english}/{chinese}:{index}"
            for english, chinese, index in KEYPOINTS
        )
        self.get_logger().info(
            f"Extracting 12 keypoints from {input_topic} to {output_topic}: "
            f"{keypoint_summary}"
        )

    def pose_callback(self, msg):
        if len(msg.joints) != JOINT_COUNT:
            self.get_logger().warn(
                f"Expected {JOINT_COUNT} joints, got {len(msg.joints)}"
            )
            return

        output_msg = Joints3D()
        output_msg.header = msg.header

        for _, _, source_index in KEYPOINTS:
            source = msg.joints[source_index]
            keypoint = Joint3D()
            keypoint.x = source.x
            keypoint.y = source.y
            keypoint.z = source.z
            output_msg.joints.append(keypoint)

        self.publisher.publish(output_msg)
        self._print_stats(output_msg)

    def _print_stats(self, msg):
        now = time.monotonic()
        if now - self.last_print_time < self.print_period:
            return

        self.last_print_time = now
        joints = msg.joints
        xs = [joint.x for joint in joints]
        ys = [joint.y for joint in joints]
        zs = [joint.z for joint in joints]

        left_wrist = joints[10]
        right_wrist = joints[11]
        left_ankle = joints[4]
        right_ankle = joints[5]

        self.get_logger().info(
            "12 keypoints "
            f"x=[{min(xs):.3f}, {max(xs):.3f}] "
            f"y=[{min(ys):.3f}, {max(ys):.3f}] "
            f"z=[{min(zs):.3f}, {max(zs):.3f}] "
            f"Lwrist=({left_wrist.x:.2f},{left_wrist.y:.2f},{left_wrist.z:.2f}) "
            f"Rwrist=({right_wrist.x:.2f},{right_wrist.y:.2f},{right_wrist.z:.2f}) "
            f"Lankle.z={left_ankle.z:.2f} Rankle.z={right_ankle.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = KeypointValidator()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
