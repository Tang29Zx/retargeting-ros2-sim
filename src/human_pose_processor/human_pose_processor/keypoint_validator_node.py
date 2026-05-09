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

    def __init__(self):
        super().__init__("keypoint_validator")

        self.declare_parameter("input_topic", "/hybrik_pose_world")
        self.declare_parameter("output_topic", "/retarget_keypoints")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.publisher = self.create_publisher(Joints3D, output_topic, 10)
        self.subscription = self.create_subscription(
            Joints3D,
            input_topic,
            self.pose_callback,
            10,
        )

    def pose_callback(self, msg):
        if len(msg.joints) != JOINT_COUNT:
            self.get_logger().warn(f"Expected {JOINT_COUNT} joints, got {len(msg.joints)}")
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

def main():
    rclpy.init()
    node = KeypointValidator()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
