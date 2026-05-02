import rclpy
from rclpy.node import Node
from hybrik_msgs.msg import Joints3D, Joint3D

JOINT_COUNT = 29
SCALE = 1.85
ANCHOR = (0.0, 0.0, 0.95)

class HumanPoseProcessor(Node):
    def __init__(self):
        super().__init__('human_pose_processor')
        self.declare_parameter('input_topic', '/hybrik_pose')
        self.declare_parameter('output_topic', '/hybrik_pose_world')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.subscription = self.create_subscription(
            Joints3D,
            input_topic,
            self.pose_callback,
            10,
        )
        self.publisher = self.create_publisher(
            Joints3D,
            output_topic,
            10,
        )

    def pose_callback(self, msg):
        Joints = msg.joints

        if len(Joints) != JOINT_COUNT:
            self.get_logger().warn(f"Expected {JOINT_COUNT} joints, got {len(msg.joints)}")
            return

        neomsg = Joints3D()
        neomsg.header.stamp = self.get_clock().now().to_msg()
        neomsg.header.frame_id = 'world'

        for joint in Joints:
            new_joint = Joint3D()
            new_joint.x = ANCHOR[0] + SCALE * joint.x
            new_joint.y = ANCHOR[1] - SCALE * joint.z
            new_joint.z = ANCHOR[2] - SCALE * joint.y

            neomsg.joints.append(new_joint)
        
        self.publisher.publish(neomsg)

def main():
    rclpy.init()
    node = HumanPoseProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

