from .inferencer import HybrikInferencer
import cv2
import rclpy
from rclpy.node import Node
from hybrik_msgs.msg import Joint3D, Joints3D

class hybrik(Node):
    def __init__(self):
        super().__init__('hybrik')
        self.get_logger().info('hybrik setup')
        self.publisher = self.create_publisher(Joints3D, 'hybrik_pose', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.infer = HybrikInferencer("/home/tang/robotics")

    def _get_joints(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        
        pose_output = self.infer.run_model(frame)
        if pose_output is None:
            return None
        return pose_output.pred_xyz_jts_29.reshape(-1, 3).detach().cpu().numpy()

    def timer_callback(self):
        points = self._get_joints()

        if points is None:
            return

        msg = Joints3D()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        for p in points:
            joint = Joint3D()
            joint.x = float(p[0])
            joint.y = float(p[1])
            joint.z = float(p[2])

            msg.joints.append(joint)

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = hybrik()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


