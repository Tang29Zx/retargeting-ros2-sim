import cv2
import rclpy
from rclpy.node import Node
from hybrik_msgs.msg import Joint3D, Joints3D
from .tcp_client import tcp_socket

class hybrik(Node):
    def __init__(self):
        super().__init__('hybrik')
        self.get_logger().info('hybrik setup')
        self.publisher = self.create_publisher(Joints3D, 'hybrik_pose', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.cap = cv2.VideoCapture(0)

    def _get_joints(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        
        try:
            joints = tcp_socket(frame)
            if joints is None:
                return None
        except Exception as e:
            self.get_logger().error(str(e))
            return None
        
        return joints

    def timer_callback(self):
        joints = self._get_joints()

        if joints is None:
            return

        msg = Joints3D()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        for p in joints:
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


