import rclpy
from rclpy.node import Node
from hybrik_msgs import Joints3D, HumanJointAngles

class HumanModelFitter(Node):
    def __init__(self):
        super.__init__('human_model_fitter')

        self.declare_parameter('input_topic', '/retarget_key_points')
        self.declare_parameter('output_topic', '/human_joint_angles')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.subscription = self.create_subscription(
            Joints3D, input_topic, self.pose_callback, 10
        )
        self.publisher = self.create_publisher(
            HumanJointAngles, output_topic, 10
        )

    def pose_callback(self):
        pass



def main():
    rclpy.init()
    node = HumanModelFitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()