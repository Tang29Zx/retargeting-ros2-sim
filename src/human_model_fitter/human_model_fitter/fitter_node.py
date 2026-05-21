import rclpy
from rclpy.node import Node
<<<<<<< HEAD
from hybrik_msgs import Joints3D, HumanJointAngles

class HumanModelFitter(Node):
    def __init__(self):
        super.__init__('human_model_fitter')

        self.declare_parameter('input_topic', '/retarget_key_points')
        self.declare_parameter('output_topic', '/human_joint_angles')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
=======
from hybrik_msgs.msg import Joints3D, HumanJointAngle, HumanJointAngles

from .variables import HumanKeypoints, HumanLength
from .udeas import UDEAS, VARIABLE_NAMES

LENGTH_FIELD_NAMES = (
    "left_upper_arm",
    "right_upper_arm",
    "left_forearm",
    "right_forearm",
    "left_thigh",
    "right_thigh",
    "left_shin",
    "right_shin",
    "hip_width",
    "shoulder_width",
    "left_hip_shoulder",
    "right_hip_shoulder",
)

class HumanModelFitter(Node):
    def __init__(self):
        super().__init__('human_model_fitter')

        self.declare_parameter('input_topic', '/retarget_keypoints')
        self.declare_parameter('output_topic', '/human_joint_angles')
        self.declare_parameter('length_init_frames', 30)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.length_init_frames = self.get_parameter('length_init_frames').value

        self.length_samples = []
        self.human_length = None
        self.udeas = None
        self.last_variables = None
        self.last_loss = None
>>>>>>> 86661d60018d83c142dcaa71ce7dc41438f088e7

        self.subscription = self.create_subscription(
            Joints3D, input_topic, self.pose_callback, 10
        )
        self.publisher = self.create_publisher(
            HumanJointAngles, output_topic, 10
        )

<<<<<<< HEAD
    def pose_callback(self):
        pass


=======
    def pose_callback(self, msg):
        if self.human_length is None:
            self.collect_length_sample(msg)
            if self.human_length is None:
                return

        self.fit_and_publish(msg)

    def collect_length_sample(self, msg):
        if len(msg.joints) < 12:
            self.get_logger().warn(
                f'Need 12 joints to estimate human length, got {len(msg.joints)}.'
            )
            return

        points = HumanKeypoints(msg)
        length = HumanLength(points)
        self.length_samples.append(length)

        sample_count = len(self.length_samples)
        if sample_count == 1 or sample_count % 10 == 0:
            self.get_logger().info(
                f'Collecting human length samples: '
                f'{sample_count}/{self.length_init_frames}'
            )

        if sample_count >= self.length_init_frames:
            self.human_length = self.average_human_length(self.length_samples)
            self.udeas = UDEAS(self.human_length)
            self.get_logger().info(
                'Human length initialized from '
                f'{sample_count} frames: {self.human_length_summary()}'
            )

    def average_human_length(self, samples):
        human_length = HumanLength()
        for name in LENGTH_FIELD_NAMES:
            value = sum(getattr(sample, name) for sample in samples) / len(samples)
            setattr(human_length, name, value)
        return human_length

    def human_length_summary(self):
        return ', '.join(
            f'{name}={getattr(self.human_length, name):.4f}'
            for name in LENGTH_FIELD_NAMES
        )

    def fit_and_publish(self, msg):
        if len(msg.joints) < 12:
            self.get_logger().warn(
                f'Need 12 joints to fit human variables, got {len(msg.joints)}.'
            )
            return
        
        fitted_variables, loss = self.udeas.fit(msg, self.last_variables)
        self.last_variables = fitted_variables
        self.last_loss = loss

        output_msg = self.variables_to_msg(fitted_variables, msg.header)
        self.publisher.publish(output_msg)
        self.get_logger().debug(
            f'Published fitted human joint angles, loss={loss:.4f}'
        )

    def variables_to_msg(self, variables, header):
        msg = HumanJointAngles()
        msg.header = header
        msg.angles = []

        for name in VARIABLE_NAMES:
            angle_msg = HumanJointAngle()
            angle_msg.name = name
            angle_msg.angle = float(getattr(variables, name))
            msg.angles.append(angle_msg)

        return msg
>>>>>>> 86661d60018d83c142dcaa71ce7dc41438f088e7

def main():
    rclpy.init()
    node = HumanModelFitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
<<<<<<< HEAD
    main()
=======
    main()
>>>>>>> 86661d60018d83c142dcaa71ce7dc41438f088e7
