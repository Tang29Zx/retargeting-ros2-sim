import rclpy
from rclpy.node import Node
from hybrik_msgs.msg import Joints3D, Joint3D

JOINT_COUNT = 29
SCALE = 1.85
ANCHOR = (0.0, 0.0, 0.95)
MAX_JUMP = 0.35
FLITER_ALPHA = 0.35
ENABLE_MOVING_AVERAGE = False
MOVING_AVERAGE_WINDOW = 3

class HumanPoseProcessor(Node):
    def __init__(self):
        super().__init__('human_pose_processor')
        self.declare_parameter('input_topic', '/hybrik_pose')
        self.declare_parameter('output_topic', '/hybrik_pose_world')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.joints_history = []
        self.prev_joints_msg = None
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

    def max_jump(self, joints_msg):
        joints = joints_msg.joints
        prev_joints = self.prev_joints_msg.joints

        for n in range(0, len(joints)):
            diffx = joints[n].x - prev_joints[n].x
            if diffx > MAX_JUMP:
                joints_msg.joints[n].x = self.prev_joints_msg.joints[n].x + MAX_JUMP
            elif diffx < -MAX_JUMP:
                joints_msg.joints[n].x = self.prev_joints_msg.joints[n].x - MAX_JUMP

            diffy = joints[n].y - prev_joints[n].y
            if diffy > MAX_JUMP:
                joints_msg.joints[n].y = self.prev_joints_msg.joints[n].y + MAX_JUMP
            elif diffy < -MAX_JUMP:
                joints_msg.joints[n].y = self.prev_joints_msg.joints[n].y - MAX_JUMP

            diffz = joints[n].z - prev_joints[n].z
            if diffz > MAX_JUMP:
                joints_msg.joints[n].z = self.prev_joints_msg.joints[n].z + MAX_JUMP
            elif diffz < -MAX_JUMP:
                joints_msg.joints[n].z = self.prev_joints_msg.joints[n].z - MAX_JUMP

        return joints_msg

    def low_pass_fliter(self, joints_msg):
        joints = joints_msg.joints
        prev_joints = self.prev_joints_msg.joints
        
        for n in range(0, len(joints)):
            joints_msg.joints[n].x = FLITER_ALPHA * joints[n].x + (1 - FLITER_ALPHA) * prev_joints[n].x
            joints_msg.joints[n].y = FLITER_ALPHA * joints[n].y + (1 - FLITER_ALPHA) * prev_joints[n].y
            joints_msg.joints[n].z = FLITER_ALPHA * joints[n].z + (1 - FLITER_ALPHA) * prev_joints[n].z

        return joints_msg

    def moving_average(self, joints_msg):
        self.joints_history.append(joints_msg)
        if len(self.joints_history) > MOVING_AVERAGE_WINDOW:
            self.joints_history.pop(0)

        for n in range(0, len(joints_msg.joints)):
            total_x = 0.0
            total_y = 0.0
            total_z = 0.0
            for j in self.joints_history:
                total_x += j.joints[n].x
                total_y += j.joints[n].y
                total_z += j.joints[n].z

            count = len(self.joints_history)
            avg_x = total_x / count
            avg_y = total_y / count
            avg_z = total_z / count

            joints_msg.joints[n].x = avg_x
            joints_msg.joints[n].y = avg_y
            joints_msg.joints[n].z = avg_z

        return joints_msg

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
        
        if self.prev_joints_msg is not None:
            neomsg = self.max_jump(neomsg)
            neomsg = self.low_pass_fliter(neomsg)
            
        if ENABLE_MOVING_AVERAGE is True:
            neomsg = self.moving_average(neomsg)
        
        self.prev_joints_msg = neomsg

        self.publisher.publish(neomsg)

def main():
    rclpy.init()
    node = HumanPoseProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
