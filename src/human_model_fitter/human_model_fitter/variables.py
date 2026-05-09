'''
人体29点定义
0   pelvis          骨盆
1   left_hip        左髋
2   right_hip       右髋
3   spine1          脊柱1 / 下脊柱
4   left_knee       左膝
5   right_knee      右膝
6   spine2          脊柱2 / 中脊柱
7   left_ankle      左踝
8   right_ankle     右踝
9   spine3          脊柱3 / 上脊柱
10  left_foot       左脚
11  right_foot      右脚
12  neck            颈部
13  left_collar     左锁骨
14  right_collar    右锁骨
15  jaw             下颌
16  left_shoulder   左肩
17  right_shoulder  右肩
18  left_elbow      左肘
19  right_elbow     右肘
20  left_wrist      左腕
21  right_wrist     右腕
22  left_thumb      左拇指
23  right_thumb     右拇指
24  head            头部
25  left_middle     左中指
26  right_middle    右中指
27  left_bigtoe     左大脚趾
28  right_bigtoe    右大脚趾
'''

'''
重映射12点定义
输出序号  HybrIK序号  English          中文
0        1          left_hip         左髋
1        2          right_hip        右髋
2        4          left_knee        左膝
3        5          right_knee       右膝
4        7          left_ankle       左踝
5        8          right_ankle      右踝
6        16         left_shoulder    左肩
7        17         right_shoulder   右肩
8        18         left_elbow       左肘
9        19         right_elbow      右肘
10       20         left_wrist       左腕
11       21         right_wrist      右腕
'''

import numpy as np

KEYPOINT_NAMES = (
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "root",
)

class Point3D():
    def __init__(self, x=None, y=None, z=None):
        self.x = x
        self.y = y
        self.z = z

    def as_array(self):
        return np.array([self.x, self.y, self.z, 1])

class HumanKeypoints():
    def __init__(self, msg = None):
        self.left_hip = Point3D()
        self.right_hip = Point3D()
        self.left_knee = Point3D()
        self.right_knee = Point3D()
        self.left_ankle = Point3D()
        self.right_ankle = Point3D()
        self.left_shoulder = Point3D()
        self.right_shoulder = Point3D()
        self.left_elbow = Point3D()
        self.right_elbow = Point3D()
        self.left_wrist = Point3D()
        self.right_wrist = Point3D()

        self.root = Point3D()

        if msg is not None:
            self.joint_msg2HumanKeypoints(msg)

    def joint_msg2HumanKeypoints(self, msg):
        joints = msg.joints

        self.left_hip = self.joint2Point3D(joints[0])
        self.right_hip = self.joint2Point3D(joints[1])
        self.left_knee = self.joint2Point3D(joints[2])
        self.right_knee = self.joint2Point3D(joints[3])
        self.left_ankle = self.joint2Point3D(joints[4])
        self.right_ankle = self.joint2Point3D(joints[5])
        self.left_shoulder = self.joint2Point3D(joints[6])
        self.right_shoulder = self.joint2Point3D(joints[7])
        self.left_elbow = self.joint2Point3D(joints[8])
        self.right_elbow = self.joint2Point3D(joints[9])
        self.left_wrist = self.joint2Point3D(joints[10])
        self.right_wrist = self.joint2Point3D(joints[11])

        self.get_root()

    def joint2Point3D(self, joint):
        return Point3D(joint.x, joint.y, joint.z)

    def get_root(self):
        self.root.x = (self.left_hip.x + self.right_hip.x) / 2
        self.root.y = (self.left_hip.y + self.right_hip.y) / 2
        self.root.z = (self.left_hip.z + self.right_hip.z) / 2

    def as_list(self):
        return [getattr(self, name) for name in KEYPOINT_NAMES]

    def as_dict(self):
        return {name: getattr(self, name) for name in KEYPOINT_NAMES}

class HumanVariables():
    def __init__(self):
        #角度制记录
        self.body_yaw = None

        self.left_shoulder_yaw = None
        self.left_shoulder_roll = None
        self.left_shoulder_pitch = None
        self.left_elbow_flexion = None

        self.right_shoulder_yaw = None
        self.right_shoulder_roll = None
        self.right_shoulder_pitch = None
        self.right_elbow_flexion = None

        self.left_hip_yaw = None
        self.left_hip_roll = None
        self.left_hip_pitch = None
        self.left_knee_flexion = None

        self.right_hip_yaw = None
        self.right_hip_roll = None
        self.right_hip_pitch = None
        self.right_knee_flexion = None

class HumanLength():
    def __init__(self, points: HumanKeypoints = None):
        self.left_upper_arm = None
        self.right_upper_arm = None
        self.left_forearm = None
        self.right_forearm = None

        self.left_thigh = None
        self.right_thigh = None
        self.left_shin = None
        self.right_shin = None

        self.hip_width = None
        self.shoulder_width = None
        self.left_hip_shoulder = None
        self.right_hip_shoulder = None

        if points is not None:
            self.point2length(points)

    def point2length(self, points: HumanKeypoints):
        self.left_upper_arm = np.linalg.norm(
            points.left_shoulder.as_array() -
            points.left_elbow.as_array()
        )
        self.right_upper_arm = np.linalg.norm(
            points.right_shoulder.as_array() -
            points.right_elbow.as_array()
        )
        self.left_forearm = np.linalg.norm(
            points.left_elbow.as_array() -
            points.left_wrist.as_array()
        )
        self.right_forearm = np.linalg.norm(
            points.right_elbow.as_array() -
            points.right_wrist.as_array()
        )

        self.left_thigh = np.linalg.norm(
            points.left_hip.as_array() -
            points.left_knee.as_array()
        )
        self.right_thigh = np.linalg.norm(
            points.right_hip.as_array() -
            points.right_knee.as_array()
        )
        self.left_shin = np.linalg.norm(
            points.left_knee.as_array() -
            points.left_ankle.as_array()
        )
        self.right_shin = np.linalg.norm(
            points.right_knee.as_array() -
            points.right_ankle.as_array()
        )

        self.hip_width = np.linalg.norm(
            points.left_hip.as_array() -
            points.right_hip.as_array()
        )
        self.shoulder_width = np.linalg.norm(
            points.left_shoulder.as_array() -
            points.right_shoulder.as_array()
        )
        self.left_hip_shoulder = np.linalg.norm(
            points.left_hip.as_array() -
            points.left_shoulder.as_array()
        )
        self.right_hip_shoulder = np.linalg.norm(
            points.right_hip.as_array() -
            points.right_shoulder.as_array()
        )

