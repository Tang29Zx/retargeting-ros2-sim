import numpy as np
from .variables import HumanVariables, HumanKeypoints, HumanLength, Point3D
from math import sin, cos, radians

'''
以竖直向上为初始z轴
人的朝向为初始x轴
位移变换矩阵以x轴为正方向
旋转变换矩阵以 pitch(y) @ roll(x) @ yaw(z) 为标准顺序 
'''

class HumanTrans():
    def __init__(
            self, 
            human_variables: HumanVariables, 
            human_length: HumanLength, 
            processed_joints_msg
    ):
        self.var = human_variables
        self.raw = HumanKeypoints(processed_joints_msg)

        #待优化
        self.T_body_yaw = self.rotz(self.var.body_yaw)

        self.T_left_shoulder_yaw = self.rotz(self.var.left_shoulder_yaw)
        self.T_left_shoulder_roll = self.rotx(self.var.left_shoulder_roll)
        self.T_left_shoulder_pitch = self.roty(self.var.left_shoulder_pitch)
        self.T_left_elbow_flexion = self.rotz(self.var.left_elbow_flexion)

        self.T_right_shoulder_yaw = self.rotz(self.var.right_shoulder_yaw)
        self.T_right_shoulder_roll = self.rotx(self.var.right_shoulder_roll)
        self.T_right_shoulder_pitch = self.roty(self.var.right_shoulder_pitch)
        self.T_right_elbow_flexion = self.rotz(self.var.right_elbow_flexion)

        self.T_left_hip_yaw = self.rotz(self.var.left_hip_yaw)
        self.T_left_hip_roll = self.rotx(self.var.left_hip_roll)
        self.T_left_hip_pitch = self.roty(self.var.left_hip_pitch)
        self.T_left_knee_flexion = self.rotz(self.var.left_knee_flexion)

        self.T_right_hip_yaw = self.rotz(self.var.right_hip_yaw)
        self.T_right_hip_roll = self.rotx(self.var.right_hip_roll)
        self.T_right_hip_pitch = self.roty(self.var.right_hip_pitch)
        self.T_right_knee_flexion = self.rotz(self.var.right_knee_flexion)

        #已知
        self.T_root = self.get_T_root(self.raw, self.var)
        self.T_left_shoulder_offset = self.get_T_left_shoulder_offset(self.raw)
        self.T_right_shoulder_offset = self.get_T_right_shoulder_offset(self.raw)
        self.T_left_hip_offset = self.get_T_left_hip_offset(self.raw)
        self.T_right_hip_offset = self.get_T_right_hip_offset(self.raw)

        self.T_left_upper_arm_offset = self.get_T_offset(human_length.left_upper_arm, 0, 0)
        self.T_right_upper_arm_offset = self.get_T_offset(human_length.right_upper_arm, 0, 0)
        self.T_left_forearm_offset = self.get_T_offset(human_length.left_forearm, 0, 0)
        self.T_right_forearm_offset = self.get_T_offset(human_length.right_forearm, 0, 0)

        self.T_left_thigh_offset = self.get_T_offset(human_length.left_thigh, 0, 0)
        self.T_right_thigh_offset = self.get_T_offset(human_length.right_thigh, 0, 0)
        self.T_left_shin_offset = self.get_T_offset(human_length.left_shin, 0, 0)
        self.T_right_shin_offset = self.get_T_offset(human_length.right_shin, 0, 0)

    def angle_or_zero(self, angle):
        if angle is None:
            return 0
        return angle

    def rotx(self, angle):
        rad = radians(self.angle_or_zero(angle))
        c = cos(rad)
        s = sin(rad)
        return np.array([
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1],
        ])

    def roty(self, angle):
        rad = radians(self.angle_or_zero(angle))
        c = cos(rad)
        s = sin(rad)
        return np.array([
            [c, 0, s, 0],
            [0, 1, 0, 0],
            [-s, 0, c, 0],
            [0, 0, 0, 1],
        ])

    def rotz(self, angle):
        rad = radians(self.angle_or_zero(angle))
        c = cos(rad)
        s = sin(rad)
        return np.array([
            [c, -s, 0, 0],
            [s, c, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
    
    def get_T_root(self, raw: HumanKeypoints, var: HumanVariables):
        dx = raw.root.x
        dy = raw.root.y
        dz = raw.root.z

        rad = radians(self.angle_or_zero(var.body_yaw))
        c = cos(rad)
        s = sin(rad)
        return np.array([
            [c, -s, 0, dx],
            [s, c, 0, dy],
            [0, 0, 1, dz],
            [0, 0, 0, 1],
        ])

    def get_T_left_shoulder_offset(self, raw: HumanKeypoints):
        return self.get_T_root_offset(raw.left_shoulder, raw)

    def get_T_right_shoulder_offset(self, raw: HumanKeypoints):
        return self.get_T_root_offset(raw.right_shoulder, raw)

    def get_T_left_hip_offset(self, raw: HumanKeypoints):
        return self.get_T_root_offset(raw.left_hip, raw)

    def get_T_right_hip_offset(self, raw: HumanKeypoints):
        return self.get_T_root_offset(raw.right_hip, raw)

    def get_T_root_offset(self, point: Point3D, raw: HumanKeypoints):
        offset_world = np.array([
            point.x - raw.root.x,
            point.y - raw.root.y,
            point.z - raw.root.z,
            0,
        ])
        offset_local = self.rotz(-self.angle_or_zero(self.var.body_yaw)) @ offset_world
        return self.get_T_offset(offset_local[0], offset_local[1], offset_local[2])

    def get_T_offset(self, dx, dy, dz):
        return np.array([
            [1, 0, 0, dx],
            [0, 1, 0, dy],
            [0, 0, 1, dz],
            [0, 0, 0, 1],
        ])
        
class HumanModel():
    def __init__(
            self, 
            human_length: HumanLength, 
    ):
        self.human_length = human_length

    def array2Point3D(self, point):
        return Point3D(point[0], point[1], point[2])

    def forward_kinematics(
            self, 
            var: HumanVariables, 
            processed_joints_msg
    ):
        '''
        输入HamanVariables
        输出FK点坐标
        '''
        #计算矩阵
        human_trans = HumanTrans(var, self.human_length, processed_joints_msg)
        #当前点值（输出）
        now = HumanKeypoints(processed_joints_msg)
        origin = np.array([0, 0, 0, 1])

        now.root = self.array2Point3D(
            human_trans.T_root @
            origin
        )

        #FK
        now.left_shoulder = self.array2Point3D(
            human_trans.T_root @
            human_trans.T_left_shoulder_offset @
            origin
        )
        now.left_elbow = self.array2Point3D(
            human_trans.T_root @
            human_trans.T_left_shoulder_offset @
            human_trans.T_left_shoulder_pitch @
            human_trans.T_left_shoulder_roll @
            human_trans.T_left_shoulder_yaw @
            human_trans.T_left_upper_arm_offset @
            origin
        )
        now.left_wrist = self.array2Point3D(
            human_trans.T_root @
            human_trans.T_left_shoulder_offset @
            human_trans.T_left_shoulder_pitch @
            human_trans.T_left_shoulder_roll @
            human_trans.T_left_shoulder_yaw @
            human_trans.T_left_upper_arm_offset @
            human_trans.T_left_elbow_flexion @
            human_trans.T_left_forearm_offset @
            origin
        )

        now.right_shoulder = self.array2Point3D(
            human_trans.T_root @
            human_trans.T_right_shoulder_offset @
            origin
        )
        now.right_elbow = self.array2Point3D(
            human_trans.T_root @
            human_trans.T_right_shoulder_offset @
            human_trans.T_right_shoulder_pitch @
            human_trans.T_right_shoulder_roll @
            human_trans.T_right_shoulder_yaw @
            human_trans.T_right_upper_arm_offset @
            origin
        )
        now.right_wrist = self.array2Point3D(
            human_trans.T_root @
            human_trans.T_right_shoulder_offset @
            human_trans.T_right_shoulder_pitch @
            human_trans.T_right_shoulder_roll @
            human_trans.T_right_shoulder_yaw @
            human_trans.T_right_upper_arm_offset @
            human_trans.T_right_elbow_flexion @
            human_trans.T_right_forearm_offset @
            origin
        )

        now.left_hip = self.array2Point3D(
            human_trans.T_root @
            human_trans.T_left_hip_offset @
            origin
        )
        now.left_knee = self.array2Point3D(
            human_trans.T_root @
            human_trans.T_left_hip_offset @
            human_trans.T_left_hip_pitch @
            human_trans.T_left_hip_roll @
            human_trans.T_left_hip_yaw @
            human_trans.T_left_thigh_offset @
            origin
        )
        now.left_ankle = self.array2Point3D(
            human_trans.T_root @
            human_trans.T_left_hip_offset @
            human_trans.T_left_hip_pitch @
            human_trans.T_left_hip_roll @
            human_trans.T_left_hip_yaw @
            human_trans.T_left_thigh_offset @
            human_trans.T_left_knee_flexion @
            human_trans.T_left_shin_offset @
            origin
        )

        now.right_hip = self.array2Point3D(
            human_trans.T_root @
            human_trans.T_right_hip_offset @
            origin
        )
        now.right_knee = self.array2Point3D(
            human_trans.T_root @
            human_trans.T_right_hip_offset @
            human_trans.T_right_hip_pitch @
            human_trans.T_right_hip_roll @
            human_trans.T_right_hip_yaw @
            human_trans.T_right_thigh_offset @
            origin
        )
        now.right_ankle = self.array2Point3D(
            human_trans.T_root @
            human_trans.T_right_hip_offset @
            human_trans.T_right_hip_pitch @
            human_trans.T_right_hip_roll @
            human_trans.T_right_hip_yaw @
            human_trans.T_right_thigh_offset @
            human_trans.T_right_knee_flexion @
            human_trans.T_right_shin_offset @
            origin
        )

        return now
