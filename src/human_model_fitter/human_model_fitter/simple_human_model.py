"""给 uDEAS 早期实验用的简化人体正运动学模型。

这个文件不是论文里的完整 Denavit-Hartenberg 人体模型，而是一个更小、
更容易读懂的版本。它的目标是先把核心数据流跑通：

    一组人体关节角 -> 正运动学 -> 12 个模型关键点

后面的 uDEAS 优化器会不断尝试不同的人体关节角，然后调用这里的
forward_kinematics() 生成一套模型关键点，再和 HybrIK 观测到的
/retarget_keypoints 做距离比较。距离越小，说明这组关节角越像当前人体姿态。

重要约定：

1. 所有角度都使用 radians，也就是弧度。
   ROS 控制和数学库里通常都用弧度；日志显示时再转成角度即可。

2. 输出的 12 个关键点顺序必须和 /retarget_keypoints 一模一样。
   这样 loss 函数才能逐点比较，不会把左肘拿去和右膝比较。

3. 这里的模型故意简化：
   - pelvis/root 是人体根节点。
   - 左右髋相对 root 固定。
   - 左右肩相对 root 固定。
   - 胳膊和腿都是两段链条。
   - 每条链用 pitch/roll 控制第一段方向，用 flexion 控制第二段弯曲。

这个文件的作用是“教学 + 原型”，不是最终控制真实机器人的安全模型。
"""

from dataclasses import dataclass
from math import cos, sin, sqrt


# 这个顺序就是 /retarget_keypoints 的顺序。
# keypoint_validator_node.py 会从 HybrIK 29 点里抽出这 12 个点。
#
# 下游所有代码都应该遵守这个顺序：
# 0-5 是下半身，6-11 是上半身。
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
)


@dataclass(frozen=True)
class Vec3:
    """一个很小的三维向量类。

    为什么不用 numpy？
    这里先故意不用 numpy，让你能直接看懂每一步向量加减乘。
    之后如果要跑得更快，可以再换成 numpy 数组。

    frozen=True 表示这个对象创建后不能被修改。
    这样可以避免某个函数不小心原地改掉点坐标，调试时更稳。
    """

    x: float
    y: float
    z: float

    def __add__(self, other):
        """向量加法：两个点/向量逐坐标相加。"""
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        """向量减法：常用于算两个点之间的差向量。"""
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scale):
        """向量乘标量：常用于 direction * bone_length。"""
        return Vec3(self.x * scale, self.y * scale, self.z * scale)


@dataclass(frozen=True)
class BodyDimensions:
    """人体的固定骨长/身体尺寸。

    uDEAS 搜的是关节角，不应该让骨长每一帧乱跳。
    所以一个合理做法是：

    1. 第一帧或校准姿态中，从 /retarget_keypoints 估计这些长度。
    2. 后续帧保持这些长度固定。
    3. 只优化关节角和少量整体姿态变量。

    默认值只是一个近似人体比例，单位跟 /retarget_keypoints 的坐标单位一致。
    在你的当前工程中，/retarget_keypoints 来自 /hybrik_pose_world，所以这些
    长度可以理解为经过 processor_node.py scale/anchor 后的世界坐标长度。
    """

    # 左肩到右肩的距离。
    shoulder_width: float = 0.36

    # 左髋到右髋的距离。
    hip_width: float = 0.28

    # 髋中心到肩中心的距离。这里简化成竖直躯干长度。
    torso_length: float = 0.55

    # 肩到肘的距离。
    upper_arm_length: float = 0.30

    # 肘到腕的距离。
    forearm_length: float = 0.27

    # 髋到膝的距离。
    thigh_length: float = 0.42

    # 膝到踝的距离。
    shin_length: float = 0.42


@dataclass(frozen=True)
class HumanVariables:
    """这个简化模型当前支持的“待优化人体变量”。

    这些变量就是 uDEAS 后面要搜索的 V。
    论文完整模型有更多变量，例如 gamma、body pitch/roll/yaw、腰 pitch/yaw、
    肩 yaw 等。这里先保留最容易跑通的一组。

    命名约定：

    body_yaw:
      整个人体绕世界 z 轴转多少。它用于表达人体整体朝向。

    shoulder_pitch / hip_pitch:
      控制上臂/大腿向前后方向摆动。

    shoulder_roll / hip_roll:
      控制上臂/大腿向左右方向打开或收回。

    elbow_flexion / knee_flexion:
      控制肘/膝弯曲。这里是简化版本，只把 flexion 加到第二段 limb 的 pitch 上。

    注意：
    这里没有强制关节限位。真正使用 uDEAS 时，范围应该放在 variables.py 里管。
    """

    body_yaw: float = 0.0
    left_shoulder_pitch: float = 0.0
    right_shoulder_pitch: float = 0.0
    left_shoulder_roll: float = 0.0
    right_shoulder_roll: float = 0.0
    left_elbow_flexion: float = 0.0
    right_elbow_flexion: float = 0.0
    left_hip_pitch: float = 0.0
    right_hip_pitch: float = 0.0
    left_hip_roll: float = 0.0
    right_hip_roll: float = 0.0
    left_knee_flexion: float = 0.0
    right_knee_flexion: float = 0.0


def distance(a, b):
    """计算两个 3D 点之间的欧氏距离。

    loss/MPJPE 会大量用到这个函数：

        distance(model_point, observed_point)

    距离越小，说明模型点越贴近 HybrIK 观测点。
    """

    delta = a - b
    return sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z)


def normalize(v):
    """把向量变成单位向量。

    正运动学里经常先算一个方向 direction，然后乘以骨长。
    为了保证骨长不被方向向量长度污染，需要先 normalize。
    """

    length = sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    if length == 0.0:
        # 极端保护：如果输入是零向量，就默认指向 -z，也就是“向下”。
        return Vec3(0.0, 0.0, -1.0)
    return Vec3(v.x / length, v.y / length, v.z / length)


def rotate_x(v, angle):
    """绕 x 轴旋转一个向量。

    这里用右手系旋转矩阵：

        x' = x
        y' = cos(a) * y - sin(a) * z
        z' = sin(a) * y + cos(a) * z

    在这个简化模型里，pitch 暂时通过 rotate_x 表达。
    后面如果你想严格对应机器人/人体坐标系，可以在这里统一调整约定。
    """

    c = cos(angle)
    s = sin(angle)
    return Vec3(v.x, c * v.y - s * v.z, s * v.y + c * v.z)


def rotate_y(v, angle):
    """绕 y 轴旋转一个向量。

    这里用来表达 roll 的效果：让原本向下的 limb 产生左右偏移。
    这个轴约定是为了原型简单，不是最终 G1 关节方向约定。
    """

    c = cos(angle)
    s = sin(angle)
    return Vec3(c * v.x + s * v.z, v.y, -s * v.x + c * v.z)


def rotate_z(v, angle):
    """绕 z 轴旋转一个向量。

    z 轴在当前 world 坐标里可以理解为竖直方向。
    body_yaw 就是用 rotate_z 作用到肩/髋偏移和 limb 方向上。
    """

    c = cos(angle)
    s = sin(angle)
    return Vec3(c * v.x - s * v.y, s * v.x + c * v.y, v.z)


def limb_direction(body_yaw, pitch, roll):
    """根据 body_yaw + pitch + roll 算出一段 limb 的方向。

    默认 limb 方向是 Vec3(0, 0, -1)，意思是从关节向下指。
    例如：

        肩 -> 肘：上臂默认自然下垂。
        髋 -> 膝：大腿默认向下。

    然后依次施加：

    1. pitch: 让 limb 前后摆。
    2. roll: 让 limb 左右摆。
    3. body_yaw: 把整个身体方向转到世界坐标中。

    这个顺序是简化约定，目的是让第一版模型容易调试。
    """

    direction = Vec3(0.0, 0.0, -1.0)
    direction = rotate_x(direction, pitch)
    direction = rotate_y(direction, roll)
    direction = rotate_z(direction, body_yaw)
    return normalize(direction)


def estimate_dimensions(keypoints):
    """从一帧观测到的 12 个关键点估计人体骨长。

    输入可以是：

    1. dict:
       {"left_hip": Vec3(...), ...}

    2. list/tuple:
       [left_hip, right_hip, ..., right_wrist]
       顺序必须和 KEYPOINT_NAMES 一样。

    为什么要估计骨长？
    因为不同人的上臂、大腿、肩宽都不一样。uDEAS 应该主要搜索“角度”，
    不应该每一帧连骨长都一起乱变。比较稳的做法是开头估一次骨长。
    """

    points = _coerce_keypoints(keypoints)

    return BodyDimensions(
        # 肩宽：左肩到右肩的距离。
        shoulder_width=distance(points["left_shoulder"], points["right_shoulder"]),
        # 髋宽：左髋到右髋的距离。
        hip_width=distance(points["left_hip"], points["right_hip"]),
        # 躯干长：髋中心到肩中心的距离。
        torso_length=distance(
            midpoint(points["left_hip"], points["right_hip"]),
            midpoint(points["left_shoulder"], points["right_shoulder"]),
        ),
        # 上臂长：左右肩肘距离取平均，降低单侧噪声影响。
        upper_arm_length=0.5
        * (
            distance(points["left_shoulder"], points["left_elbow"])
            + distance(points["right_shoulder"], points["right_elbow"])
        ),
        # 前臂长：左右肘腕距离取平均。
        forearm_length=0.5
        * (
            distance(points["left_elbow"], points["left_wrist"])
            + distance(points["right_elbow"], points["right_wrist"])
        ),
        # 大腿长：左右髋膝距离取平均。
        thigh_length=0.5
        * (
            distance(points["left_hip"], points["left_knee"])
            + distance(points["right_hip"], points["right_knee"])
        ),
        # 小腿长：左右膝踝距离取平均。
        shin_length=0.5
        * (
            distance(points["left_knee"], points["left_ankle"])
            + distance(points["right_knee"], points["right_ankle"])
        ),
    )


def midpoint(a, b):
    """两个点的中点。

    常用于算：

        pelvis/root = left_hip 和 right_hip 的中点
        shoulder_center = left_shoulder 和 right_shoulder 的中点
    """

    return Vec3(
        0.5 * (a.x + b.x),
        0.5 * (a.y + b.y),
        0.5 * (a.z + b.z),
    )


class SimpleHumanModel:
    """以骨盆中心为 root 的最小人体运动学树。

    这个模型的树结构：

        root / pelvis center
        ├── left_hip  ── left_knee  ── left_ankle
        ├── right_hip ── right_knee ── right_ankle
        ├── left_shoulder  ── left_elbow  ── left_wrist
        └── right_shoulder ── right_elbow ── right_wrist

    真实人体当然更复杂：
    肩膀和髋关节有更多自由度，躯干不是一根硬杆，膝/肘旋转轴也不是这么简单。
    但这个简化模型已经足够给 uDEAS 做第一版 loss 实验。
    """

    def __init__(self, dimensions=None):
        """创建模型。

        dimensions:
          如果传入 BodyDimensions，就用传入的骨长。
          如果不传，就用默认近似人体尺寸。
        """

        self.dimensions = dimensions or BodyDimensions()

    def forward_kinematics(self, variables, root=Vec3(0.0, 0.0, 0.95)):
        """根据一组关节变量，生成 12 个模型关键点。

        这是本文件最重要的函数。

        输入：

        variables:
          HumanVariables，一组人体关节角。

        root:
          骨盆中心在世界坐标中的位置。默认是 (0, 0, 0.95)。
          你当前工程里 processor_node.py 已经给人体骨架加了 anchor，
          所以将来可以把 observed keypoints 的髋中心作为 root 传进来。

        输出：

        dict:
          key 是关键点名字，value 是 Vec3 坐标。

        注意：
        这里做的是正运动学 FK：

            角度 + 骨长 -> 点的位置

        uDEAS 会通过反复调用它来间接求“点的位置 -> 角度”。
        """

        dims = self.dimensions
        yaw = variables.body_yaw

        # 1. 先从 root 生成左右髋。
        #
        # root 是骨盆中心。
        # 左髋在 root 的 +x 方向半个 hip_width。
        # 右髋在 root 的 -x 方向半个 hip_width。
        #
        # 如果 body_yaw 不为 0，说明整个人体绕 z 轴转了，所以髋的横向偏移也要转。
        left_hip = root + rotate_z(Vec3(dims.hip_width * 0.5, 0.0, 0.0), yaw)
        right_hip = root + rotate_z(Vec3(-dims.hip_width * 0.5, 0.0, 0.0), yaw)

        # 2. 从 root 往上找到肩中心，再生成左右肩。
        #
        # 这里把躯干简化成一根竖直杆：
        #
        #     shoulder_center = root + (0, 0, torso_length)
        #
        # 暂时没有 torso pitch/roll/yaw。后面要更像论文，可以给躯干也加角度。
        shoulder_center = root + Vec3(0.0, 0.0, dims.torso_length)
        left_shoulder = shoulder_center + rotate_z(
            Vec3(dims.shoulder_width * 0.5, 0.0, 0.0), yaw
        )
        right_shoulder = shoulder_center + rotate_z(
            Vec3(-dims.shoulder_width * 0.5, 0.0, 0.0), yaw
        )

        # 3. 生成左腿和右腿。
        #
        # 每条腿是两段：
        #   hip -> knee: 大腿
        #   knee -> ankle: 小腿
        #
        # hip_pitch/hip_roll 控制大腿方向。
        # knee_flexion 在这个简化模型里加到小腿 pitch 上。
        left_knee, left_ankle = self._leg_chain(
            left_hip,
            yaw,
            variables.left_hip_pitch,
            variables.left_hip_roll,
            variables.left_knee_flexion,
        )
        right_knee, right_ankle = self._leg_chain(
            right_hip,
            yaw,
            variables.right_hip_pitch,
            variables.right_hip_roll,
            variables.right_knee_flexion,
        )

        # 4. 生成左臂和右臂。
        #
        # 每条胳膊也是两段：
        #   shoulder -> elbow: 上臂
        #   elbow -> wrist: 前臂
        #
        # shoulder_pitch/shoulder_roll 控制上臂方向。
        # elbow_flexion 在这个简化模型里加到前臂 pitch 上。
        left_elbow, left_wrist = self._arm_chain(
            left_shoulder,
            yaw,
            variables.left_shoulder_pitch,
            variables.left_shoulder_roll,
            variables.left_elbow_flexion,
        )
        right_elbow, right_wrist = self._arm_chain(
            right_shoulder,
            yaw,
            variables.right_shoulder_pitch,
            variables.right_shoulder_roll,
            variables.right_elbow_flexion,
        )

        # 5. 用名字返回，方便 loss 函数按语义比较。
        return {
            "left_hip": left_hip,
            "right_hip": right_hip,
            "left_knee": left_knee,
            "right_knee": right_knee,
            "left_ankle": left_ankle,
            "right_ankle": right_ankle,
            "left_shoulder": left_shoulder,
            "right_shoulder": right_shoulder,
            "left_elbow": left_elbow,
            "right_elbow": right_elbow,
            "left_wrist": left_wrist,
            "right_wrist": right_wrist,
        }

    def forward_kinematics_list(self, variables, root=Vec3(0.0, 0.0, 0.95)):
        """返回 list 形式的 12 点，顺序严格匹配 /retarget_keypoints。

        fitter_node.py 从 ROS 消息里拿到的是 Joints3D.joints 数组。
        那种情况下 list 形式更方便逐项比较或重新发布。
        """

        points = self.forward_kinematics(variables, root)
        return [points[name] for name in KEYPOINT_NAMES]

    def mpjpe(self, variables, observed_keypoints, root=Vec3(0.0, 0.0, 0.95)):
        """计算当前变量生成的模型点和观测点之间的 MPJPE。

        MPJPE = Mean Per Joint Position Error
        中文可以叫“平均每关节位置误差”。

        对 12 个点分别算欧氏距离，再取平均：

            loss = mean(distance(model_point_i, observed_point_i))

        uDEAS 优化时会最小化这个 loss。
        """

        observed = _coerce_keypoints(observed_keypoints)
        model_points = self.forward_kinematics(variables, root)
        total = 0.0
        for name in KEYPOINT_NAMES:
            total += distance(model_points[name], observed[name])
        return total / len(KEYPOINT_NAMES)

    def _arm_chain(self, shoulder, yaw, shoulder_pitch, shoulder_roll, elbow_flexion):
        """从肩点生成肘点和腕点。

        输入：

        shoulder:
          当前肩关节世界坐标。

        yaw:
          整体身体朝向。

        shoulder_pitch / shoulder_roll:
          控制上臂方向。

        elbow_flexion:
          控制前臂相对上臂继续弯曲。

        简化逻辑：

        1. 上臂方向 = limb_direction(yaw, shoulder_pitch, shoulder_roll)
        2. 肘点 = 肩点 + 上臂方向 * 上臂长度
        3. 前臂方向 = limb_direction(yaw, shoulder_pitch + elbow_flexion, shoulder_roll)
        4. 腕点 = 肘点 + 前臂方向 * 前臂长度

        这不是严格的人体肘关节旋转轴，只是为了先让优化器有一个连续可调的模型。
        """

        dims = self.dimensions
        upper_dir = limb_direction(yaw, shoulder_pitch, shoulder_roll)
        elbow = shoulder + upper_dir * dims.upper_arm_length

        forearm_dir = limb_direction(
            yaw,
            shoulder_pitch + elbow_flexion,
            shoulder_roll,
        )
        wrist = elbow + forearm_dir * dims.forearm_length
        return elbow, wrist

    def _leg_chain(self, hip, yaw, hip_pitch, hip_roll, knee_flexion):
        """从髋点生成膝点和踝点。

        逻辑和 _arm_chain 很像：

        1. 大腿方向 = limb_direction(yaw, hip_pitch, hip_roll)
        2. 膝点 = 髋点 + 大腿方向 * 大腿长度
        3. 小腿方向 = limb_direction(yaw, hip_pitch + knee_flexion, hip_roll)
        4. 踝点 = 膝点 + 小腿方向 * 小腿长度

        后面如果要更像真实腿部，可以把膝盖 flexion 的方向改成相对大腿局部坐标旋转。
        """

        dims = self.dimensions
        thigh_dir = limb_direction(yaw, hip_pitch, hip_roll)
        knee = hip + thigh_dir * dims.thigh_length

        shin_dir = limb_direction(yaw, hip_pitch + knee_flexion, hip_roll)
        ankle = knee + shin_dir * dims.shin_length
        return knee, ankle


def _coerce_keypoints(keypoints):
    """把不同输入形式统一成 dict[name] = Vec3。

    为了方便测试和接 ROS，这里允许两种输入：

    1. dict:
       {"left_hip": Vec3(...), ...}

    2. list/tuple:
       [Vec3(...), Vec3(...), ...]
       顺序必须等于 KEYPOINT_NAMES。

    如果 list 长度不是 12，直接报错。这样能早点发现 topic 顺序或数量问题。
    """

    if isinstance(keypoints, dict):
        return keypoints

    if len(keypoints) != len(KEYPOINT_NAMES):
        raise ValueError(
            f"Expected {len(KEYPOINT_NAMES)} keypoints, got {len(keypoints)}"
        )

    return {
        name: point if isinstance(point, Vec3) else Vec3(point[0], point[1], point[2])
        for name, point in zip(KEYPOINT_NAMES, keypoints)
    }
