from dataclasses import dataclass
from math import radians


DEFAULT_MAX_STEP_RAD = 0.2


@dataclass(frozen=True)
class JointMapping:
    human_name: str
    robot_name: str
    lower: float
    upper: float
    sign: float = 1.0
    scale: float = 1.0
    offset: float = 0.0


G1_JOINT_MAPPINGS = (
    JointMapping("body_yaw", "waist_yaw_joint", -2.618, 2.618),

    JointMapping(
        "left_hip_yaw",
        "left_hip_yaw_joint",
        -2.7576,
        2.7576,
    ),
    JointMapping(
        "left_hip_roll",
        "left_hip_roll_joint",
        -0.5236,
        2.9671,
    ),
    JointMapping(
        "left_hip_pitch",
        "left_hip_pitch_joint",
        -2.5307,
        2.8798,
    ),
    JointMapping(
        "left_knee_flexion",
        "left_knee_joint",
        -0.087267,
        2.8798,
    ),

    JointMapping(
        "right_hip_yaw",
        "right_hip_yaw_joint",
        -2.7576,
        2.7576,
    ),
    JointMapping(
        "right_hip_roll",
        "right_hip_roll_joint",
        -2.9671,
        0.5236,
    ),
    JointMapping(
        "right_hip_pitch",
        "right_hip_pitch_joint",
        -2.5307,
        2.8798,
    ),
    JointMapping(
        "right_knee_flexion",
        "right_knee_joint",
        -0.087267,
        2.8798,
    ),

    JointMapping(
        "left_shoulder_yaw",
        "left_shoulder_yaw_joint",
        -2.618,
        2.618,
    ),
    JointMapping(
        "left_shoulder_roll",
        "left_shoulder_roll_joint",
        -1.5882,
        2.2515,
    ),
    JointMapping(
        "left_shoulder_pitch",
        "left_shoulder_pitch_joint",
        -3.0892,
        2.6704,
    ),
    JointMapping(
        "left_elbow_flexion",
        "left_elbow_joint",
        -1.0472,
        2.0944,
    ),

    JointMapping(
        "right_shoulder_yaw",
        "right_shoulder_yaw_joint",
        -2.618,
        2.618,
    ),
    JointMapping(
        "right_shoulder_roll",
        "right_shoulder_roll_joint",
        -2.2515,
        1.5882,
    ),
    JointMapping(
        "right_shoulder_pitch",
        "right_shoulder_pitch_joint",
        -3.0892,
        2.6704,
    ),
    JointMapping(
        "right_elbow_flexion",
        "right_elbow_joint",
        -1.0472,
        2.0944,
    ),
)


class G1Adapter():
    def __init__(
            self,
            max_step_rad=DEFAULT_MAX_STEP_RAD,
            joint_mappings=G1_JOINT_MAPPINGS,
    ):
        self.max_step_rad = max(0.0, float(max_step_rad))
        self.joint_mappings = joint_mappings
        self.last_positions = {}

    def adapt(self, human_angles):
        names = []
        positions = []
        next_positions = {}

        for mapping in self.joint_mappings:
            target = self.get_target_position(mapping, human_angles)
            target = self.clamp(target, mapping.lower, mapping.upper)
            target = self.limit_step(mapping.robot_name, target)
            target = self.clamp(target, mapping.lower, mapping.upper)

            names.append(mapping.robot_name)
            positions.append(target)
            next_positions[mapping.robot_name] = target

        self.last_positions = next_positions
        return names, positions

    def get_target_position(self, mapping, human_angles):
        if mapping.human_name not in human_angles:
            return self.last_positions.get(mapping.robot_name, 0.0)

        human_deg = float(human_angles[mapping.human_name])
        human_rad = radians(human_deg)
        return mapping.offset + mapping.sign * mapping.scale * human_rad

    def limit_step(self, robot_name, target):
        previous = self.last_positions.get(robot_name, 0.0)
        diff = target - previous

        if diff > self.max_step_rad:
            return previous + self.max_step_rad
        if diff < -self.max_step_rad:
            return previous - self.max_step_rad
        return target

    def clamp(self, value, lower, upper):
        return min(max(value, lower), upper)
