import numpy as np

from .variables import HumanVariables, HumanKeypoints
from .human_model import HumanModel

VARIABLE_NAMES = (
    "body_yaw",
    "left_shoulder_yaw",
    "left_shoulder_roll",
    "left_shoulder_pitch",
    "left_elbow_flexion",
    "right_shoulder_yaw",
    "right_shoulder_roll",
    "right_shoulder_pitch",
    "right_elbow_flexion",
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee_flexion",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee_flexion",
)

LOSS_KEYPOINT_NAMES = (
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

ANGLE_BOUNDS = {
    "body_yaw": (-45.0, 45.0),
    "left_shoulder_yaw": (-90.0, 90.0),
    "left_shoulder_roll": (-90.0, 90.0),
    "left_shoulder_pitch": (-90.0, 90.0),
    "left_elbow_flexion": (-150.0, 150.0),
    "right_shoulder_yaw": (-90.0, 90.0),
    "right_shoulder_roll": (-90.0, 90.0),
    "right_shoulder_pitch": (-90.0, 90.0),
    "right_elbow_flexion": (-150.0, 150.0),
    "left_hip_yaw": (-45.0, 45.0),
    "left_hip_roll": (-45.0, 45.0),
    "left_hip_pitch": (-90.0, 90.0),
    "left_knee_flexion": (-150.0, 150.0),
    "right_hip_yaw": (-45.0, 45.0),
    "right_hip_roll": (-45.0, 45.0),
    "right_hip_pitch": (-90.0, 90.0),
    "right_knee_flexion": (-150.0, 150.0),
}

_LOWER_BOUNDS = np.array(
    [ANGLE_BOUNDS[name][0] for name in VARIABLE_NAMES],
    dtype=float,
)
_UPPER_BOUNDS = np.array(
    [ANGLE_BOUNDS[name][1] for name in VARIABLE_NAMES],
    dtype=float,
)


def _variables_to_array(var):
    values = []
    for name in VARIABLE_NAMES:
        value = getattr(var, name, None)
        if value is None:
            value = 0.0
        values.append(float(value))
    return np.array(values, dtype=float)


def _array_to_variables(values):
    var = HumanVariables()
    for name, value in zip(VARIABLE_NAMES, values):
        setattr(var, name, float(value))
    return var


def _clip_to_bounds(values):
    return np.clip(values, _LOWER_BOUNDS, _UPPER_BOUNDS)


def _point_xyz(point):
    return np.array([point.x, point.y, point.z], dtype=float)


class UDEAS():
    def __init__(
            self,
            human_length,
            iteration_count=4,
            candidates_per_iteration=64,
            shrink_rate=0.5,
            seed=None,
    ):
        self.human_length = human_length
        self.model = HumanModel(human_length)
        self.iteration_count = iteration_count
        self.candidates_per_iteration = candidates_per_iteration
        self.shrink_rate = shrink_rate
        self.rng = np.random.default_rng(seed)

    def loss(self, candidate_variables, observed_msg):
        predicted = self.model.forward_kinematics(
            candidate_variables,
            observed_msg,
        )
        observed = HumanKeypoints(observed_msg)

        distances = []
        for name in LOSS_KEYPOINT_NAMES:
            predicted_point = getattr(predicted, name)
            observed_point = getattr(observed, name)
            distance = np.linalg.norm(
                _point_xyz(predicted_point) -
                _point_xyz(observed_point)
            )
            distances.append(distance)

        return float(np.mean(distances))

    def fit(self, observed_msg, initial_variables=None):
        if initial_variables is None:
            center = np.zeros(len(VARIABLE_NAMES), dtype=float)
        else:
            center = _variables_to_array(initial_variables)

        center = _clip_to_bounds(center)
        best_values = center.copy()
        best_loss = self.loss(_array_to_variables(best_values), observed_msg)
        radius = (_UPPER_BOUNDS - _LOWER_BOUNDS) / 2.0

        for _ in range(self.iteration_count):
            for _ in range(self.candidates_per_iteration):
                noise = self.rng.uniform(-radius, radius)
                candidate_values = _clip_to_bounds(center + noise)
                candidate_variables = _array_to_variables(candidate_values)
                candidate_loss = self.loss(candidate_variables, observed_msg)

                if candidate_loss < best_loss:
                    best_loss = candidate_loss
                    best_values = candidate_values

            center = best_values.copy()
            radius = radius * self.shrink_rate

        return _array_to_variables(best_values), best_loss


def fit_udeas(observed_msg, human_length, initial_variables=None):
    udeas = UDEAS(human_length)
    return udeas.fit(observed_msg, initial_variables)
