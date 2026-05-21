from copy import deepcopy


DEFAULT_MAX_JUMP_DEG = 20.0
DEFAULT_MOVING_AVERAGE_WINDOW = 5


class AngleProcessor():
    def __init__(
            self,
            max_jump_deg=DEFAULT_MAX_JUMP_DEG,
            moving_average_window=DEFAULT_MOVING_AVERAGE_WINDOW,
    ):
        self.max_jump_deg = max(0.0, float(max_jump_deg))
        self.moving_average_window = max(1, int(moving_average_window))
        self.prev_angles = {}
        self.angle_history = []

    def process(self, msg):
        output_msg = deepcopy(msg)
        clamped_angles = {}

        for angle_msg in output_msg.angles:
            name = angle_msg.name
            raw_angle = float(angle_msg.angle)
            clamped_angle = self.clamp_angle(name, raw_angle)
            clamped_angles[name] = clamped_angle

        self.prev_angles.update(clamped_angles)
        self.angle_history.append(clamped_angles)
        if len(self.angle_history) > self.moving_average_window:
            self.angle_history.pop(0)

        for angle_msg in output_msg.angles:
            angle_msg.angle = float(self.average_angle(angle_msg.name))

        return output_msg

    def clamp_angle(self, name, raw_angle):
        if name not in self.prev_angles:
            return raw_angle

        prev_angle = self.prev_angles[name]
        diff = raw_angle - prev_angle
        if diff > self.max_jump_deg:
            return prev_angle + self.max_jump_deg
        if diff < -self.max_jump_deg:
            return prev_angle - self.max_jump_deg
        return raw_angle

    def average_angle(self, name):
        total = 0.0
        count = 0

        for angles in self.angle_history:
            if name not in angles:
                continue
            total += angles[name]
            count += 1

        if count == 0:
            return 0.0
        return total / count
