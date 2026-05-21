from angle_processor.processor import AngleProcessor


class FakeAngle():
    def __init__(self, name, angle):
        self.name = name
        self.angle = angle


class FakeAnglesMsg():
    def __init__(self, values):
        self.header = 'header'
        self.angles = [
            FakeAngle(name, angle)
            for name, angle in values
        ]


def angle_values(msg):
    return {
        angle.name: angle.angle
        for angle in msg.angles
    }


def test_first_frame_passes_through():
    processor = AngleProcessor()

    output = processor.process(FakeAnglesMsg([('left_elbow_flexion', 12.0)]))

    assert output.header == 'header'
    assert angle_values(output) == {'left_elbow_flexion': 12.0}


def test_max_jump_clamps_angle_change():
    processor = AngleProcessor(max_jump_deg=20.0, moving_average_window=1)

    processor.process(FakeAnglesMsg([('left_elbow_flexion', 0.0)]))
    output = processor.process(FakeAnglesMsg([('left_elbow_flexion', 100.0)]))

    assert angle_values(output) == {'left_elbow_flexion': 20.0}


def test_moving_average_uses_recent_clamped_angles():
    processor = AngleProcessor(max_jump_deg=20.0, moving_average_window=5)

    output = None
    for value in (0.0, 10.0, 20.0, 30.0, 40.0):
        output = processor.process(
            FakeAnglesMsg([('left_elbow_flexion', value)])
        )

    assert angle_values(output) == {'left_elbow_flexion': 20.0}


def test_angle_names_are_processed_independently():
    processor = AngleProcessor(max_jump_deg=20.0, moving_average_window=1)

    processor.process(FakeAnglesMsg([
        ('left_elbow_flexion', 0.0),
        ('right_elbow_flexion', 50.0),
    ]))
    output = processor.process(FakeAnglesMsg([
        ('left_elbow_flexion', 100.0),
        ('right_elbow_flexion', -50.0),
    ]))

    assert angle_values(output) == {
        'left_elbow_flexion': 20.0,
        'right_elbow_flexion': 30.0,
    }
