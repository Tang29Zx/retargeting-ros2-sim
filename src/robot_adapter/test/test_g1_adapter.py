from math import isclose, pi

from robot_adapter.g1_adapter import G1Adapter


def position_for(names, positions, joint_name):
    return positions[names.index(joint_name)]


def test_degrees_are_converted_to_radians():
    adapter = G1Adapter(max_step_rad=10.0)

    names, positions = adapter.adapt({'left_elbow_flexion': 90.0})

    position = position_for(names, positions, 'left_elbow_joint')
    assert isclose(position, pi / 2.0)


def test_joint_limits_are_applied():
    adapter = G1Adapter(max_step_rad=10.0)

    names, positions = adapter.adapt({'left_elbow_flexion': 1000.0})

    position = position_for(names, positions, 'left_elbow_joint')
    assert isclose(position, 2.0944)


def test_max_step_limits_single_frame_change():
    adapter = G1Adapter(max_step_rad=0.2)

    names, positions = adapter.adapt({'left_elbow_flexion': 90.0})

    position = position_for(names, positions, 'left_elbow_joint')
    assert isclose(position, 0.2)


def test_missing_input_keeps_previous_target():
    adapter = G1Adapter(max_step_rad=10.0)

    adapter.adapt({'left_elbow_flexion': 30.0})
    names, positions = adapter.adapt({})

    position = position_for(names, positions, 'left_elbow_joint')
    assert isclose(position, pi / 6.0)
