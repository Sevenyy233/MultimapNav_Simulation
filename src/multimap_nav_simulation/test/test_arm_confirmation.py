"""Startup must use complete, fresh, stopped measured arm feedback."""
import importlib.util
from pathlib import Path

from sensor_msgs.msg import JointState

SPEC = importlib.util.spec_from_file_location(
    'clearing_arm_startup', Path(__file__).parents[1]/'scripts/initialize_clearing_arm.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def feedback():
    message = JointState()
    message.header.stamp.sec = 10
    message.name = [f'joint{i}' for i in range(1, 8)]
    message.position = [0.0]*7
    message.velocity = [0.0]*7
    return message


def test_stowed_fresh_and_stopped():
    assert MODULE.arm_is_stowed(feedback(), [0.0]*7, 10.1)


def test_stale_or_missing_feedback_is_rejected():
    assert not MODULE.arm_is_stowed(None, [0.0]*7, 10.1)
    assert not MODULE.arm_is_stowed(feedback(), [0.0]*7, 11.0)
    message = feedback()
    message.name.pop()
    assert not MODULE.arm_is_stowed(message, [0.0]*7, 10.1)


def test_moving_or_wrong_pose_is_rejected():
    message = feedback()
    message.velocity[2] = .05
    assert not MODULE.arm_is_stowed(message, [0.0]*7, 10.1)
    message = feedback()
    message.position[2] = .05
    assert not MODULE.arm_is_stowed(message, [0.0]*7, 10.1)
