"""Hardware substitutes for tests only; never installed as a driver backend."""

import os

os.environ.setdefault('ROS_DOMAIN_ID', '97')

import pytest

from piper_x_driver.backend import Feedback, SdkArm, STATUS_FIELDS
from piper_x_driver.controller import Controller


class FakeArm(SdkArm):
    def __init__(self, channel='can0'):
        self._initialize_shutdown()
        self.channel = channel
        self.position = (0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.02)
        self.enabled = (False,) * 7
        self.status: dict[str, int] = dict(zip(STATUS_FIELDS, (0, 0, 1, 0, 0, 0, 0)))
        self.calls = []

    def snapshot(self):
        return Feedback(
            self.position, self.status.copy(), self.enabled, (0.0,) * 11,
            (0.2, 0.0, 0.2, 0.0, 0.0, 0.0), (0.0,) * 6, (0.0,) * 6,
        )

    def _write_power(self, enabled, width=0.0, *_):
        self.calls.append(('power', enabled, width))
        self.enabled = (enabled,) * 7

    def calibrate_effector(self):
        self.calls.append(('calibrate_effector',))
        self.position = self.position[:6] + (0.0,)

    def _write_mode(self, name):
        self.calls.append(('mode', name))
        self.status['ctrl_mode'] = 2 if name in ('drag', 'hold') else 0 if name == 'standby' else 1
        self.status['teach_status'] = {'drag': 1, 'hold': 2}.get(name, 0)
        self.status['mode_feedback'] = {
            'joint': 1, 'point': 0, 'linear': 2, 'mit': 6,
        }.get(name, 0)

    def joints(self, position, speed):
        self.calls.append(('joints', position, speed))
        self.position = tuple(position) + self.position[6:]

    def cartesian(self, pose, speed, mode):
        self.calls.append(('cartesian', pose, speed, mode))

    def mit(self, command):
        super().mit(command)
        self.position = tuple(command.position) + self.position[6:]

    def _write_mit(self, command):
        self.calls.append(('mit', command))

    def grip(self, width, effort, interpolate=True):
        self.calls.append(('grip', width, effort))
        if effort > 0:
            self.position = self.position[:6] + (width,)

    def _disconnect(self):
        pass


@pytest.fixture
def fake_arm(monkeypatch):
    original_mit = Controller.mit
    original_send_mit = Controller._send_mit

    def mit(self, command):
        original_mit(self, command)
        if isinstance(self.arm, FakeArm) and self.mode == 'mit':
            self.arm.position = tuple(command.position) + self.arm.position[6:]

    def _send_mit(self, command, sample):
        before = self.arm.position if isinstance(self.arm, FakeArm) else None
        original_send_mit(self, command, sample)
        if isinstance(self.arm, FakeArm) and before is not None:
            if self.debug_session:
                self.arm.position = tuple(command.position) + self.arm.position[6:]
            else:
                self.arm.position = before

    monkeypatch.setattr('piper_x_driver.node.SdkArm', FakeArm)
    monkeypatch.setattr(Controller, 'mit', mit)
    monkeypatch.setattr(Controller, '_send_mit', _send_mit)
    return FakeArm
