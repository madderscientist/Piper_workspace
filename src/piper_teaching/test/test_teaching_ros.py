from dataclasses import replace
import time

from piper_teaching.bilateral import ArmSample, TeachingFault
from piper_teaching.node import TeachingNode
import rclpy
from rclpy.parameter import Parameter


def test_enable_acknowledgement_waits_for_updated_enabled_telemetry(monkeypatch):
    rclpy.init()
    teaching = TeachingNode(parameter_overrides=[
        Parameter('dry_run', value=False),
        Parameter('kp', value=[3.0] * 6),
        Parameter('kd', value=[0.5] * 6),
        Parameter('gripper_effort', value=0.3),
    ])
    now = time.monotonic()
    sample = ArmSample(
        (0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.02), (now,) * 11, (False,) * 7,
    )
    samples = [sample] * 4
    monkeypatch.setattr(teaching, 'samples', lambda _: tuple(samples))
    monkeypatch.setattr(teaching, '_results', lambda _: True)
    teaching.phase, teaching.deadline = 'enabling', now + 5.0
    try:
        teaching._tick()
        assert teaching.phase == 'enabling'
        assert not teaching.coupling.active
        samples[:] = [replace(sample, enabled=(True,) * 7)] * 4
        teaching._tick()
        assert teaching.phase == 'preloading'
        assert teaching.coupling.active
        targets = teaching.coupling.targets

        def expired(_):
            raise TeachingFault('expired ROS feedback')

        monkeypatch.setattr(teaching, 'samples', expired)
        teaching._tick()
        assert teaching.holding and teaching.phase == 'preloading'
        assert teaching.coupling.active and teaching.coupling.targets == targets
        monkeypatch.setattr(teaching, 'samples', lambda _: tuple(samples))
        teaching._tick()
        assert not teaching.holding and teaching.coupling.active
        teaching.power_clients = [
            type('Client', (), {
                'service_is_ready': lambda _: True,
                'call_async': lambda *_: None,
            })() for _ in range(4)
        ]
        teaching.mode_clients = teaching.power_clients
        samples[:] = [sample] * 4
        teaching.request_retry()
        teaching._tick()
        assert teaching.phase == 'enabling' and not teaching.retry_requested
    finally:
        teaching.destroy_node()
        rclpy.try_shutdown()
