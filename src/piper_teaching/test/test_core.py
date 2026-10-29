from dataclasses import replace
import math

import pytest

from piper_teaching.bilateral import (
    ArmSample, BilateralCoupling, TeachingConfig, TeachingFault, references,
)
from piper_x_driver.backend import FEEDBACK_FIELDS


def samples(now=100.0):
    return tuple(
        ArmSample((offset, 0.5, -0.5, 0.0, 0.0, 0.0, 0.02 + offset / 10),
                  (now,) * 11, (True,) * 7, mode=6,
                  velocity=(offset + 0.1,) * 6)
        for offset in (0.0, 0.01, 0.02, 0.03))


def config():
    return TeachingConfig(kp=(3.0,) * 6, kd=(0.5,) * 6, gripper_effort=0.3)


def test_home_engage_then_pair_coupling():
    state = samples()
    state = state[:2] + tuple(replace(sample, position=(
        sample.position[0] + 0.5, *sample.position[1:6], 0.06)) for sample in state[2:])
    assert references(state, config(), 100) == tuple(
        state[index].position for index in (2, 3, 0, 1))
    coupling = BilateralCoupling(config())
    coupling.start(state, 100)
    target, kp = coupling.step(state, 100.01, False)
    assert target == tuple(sample.position for sample in state)
    assert kp == config().kp
    goals = coupling.engage_goals
    assert goals[0] == pytest.approx(goals[2])
    assert goals[1] == pytest.approx(goals[3])
    now = 100.01
    while not coupling.engaged:
        now += 0.02
        fresh = tuple(replace(sample, received=(now,) * 11) for sample in state)
        previous = target
        target, _ = coupling.step(fresh, now, True)
        assert all(abs(q - old) <= math.radians(24) * 0.02 + 1e-12
                   for arm, before in zip(target, previous)
                   for q, old in zip(arm[:6], before[:6]))
        assert [arm[6] for arm in target] == [sample.position[6] for sample in state]
    assert all(arm[:6] == pytest.approx(goal) for arm, goal in zip(target, goals))
    assert coupling.engaged
    assert coupling.velocities == ((0.0,) * 6,) * 4
    now += 0.02
    fresh = tuple(replace(sample, received=(now,) * 11) for sample in state)
    target, _ = coupling.step(fresh, now, True)
    assert target[0] == pytest.approx(state[2].position)
    assert coupling.velocities == tuple(
        state[index].velocity for index in (2, 3, 0, 1))
    previous = coupling.targets
    now += 1.5
    with pytest.raises(TeachingFault, match='deadline'):
        coupling.step(fresh, now, True)
    assert coupling.active and coupling.targets == previous
    now += 0.02
    fresh = tuple(replace(sample, received=(now,) * 11) for sample in fresh)
    coupling.step(fresh, now, True)
    assert coupling.active


def test_master_slave_mode_self_references_masters_and_couples_slaves():
    state = samples()
    state = state[:2] + tuple(replace(sample, position=(
        sample.position[0] + 0.5, *sample.position[1:6], 0.06)) for sample in state[2:])
    ms_config = replace(config(), mode='master_slave')
    assert references(state, ms_config, 100) == (
        state[0].position, state[1].position, state[0].position, state[1].position,
    )
    coupling = BilateralCoupling(ms_config)
    coupling.start(state, 100)
    now = 100.01
    while not coupling.engaged:
        now += 0.02
        fresh = tuple(replace(sample, received=(now,) * 11) for sample in state)
        target, kp = coupling.step(fresh, now, True)
        assert target[0][:6] == pytest.approx(state[0].position[:6])
        assert target[1][:6] == pytest.approx(state[1].position[:6])
    assert coupling.engaged
    assert target[2][:6] == pytest.approx(state[0].position[:6])
    assert target[3][:6] == pytest.approx(state[1].position[:6])
    assert kp == ms_config.kp
    now += 0.02
    fresh = tuple(replace(sample, received=(now,) * 11) for sample in state)
    target, _ = coupling.step(fresh, now, True)
    assert target == (
        state[0].position, state[1].position, state[0].position, state[1].position,
    )
    assert coupling.velocities == (
        (0.0,) * 6, (0.0,) * 6, state[0].velocity[:6], state[1].velocity[:6],
    )


@pytest.mark.parametrize('angle_deg', [-2.0, 195.0])
def test_references_preserve_raw_j2_outside_default_model_workspace(angle_deg):
    state = tuple(replace(sample, position=(
        sample.position[0], math.radians(angle_deg), *sample.position[2:]))
        for sample in samples())
    refs = references(state, config(), 100)
    assert all(ref[1] == math.radians(angle_deg) for ref in refs)


@pytest.mark.parametrize('index', range(11))
def test_every_feedback_segment_must_be_fresh(index):
    state = list(samples())
    stamps = list(state[0].received)
    stamps[index] = 98.99
    state[0] = replace(state[0], received=tuple(stamps))
    with pytest.raises(TeachingFault, match='expired') as error:
        references(tuple(state), config(), 100)
    assert FEEDBACK_FIELDS[index] in str(error.value)


@pytest.mark.parametrize('fault', [
    {'fault': 'overtemperature'}, {'enabled': (True,) * 5 + (False, True)},
])
def test_faults_block_start_without_modifying_measurements(fault):
    state = (replace(samples()[0], **fault),) + samples()[1:]
    with pytest.raises(TeachingFault):
        BilateralCoupling(config()).start(state, 100)
