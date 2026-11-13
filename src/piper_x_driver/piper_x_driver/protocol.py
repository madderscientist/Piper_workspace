"""Hardware-free CAN protocol constants, bounds, and telemetry helpers shared across packages."""

from collections import deque
import math

STATUS_FIELDS = (
    'ctrl_mode', 'arm_status', 'mode_feedback', 'teach_status',
    'motion_status', 'trajectory_num', 'err_code',
)
FEEDBACK_FIELDS = (
    'joint12(0x2A5)', 'joint34(0x2A6)', 'joint56(0x2A7)',
    'gripper(0x2A8)', 'arm_status(0x2A1)',
) + tuple(f'driver{i}(0x{0x260 + i:X})' for i in range(1, 7))

MIT_GAIN_SCALE = (24.75, 24.75, 28.41, 2.10, 1.36, 1.0)
_MIT_KP_MAX = 500.0
_MIT_KD_SPAN = 10.0
_MIT_KP_STEPS = 4095
MIT_KP_LSB = tuple(scale * _MIT_KP_MAX / _MIT_KP_STEPS for scale in MIT_GAIN_SCALE)
MIT_KD_LSB = tuple(scale * _MIT_KD_SPAN / _MIT_KP_STEPS for scale in MIT_GAIN_SCALE)

MAX_GRAVITY_OFFSET = 1.0
MIT_POSITION_LIMIT = 12.5
LOWER = (-MIT_POSITION_LIMIT,) * 6
UPPER = (MIT_POSITION_LIMIT,) * 6
SLIDER_LOWER = tuple(map(math.radians, (-150, 0, -170, -89, -89, -170)))
SLIDER_UPPER = tuple(map(math.radians, (150, 180, 0, 89, 89, 170)))
MODES = ('standby', 'joint', 'point', 'linear', 'mit', 'drag', 'hold')


def fresh(ages, timeout: float = 1.0) -> bool:
    return all(age is not None and math.isfinite(age) and 0 <= age <= timeout for age in ages)


def feedback_problem(ages, timeout: float = 1.0) -> str:
    if len(ages) != len(FEEDBACK_FIELDS):
        return f'expected 11 feedback segments, received {len(ages)}'
    problems = []
    for field, age in zip(FEEDBACK_FIELDS, ages):
        if age is None or age == -1:
            problems.append(f'{field}=missing')
        elif not math.isfinite(age) or not 0 <= age <= timeout:
            problems.append(
                f'{field} age={age * 1000:.3f} ms (required 0..{timeout * 1000:g} ms)'
            )
    return '; '.join(problems)


def vector(values, name: str, lower, upper) -> None:
    if len(values) != 6 or any(
        not math.isfinite(value) or not lo <= value <= hi
        for value, lo, hi in zip(values, lower, upper)
    ):
        raise ValueError(f'{name}: six finite values within the documented bounds required')


class RateMeter:
    """Sliding-window event frequency tracker (Hz) over a monotonic clock window."""

    __slots__ = ('_stamps', '_window')

    def __init__(self, window: float = 1.0):
        self._stamps: deque[float] = deque(maxlen=1024)
        self._window = float(window)

    def mark(self, now: float) -> None:
        self._stamps.append(float(now))

    def hz(self, now: float) -> float:
        stamps = self._stamps
        cutoff = float(now) - self._window
        while stamps and stamps[0] < cutoff:
            stamps.popleft()
        n = len(stamps)
        if n < 2:
            return 0.0
        span = stamps[-1] - stamps[0]
        return round((n - 1) / span, 1) if span > 1e-6 else 0.0

    def clear(self) -> None:
        self._stamps.clear()
