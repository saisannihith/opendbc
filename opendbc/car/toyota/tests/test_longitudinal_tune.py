import pytest

from opendbc.car.common.pid import PIDController
from opendbc.car.toyota.carcontroller import ACCEL_PID_UNWIND, STOPPING_PID_UNWIND, unwind_long_pid


def make_pid(integral):
  pid = PIDController(0.0, 0.5, rate=100 / 3)
  pid.i = integral
  return pid


def test_over_decelerating_stop_unwinds_negative_integral_smoothly():
  pid = make_pid(-0.24)
  values = []
  for _ in range(100):
    unwind_long_pid(pid, True, 0.55, -0.50, -0.33, -0.33)
    values.append(pid.i)

  assert values[0] == pytest.approx(-0.24 + STOPPING_PID_UNWIND)
  assert all(0.0 <= current - previous <= STOPPING_PID_UNWIND + 1e-12
             for previous, current in zip([-0.24, *values[:-1]], values, strict=True))
  assert values[-1] == 0.0


@pytest.mark.parametrize(("stopping", "v_ego", "a_ego", "accel", "requested_accel"), (
  (False, 0.55, -0.50, -0.33, -0.33),
  (True, 0.05, -0.50, -0.33, -0.33),
  (True, 1.0, -0.50, -0.33, -0.33),
  (True, 0.55, -0.37, -0.33, -0.33),
  (True, 0.55, -1.20, -0.33, -1.00),
))
def test_non_terminal_states_keep_the_normal_unwind(stopping, v_ego, a_ego, accel, requested_accel):
  pid = make_pid(-0.24)
  unwind_long_pid(pid, stopping, v_ego, a_ego, accel, requested_accel)
  assert pid.i == pytest.approx(-0.24 + ACCEL_PID_UNWIND)


def test_positive_integral_keeps_the_normal_unwind_during_a_stop():
  pid = make_pid(0.24)
  unwind_long_pid(pid, True, 0.55, -0.50, -0.33, -0.33)
  assert pid.i == pytest.approx(0.24 - ACCEL_PID_UNWIND)
