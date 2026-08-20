import math

from openpilot.selfdrive.controls.controlsd import valid_position


def test_valid_position():
  assert valid_position((1.0, 2.0, 3.0))
  assert not valid_position((1.0, 2.0))
  assert not valid_position((1.0, math.nan, 3.0))
