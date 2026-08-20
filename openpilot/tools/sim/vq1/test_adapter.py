import struct

from openpilot.tools.sim.vq1.adapter import aperture_center, parse_track, rotate_body_to_ned


def test_rotate_identity():
  assert rotate_body_to_ned((1.0, 0.0, 0.0, 0.0), (1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)


def test_track_and_aperture_center():
  gate = struct.pack("<Hfffffffff", 4, 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0, 1.5, 2.0)
  gates = parse_track(struct.pack("<H", 1) + gate)
  assert len(gates) == 1
  assert aperture_center(gates[0]) == (1.0, 2.0, 2.0)


def test_rejects_invalid_track():
  gate = struct.pack("<Hfffffffff", 4, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.5, 2.0)
  assert parse_track(struct.pack("<H", 1) + gate) == []
