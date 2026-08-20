#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path
import struct
import time

from pymavlink import mavutil

from openpilot.cereal import messaging


RACE_STATUS_ID = 1
TRACK_PACKET_ID = 2
SIM_RESET_COMMAND = 31000
TRACK_GATE = struct.Struct("<Hfffffffff")
RACE_STATUS = struct.Struct("<BQqqIq")


def rotate_body_to_ned(quaternion, vector):
  magnitude = math.sqrt(sum(value * value for value in quaternion))
  if magnitude <= 1e-9:
    raise ValueError("quaternion must be nonzero")
  w, x, y, z = (value / magnitude for value in quaternion)
  q = (x, y, z)
  first = (
    2.0 * (q[1] * vector[2] - q[2] * vector[1]),
    2.0 * (q[2] * vector[0] - q[0] * vector[2]),
    2.0 * (q[0] * vector[1] - q[1] * vector[0]),
  )
  second = (
    q[1] * first[2] - q[2] * first[1],
    q[2] * first[0] - q[0] * first[2],
    q[0] * first[1] - q[1] * first[0],
  )
  return tuple(vector[index] + w * first[index] + second[index] for index in range(3))


def parse_track(payload):
  if len(payload) < 2:
    return []
  gate_count, = struct.unpack_from("<H", payload)
  offset = 2
  gates = []
  for _ in range(gate_count):
    if offset + TRACK_GATE.size > len(payload):
      return []
    values = TRACK_GATE.unpack_from(payload, offset)
    offset += TRACK_GATE.size
    gate_id = int(values[0])
    position = tuple(float(value) for value in values[1:4])
    orientation = tuple(float(value) for value in values[4:8])
    width, height = (float(value) for value in values[8:10])
    finite = all(math.isfinite(value) for value in values[1:])
    quaternion_norm = math.sqrt(sum(value * value for value in orientation))
    if not finite or width <= 0.0 or height <= 0.0 or not 0.99 <= quaternion_norm <= 1.01:
      return []
    gates.append((gate_id, position, orientation, width, height))
  if len({gate[0] for gate in gates}) != gate_count:
    return []
  return gates


def aperture_center(gate):
  _, position, orientation, _, height = gate
  vertical = rotate_body_to_ned(orientation, (0.0, 0.0, 1.0))
  return tuple(position[index] - 0.5 * height * vertical[index] for index in range(3))


class TrackTransfer:
  def __init__(self):
    self.chunks = {}
    self.expected = {}
    self.gates = []

  def add(self, message):
    kind = message.get_type()
    if kind == "DATA_TRANSMISSION_HANDSHAKE":
      transfer_id = int(message.width)
      self.chunks[transfer_id] = {}
      self.expected[transfer_id] = int(message.packets)
      return
    if kind != "ENCAPSULATED_DATA":
      return
    payload = bytes(message.data)
    if len(payload) < 3 or payload[0] != TRACK_PACKET_ID:
      return
    _, transfer_id = struct.unpack_from("<BH", payload)
    if transfer_id not in self.expected:
      return
    self.chunks[transfer_id][int(message.seqnr)] = payload[3:]
    if len(self.chunks[transfer_id]) != self.expected[transfer_id]:
      return
    joined = b"".join(self.chunks[transfer_id][index] for index in range(self.expected[transfer_id]))
    del self.chunks[transfer_id]
    del self.expected[transfer_id]
    self.gates = parse_track(joined)


def parse_race_status(message):
  if message.get_type() != "ENCAPSULATED_DATA":
    return None
  payload = bytes(message.data)
  if len(payload) < RACE_STATUS.size or payload[0] != RACE_STATUS_ID:
    return None
  return RACE_STATUS.unpack_from(payload)


class Vq1Adapter:
  def __init__(self, endpoint, result_path, reset=False, arm=False, timeout=240.0, goals=None):
    self.endpoint = endpoint
    self.result_path = Path(result_path)
    self.reset = reset
    self.arm = arm
    self.timeout = timeout
    self.link = None
    self.boot_ms = 0
    self.track = TrackTransfer()
    self.goals = [] if goals is None else goals
    self.pm = messaging.PubMaster(["vehicleState", "positionGoal"])
    self.sm = messaging.SubMaster(["positionTarget"], poll="positionTarget")
    self.active_gate = None
    self.race_boot_ms = None
    self.race_start_ms = None
    self.gate_sequence = []
    self.collisions = 0
    self.latest_position = None
    self.latest_state_at = None
    self.latest_target = None
    self.latest_target_at = None
    self.started_at = None
    self.commands = 0
    self.next_goal_at = 0.0
    self.next_arm_at = 0.0
    self.armed = False

  def race_live(self):
    return (self.race_boot_ms is not None and self.race_start_ms is not None
            and self.race_start_ms >= 0 and self.race_boot_ms >= self.race_start_ms)

  def send_command(self, command, param1=0.0):
    self.link.mav.command_long_send(
      self.link.target_system, self.link.target_component, command, 0,
      param1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    )

  def request_message(self, message_id, rate_hz):
    self.link.mav.command_long_send(
      self.link.target_system, self.link.target_component,
      mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
      message_id, 1_000_000 / rate_hz, 0.0, 0.0, 0.0, 0.0, 0.0,
    )

  def publish_state(self, message):
    if int(message.frame_id) != mavutil.mavlink.MAV_FRAME_LOCAL_NED:
      raise RuntimeError(f"unexpected odometry frame {message.frame_id}")
    if int(message.child_frame_id) != mavutil.mavlink.MAV_FRAME_BODY_NED:
      raise RuntimeError(f"unexpected odometry child frame {message.child_frame_id}")
    position = (float(message.x), float(message.y), float(message.z))
    velocity = rotate_body_to_ned(tuple(float(value) for value in message.q),
                                  (float(message.vx), float(message.vy), float(message.vz)))
    event = messaging.new_message("vehicleState")
    event.vehicleState.positionNED = position
    event.vehicleState.velocityNED = velocity
    self.pm.send("vehicleState", event)
    self.latest_position = position
    self.latest_state_at = time.monotonic()

  def publish_goal(self):
    if self.active_gate is None or self.active_gate >= self.goal_count():
      return
    event = messaging.new_message("positionGoal")
    if self.goals:
      event.positionGoal.positionNED = self.goals[self.active_gate]
    else:
      event.positionGoal.positionNED = aperture_center(self.track.gates[self.active_gate])
    self.pm.send("positionGoal", event)

  def goal_count(self):
    return len(self.goals) if self.goals else len(self.track.gates)

  def receive_target(self):
    self.sm.update(0)
    if self.sm.updated["positionTarget"]:
      target = tuple(float(value) for value in self.sm["positionTarget"].positionNED)
      if len(target) != 3 or not all(math.isfinite(value) for value in target):
        raise RuntimeError("invalid position target")
      self.latest_target = target
      self.latest_target_at = time.monotonic()

  def send_target(self):
    now = time.monotonic()
    if (self.latest_target is None or self.latest_target_at is None or self.latest_state_at is None
        or now - self.latest_target_at > 0.25 or now - self.latest_state_at > 0.25):
      return
    mask = (
      mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
      | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
      | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
      | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
      | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
      | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
      | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
      | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    self.link.mav.set_position_target_local_ned_send(
      int(time.monotonic() * 1000) - self.boot_ms,
      self.link.target_system, self.link.target_component,
      mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
      *self.latest_target, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    )
    self.commands += 1

  def write_result(self, outcome):
    elapsed = None if self.started_at is None else time.monotonic() - self.started_at
    result = {
      "outcome": outcome,
      "elapsed_s": elapsed,
      "gate_sequence": self.gate_sequence,
      "collisions": self.collisions,
      "commands": self.commands,
    }
    self.result_path.parent.mkdir(parents=True, exist_ok=True)
    self.result_path.write_text(json.dumps(result, indent=2) + "\n")
    return result

  def handle_message(self, message):
    had_track = bool(self.track.gates)
    self.track.add(message)
    if self.track.gates and not had_track:
      self.publish_goal()
    kind = message.get_type()
    if kind == "ODOMETRY":
      self.publish_state(message)
    elif kind == "HEARTBEAT":
      self.armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    elif kind == "COLLISION" and self.race_live():
      self.collisions += 1
    status = parse_race_status(message)
    if status is not None:
      self.race_boot_ms = int(status[1])
      self.race_start_ms = int(status[2])
      gate = int(status[4])
      if gate != self.active_gate:
        self.active_gate = gate
        self.gate_sequence.append(gate)
        self.publish_goal()

  def run(self):
    self.link = mavutil.mavlink_connection(self.endpoint)
    heartbeat = self.link.wait_heartbeat(timeout=10)
    if heartbeat is None:
      raise RuntimeError("no VQ1 heartbeat")
    self.boot_ms = int(time.monotonic() * 1000)
    if self.reset:
      self.send_command(SIM_RESET_COMMAND)
    self.request_message(getattr(mavutil.mavlink, "MAVLINK_MSG_ID_ODOMETRY", 331), 50)
    connected_at = time.monotonic()
    next_command = connected_at
    outcome = "timeout"
    try:
      while time.monotonic() - connected_at < self.timeout:
        message = self.link.recv_match(blocking=False)
        while message is not None:
          self.handle_message(message)
          message = self.link.recv_match(blocking=False)
        if self.collisions:
          outcome = "collision"
          break
        if self.goal_count() and self.active_gate is not None and self.active_gate >= self.goal_count():
          outcome = "pass"
          break
        if not self.race_live():
          time.sleep(0.001)
          continue
        if self.started_at is None:
          self.started_at = time.monotonic()
        self.receive_target()
        now = time.monotonic()
        if self.arm and not self.armed and now >= self.next_arm_at:
          self.send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
          self.next_arm_at = now + 0.5
        if now >= self.next_goal_at:
          self.publish_goal()
          self.next_goal_at = now + 0.1
        if now >= next_command:
          self.send_target()
          next_command = now + 0.02
        time.sleep(0.001)
    finally:
      if self.arm:
        self.send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0)
    result = self.write_result(outcome)
    print(json.dumps(result, separators=(",", ":")), flush=True)
    if outcome != "pass":
      raise SystemExit(1)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
  parser.add_argument("--result", default="/tmp/minipilot-vq1-result.json")
  parser.add_argument("--reset", action="store_true")
  parser.add_argument("--arm", action="store_true")
  parser.add_argument("--timeout", type=float, default=240.0)
  parser.add_argument("--goals", type=Path)
  args = parser.parse_args()
  goals = None if args.goals is None else json.loads(args.goals.read_text())
  Vq1Adapter(args.endpoint, args.result, args.reset, args.arm, args.timeout, goals).run()


if __name__ == "__main__":
  main()
