#!/usr/bin/env python3
import math
import time

import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper


STATE_MAX_AGE = 0.1
GOAL_MAX_AGE = 0.25


def valid_position(values) -> bool:
  return len(values) == 3 and all(math.isfinite(value) for value in values)


class Controls:
  def __init__(self) -> None:
    self.sm = messaging.SubMaster(["vehicleState", "positionGoal"])
    self.pm = messaging.PubMaster(["positionTarget"])

  def step(self) -> bool:
    self.sm.update(0)
    now = time.monotonic()
    state_fresh = self.sm.seen["vehicleState"] and now - self.sm.recv_time["vehicleState"] <= STATE_MAX_AGE
    goal_fresh = self.sm.seen["positionGoal"] and now - self.sm.recv_time["positionGoal"] <= GOAL_MAX_AGE
    if not state_fresh or not goal_fresh:
      return False

    position = self.sm["positionGoal"].positionNED
    if not valid_position(position):
      return False

    message = messaging.new_message("positionTarget", valid=True)
    message.positionTarget.positionNED = position
    self.pm.send("positionTarget", message)
    return True

  def run(self) -> None:
    ratekeeper = Ratekeeper(50, print_delay_threshold=None)
    while True:
      self.step()
      ratekeeper.monitor_time()


def main() -> None:
  config_realtime_process(4, Priority.CTRL_HIGH)
  Controls().run()


if __name__ == "__main__":
  main()
