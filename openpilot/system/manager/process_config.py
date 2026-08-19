import os
import operator
import platform

from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.common.hardware import PC, COMMA_HARDWARE
from openpilot.system.manager.process import PythonProcess, NativeProcess, DaemonProcess

WEBCAM = os.getenv("USE_WEBCAM") is not None

def driverview(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started or params.get_bool("IsDriverViewEnabled")

def notcar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and CP.notCar

def iscar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not CP.notCar

def logging(started: bool, params: Params, CP: car.CarParams) -> bool:
  run = (not CP.notCar) or not params.get_bool("DisableLogging")
  return started and run

def ublox_available() -> bool:
  return os.path.exists('/dev/ttyHS0') and not os.path.exists('/persist/comma/use-quectel-gps')

def ublox(started: bool, params: Params, CP: car.CarParams) -> bool:
  use_ublox = ublox_available()
  if use_ublox != params.get_bool("UbloxAvailable"):
    params.put_bool("UbloxAvailable", use_ublox, block=True)
  return started and use_ublox

def joystick(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("JoystickDebugMode")

def not_joystick(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("JoystickDebugMode")

def long_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("LongitudinalManeuverMode")

def lat_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("LateralManeuverMode")

def not_long_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("LongitudinalManeuverMode")

def qcomgps(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not ublox_available()

def always_run(started: bool, params: Params, CP: car.CarParams) -> bool:
  return True

def only_onroad(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started

def only_offroad(started: bool, params: Params, CP: car.CarParams) -> bool:
  return not started

def livestream(started: bool, params: Params, CP: car.CarParams) -> bool:
  return params.get_bool("IsLiveStreaming")

def or_(*fns):
  return lambda *args: operator.or_(*(fn(*args) for fn in fns))

def and_(*fns):
  return lambda *args: operator.and_(*(fn(*args) for fn in fns))

def not_(*fns):
  return lambda *args: operator.not_(*(fn(*args) for fn in fns))

processes_and_rules = [
  (DaemonProcess("manage_athenad", "openpilot.system.athena.manage_athenad", "AthenadPid"), always_run),

  (NativeProcess("loggerd", "openpilot/system/loggerd", ["./loggerd"]), logging),
  (NativeProcess("encoderd", "openpilot/system/loggerd", ["./encoderd"]), only_onroad),
  (NativeProcess("stream_encoderd", "openpilot/system/loggerd", ["./encoderd", "--stream"]), or_(and_(livestream, not_(iscar)), notcar)),
  (PythonProcess("logmessaged", "openpilot.system.logmessaged"), always_run),

  (NativeProcess("camerad", "openpilot/system/camerad", ["./camerad"], enabled=not WEBCAM), or_(driverview, livestream)),
  (PythonProcess("webcamerad", "openpilot.system.camerad.webcam.camerad", enabled=WEBCAM), driverview),
  (PythonProcess("proclogd", "openpilot.system.proclogd", enabled=platform.system() != "Darwin"), only_onroad),
  (PythonProcess("journald", "openpilot.system.journald", enabled=platform.system() != "Darwin"), only_onroad),
  (PythonProcess("micd", "openpilot.system.micd"), iscar),
  (PythonProcess("timed", "openpilot.system.timed", enabled=not PC), always_run),

  (PythonProcess("modeld", "openpilot.selfdrive.modeld.modeld"), only_onroad),
  (PythonProcess("dmonitoringmodeld", "openpilot.selfdrive.modeld.dmonitoringmodeld", enabled=(WEBCAM or not PC)), driverview),

  (PythonProcess("sensord", "openpilot.system.sensord.sensord", enabled=not PC), only_onroad),
  (PythonProcess("ui", "openpilot.selfdrive.ui.ui"), always_run),
  (PythonProcess("soundd", "openpilot.selfdrive.ui.soundd"), driverview),
  (PythonProcess("locationd", "openpilot.selfdrive.locationd.locationd"), only_onroad),
  (NativeProcess("_pandad", "openpilot/selfdrive/pandad", ["./pandad"], enabled=False), always_run),
  (PythonProcess("calibrationd", "openpilot.selfdrive.locationd.calibrationd"), only_onroad),
  (PythonProcess("torqued", "openpilot.selfdrive.locationd.torqued"), only_onroad),
  (PythonProcess("controlsd", "openpilot.selfdrive.controls.controlsd"), and_(not_joystick, iscar)),
  (PythonProcess("joystickd", "openpilot.tools.joystick.joystickd"), or_(joystick, notcar)),
  (PythonProcess("selfdrived", "openpilot.selfdrive.selfdrived.selfdrived"), only_onroad),
  (PythonProcess("card", "openpilot.selfdrive.car.card"), only_onroad),
  (PythonProcess("deleter", "openpilot.system.loggerd.deleter"), always_run),
  (PythonProcess("dmonitoringd", "openpilot.selfdrive.monitoring.dmonitoringd", enabled=(WEBCAM or not PC)), driverview),
  (PythonProcess("qcomgpsd", "openpilot.system.qcomgpsd.qcomgpsd", enabled=COMMA_HARDWARE), qcomgps),
  (PythonProcess("pandad", "openpilot.selfdrive.pandad.pandad"), always_run),
  (PythonProcess("paramsd", "openpilot.selfdrive.locationd.paramsd"), only_onroad),
  (PythonProcess("lagd", "openpilot.selfdrive.locationd.lagd"), only_onroad),
  (PythonProcess("ubloxd", "openpilot.system.ubloxd.ubloxd", enabled=COMMA_HARDWARE), ublox),
  (PythonProcess("pigeond", "openpilot.system.ubloxd.pigeond", enabled=COMMA_HARDWARE), ublox),
  (PythonProcess("plannerd", "openpilot.selfdrive.controls.plannerd"), not_long_maneuver),
  (PythonProcess("maneuversd", "openpilot.tools.longitudinal_maneuvers.maneuversd"), long_maneuver),
  (PythonProcess("lateral_maneuversd", "openpilot.tools.lateral_maneuvers.lateral_maneuversd"), lat_maneuver),
  (PythonProcess("radard", "openpilot.selfdrive.controls.radard"), only_onroad),
  (PythonProcess("hardwared", "openpilot.system.hardware.hardwared"), always_run),
  (PythonProcess("modem", "openpilot.common.hardware.comma.modem", enabled=COMMA_HARDWARE), always_run),
  (PythonProcess("tombstoned", "openpilot.system.tombstoned", enabled=not PC), always_run),
  (PythonProcess("updated", "openpilot.system.updated.updated", enabled=not PC), only_offroad),
  (PythonProcess("uploader", "openpilot.system.loggerd.uploader"), always_run),

  # debug procs
  (NativeProcess("bridge", "openpilot/cereal/messaging", ["./bridge"]), notcar),
  (PythonProcess("webrtcd", "openpilot.system.webrtc.webrtcd"), or_(and_(livestream, not_(iscar)), notcar)),
  (PythonProcess("joystick", "openpilot.tools.joystick.joystick_control"), and_(joystick, iscar)),
]

procs = [p for p, _ in processes_and_rules]
managed_processes = {p.name: p for p in procs}
process_rules = {p.name: rule for p, rule in processes_and_rules}


def get_processes_to_run(started: bool, params: Params, CP: car.CarParams) -> set[str]:
  return {name for name, rule in process_rules.items() if rule(started, params, CP)}
