#!/usr/bin/env python3
"""VQ1 simulator lifecycle, telemetry, camera, and HTTP gateway.

One process owns the official MAVLink and vision UDP ports, then exposes
human-readable snapshots on http://127.0.0.1:8767.
"""

import argparse
import collections
import json
import math
import os
import socket
import struct
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from pymavlink import mavutil

from backends import BackendError, SimulatorBackend, create_backend


DEFAULT_HTTP_HOST = "127.0.0.1"
DEFAULT_HTTP_PORT = 8767
DEFAULT_MAVLINK_HOST = "127.0.0.1"
DEFAULT_MAVLINK_PORT = 14550
DEFAULT_VISION_HOST = "0.0.0.0"
DEFAULT_VISION_PORT = 5600
VISION_HEADER = struct.Struct("<IHHIIQ")
RACE_STATUS = struct.Struct("<BQqqIq")
TRACK_HEADER = struct.Struct("<BH")
TRACK_GATE = struct.Struct("<Hfffffffff")


CONTROL_SURFACES = [
    {
        "direction": "sim → pilot",
        "surface": "MAVLink HEARTBEAT",
        "rate": "sim-defined",
        "fields": "system/component identity, armed bit",
        "availability": "available",
    },
    {
        "direction": "sim ↔ pilot",
        "surface": "MAVLink TIMESYNC",
        "rate": "template requests 10 Hz",
        "fields": "tc1, ts1 nanoseconds",
        "availability": "available",
    },
    {
        "direction": "sim → pilot",
        "surface": "HIGHRES_IMU",
        "rate": "sim-defined",
        "fields": "accelerometer, gyroscope, timestamp and sensor flags",
        "availability": "observed live in VQ1 build 1.0.3391",
    },
    {
        "direction": "sim → pilot",
        "surface": "ATTITUDE / LOCAL_POSITION_NED / ODOMETRY",
        "rate": "sim-defined",
        "fields": "pose, velocity and body rates",
        "availability": "observed live in build 1.0.3391; template comments saying disabled are stale",
    },
    {
        "direction": "sim → pilot",
        "surface": "ENCAPSULATED_DATA type 1",
        "rate": "sim-defined",
        "fields": "sim time, race start/finish, active gate, last gate time",
        "availability": "available",
    },
    {
        "direction": "sim → pilot",
        "surface": "ENCAPSULATED_DATA type 2",
        "rate": "on track transfer",
        "fields": "gate IDs plus pose/dimensions",
        "availability": "transmitted, but privileged geometry is documented as nulled",
    },
    {
        "direction": "sim → pilot",
        "surface": "ACTUATOR_OUTPUT_STATUS",
        "rate": "sim-defined",
        "fields": "four motor outputs plus timestamp",
        "availability": "available",
    },
    {
        "direction": "sim → pilot",
        "surface": "COLLISION",
        "rate": "event-driven",
        "fields": "1001 gate / 1002 environment, threat level, impulse",
        "availability": "available",
    },
    {
        "direction": "sim → pilot",
        "surface": "FPV camera UDP",
        "rate": "sim-defined; measured live",
        "fields": "frame/chunk IDs, JPEG size, payload size, sim_time_ns, JPEG bytes",
        "availability": "observed live before race start on UDP 5600",
    },
    {
        "direction": "pilot → sim",
        "surface": "MAV_CMD_COMPONENT_ARM_DISARM",
        "rate": "event-driven",
        "fields": "arm/disarm",
        "availability": "supported; must be race-gated",
    },
    {
        "direction": "pilot → sim",
        "surface": "SET_ATTITUDE_TARGET",
        "rate": "template loop 250 Hz",
        "fields": "quaternion or body rates, normalized collective thrust",
        "availability": "supported; DCL extension mask bit 16 selects physical rad/s",
    },
    {
        "direction": "pilot → sim",
        "surface": "SET_POSITION_TARGET_LOCAL_NED",
        "rate": "template loop 250 Hz",
        "fields": "position/velocity/acceleration/yaw selected by type mask",
        "availability": "sample uses velocity mode; pose telemetry remains unavailable",
    },
    {
        "direction": "pilot → sim",
        "surface": "SET_ACTUATOR_CONTROL_TARGET",
        "rate": "template loop 250 Hz",
        "fields": "eight raw actuator values; first four are motors",
        "availability": "supported but unsafe without a mixer/controller",
    },
    {
        "direction": "pilot → sim",
        "surface": "MAV_CMD 31000",
        "rate": "event-driven",
        "fields": "simulator reset",
        "availability": "AI-GP extension",
    },
]


def json_safe(value):
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, bytes):
        return list(value)
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


class MonitorState:
    def __init__(self, backend: SimulatorBackend):
        self.lock = threading.Lock()
        self.frame_ready = threading.Condition(self.lock)
        self.started_at = time.time()
        self.backend = backend
        self.lifecycle_state = "stopped"
        self.lifecycle_error = None
        self.reset_frame_id = None
        self.reset_boot_time_ms = None
        self.reset_observed_rewind = False
        self.latest_messages = {}
        self.message_counts = collections.Counter()
        self.message_seen_at = {}
        self.message_times = collections.defaultdict(lambda: collections.deque(maxlen=200))
        self.race = {}
        self.track = {"transfers": {}, "gates": []}
        self.mavlink_error = None
        self.camera_error = None
        self.camera_jpeg = None
        self.camera = {
            "frames": 0,
            "frame_id": None,
            "sim_time_ns": None,
            "jpeg_size": None,
            "width": None,
            "height": None,
            "seen_at": None,
            "fps": 0.0,
        }
        self.frame_times = collections.deque(maxlen=90)

    def start_backend(self, config):
        requested_backend = config.get("backend", self.backend.name)
        if requested_backend != self.backend.name:
            raise BackendError(
                f"Gateway uses {self.backend.name}, not {requested_backend}"
            )

        with self.lock:
            if self.lifecycle_state not in {"stopped", "error"}:
                raise BackendError(
                    f"Cannot start from lifecycle {self.lifecycle_state}"
                )
            self.lifecycle_state = "starting"
            self.lifecycle_error = None
            self.race = {}
            self.track = {"transfers": {}, "gates": []}
            self.camera_jpeg = None
            self.camera.update(
                {
                    "frames": 0,
                    "frame_id": None,
                    "sim_time_ns": None,
                    "jpeg_size": None,
                    "width": None,
                    "height": None,
                    "seen_at": None,
                    "fps": 0.0,
                }
            )
            self.frame_times.clear()

        try:
            self.backend.start(config)
        except Exception as exc:
            with self.lock:
                self.lifecycle_state = "error"
                self.lifecycle_error = str(exc)
            raise

    def reset_episode(self, transport):
        with self.lock:
            if self.lifecycle_state not in {"physics_ready", "running", "ended"}:
                raise BackendError(
                    f"Cannot reset from lifecycle {self.lifecycle_state}"
                )
            self.reset_frame_id = self.camera["frame_id"]
            self.reset_boot_time_ms = self.race.get("sim_boot_time_ms")
            self.reset_observed_rewind = False
            self.lifecycle_state = "resetting"
            self.lifecycle_error = None

        try:
            self.backend.reset(transport)
        except Exception as exc:
            with self.lock:
                self.lifecycle_state = "error"
                self.lifecycle_error = str(exc)
            raise

    def stop_backend(self):
        with self.lock:
            if self.lifecycle_state == "stopped":
                return
            self.lifecycle_state = "stopping"
            self.lifecycle_error = None

        try:
            self.backend.stop()
        except Exception as exc:
            with self.lock:
                self.lifecycle_state = "error"
                self.lifecycle_error = str(exc)
            raise

        with self.lock:
            self.lifecycle_state = "stopped"
            self.race = {}
            self.track = {"transfers": {}, "gates": []}
            self.camera_jpeg = None
            self.camera.update(
                {
                    "frames": 0,
                    "frame_id": None,
                    "sim_time_ns": None,
                    "jpeg_size": None,
                    "width": None,
                    "height": None,
                    "seen_at": None,
                    "fps": 0.0,
                }
            )
            self.frame_times.clear()

    def update_message(self, msg):
        now = time.time()
        msg_type = msg.get_type()
        data = json_safe(msg.to_dict())
        data["_source_system"] = getattr(msg, "get_srcSystem", lambda: None)()
        data["_source_component"] = getattr(msg, "get_srcComponent", lambda: None)()
        data["_sequence"] = getattr(msg, "get_seq", lambda: None)()
        data["_received_at"] = now
        with self.lock:
            self.latest_messages[msg_type] = data
            self.message_counts[msg_type] += 1
            self.message_seen_at[msg_type] = now
            self.message_times[msg_type].append(now)

    def update_frame(self, frame_id, sim_time_ns, jpeg_bytes, image):
        now = time.time()
        height, width = image.shape[:2]
        with self.frame_ready:
            self.frame_times.append(now)
            fps = 0.0
            if len(self.frame_times) >= 2:
                span = self.frame_times[-1] - self.frame_times[0]
                if span > 0:
                    fps = (len(self.frame_times) - 1) / span
            self.camera_jpeg = bytes(jpeg_bytes)
            self.camera.update(
                {
                    "frames": self.camera["frames"] + 1,
                    "frame_id": frame_id,
                    "sim_time_ns": sim_time_ns,
                    "jpeg_size": len(jpeg_bytes),
                    "width": width,
                    "height": height,
                    "seen_at": now,
                    "fps": fps,
                }
            )
            self.frame_ready.notify_all()

    def snapshot_data(self):
        now = time.time()
        with self.lock:
            messages = {}
            for name, data in self.latest_messages.items():
                times = self.message_times[name]
                rate_hz = 0.0
                if len(times) >= 2 and times[-1] > times[0]:
                    rate_hz = (len(times) - 1) / (times[-1] - times[0])
                messages[name] = {
                    "count": self.message_counts[name],
                    "age_ms": round((now - self.message_seen_at[name]) * 1000, 1),
                    "rate_hz": round(rate_hz, 1),
                    "fields": data,
                }
            camera = dict(self.camera)
            camera["age_ms"] = (
                round((now - camera["seen_at"]) * 1000, 1)
                if camera["seen_at"] is not None
                else None
            )
            heartbeat_age = None
            if "HEARTBEAT" in self.message_seen_at:
                heartbeat_age = round((now - self.message_seen_at["HEARTBEAT"]) * 1000, 1)
            return {
                "monitor": {
                    "uptime_s": round(now - self.started_at, 1),
                    "mavlink_live": heartbeat_age is not None and heartbeat_age < 3000,
                    "heartbeat_age_ms": heartbeat_age,
                    "mavlink_error": self.mavlink_error,
                    "camera_live": camera["age_ms"] is not None and camera["age_ms"] < 1000,
                    "camera_error": self.camera_error,
                },
                "race": dict(self.race),
                "camera": camera,
                "track": json_safe(self.track),
                "messages": messages,
                "control_surfaces": CONTROL_SURFACES,
            }

    def lifecycle(self):
        """Advance startup only when the selected backend proves readiness."""
        with self.lock:
            lifecycle_state = self.lifecycle_state
        if lifecycle_state not in {"starting", "resetting"}:
            return lifecycle_state

        snapshot = self.snapshot_data()
        if not self.backend.is_alive():
            next_state = "error"
            error = "Simulator exited during lifecycle operation"
        elif lifecycle_state == "starting" and self.backend.is_ready(snapshot):
            next_state = "physics_ready"
            error = None
        elif lifecycle_state == "resetting" and self.reset_is_ready(snapshot):
            next_state = "physics_ready"
            error = None
        else:
            return lifecycle_state

        with self.lock:
            if self.lifecycle_state == lifecycle_state:
                self.lifecycle_state = next_state
                self.lifecycle_error = error
            return self.lifecycle_state

    def reset_is_ready(self, snapshot):
        monitor = snapshot["monitor"]
        frame_id = snapshot["camera"].get("frame_id")
        command_ack = snapshot["messages"].get("COMMAND_ACK", {})
        ack_fields = command_ack.get("fields", {})
        race = snapshot["race"]
        sim_boot_time_ms = race.get("sim_boot_time_ms")
        if (
            self.reset_boot_time_ms is not None
            and sim_boot_time_ms is not None
            and sim_boot_time_ms < self.reset_boot_time_ms
        ):
            self.reset_observed_rewind = True
        return (
            monitor["mavlink_live"]
            and monitor["camera_live"]
            and frame_id is not None
            and frame_id != self.reset_frame_id
            and ack_fields.get("command") == 31000
            and ack_fields.get("result") == 0
            and self.reset_observed_rewind
            and race.get("started") is True
            and race.get("race_start_boot_time_ms", -1) >= 0
            and race.get("active_gate_index") == 0
        )

    def snapshot(self):
        data = self.snapshot_data()
        data["simulator"] = {
            "backend": self.backend.name,
            "lifecycle": self.lifecycle(),
            "error": self.lifecycle_error,
            "process_alive": self.backend.is_alive(),
        }
        return data


class TelemetryReceiver(threading.Thread):
    def __init__(self, state, address):
        super().__init__(name="aigp-mavlink", daemon=True)
        self.state = state
        self.address = address
        self.track_chunks = {}
        self.track_expected = {}
        self.connection = None
        self.send_lock = threading.Lock()
        self.target_system = None
        self.target_component = None

    def run(self):
        try:
            connection = mavutil.mavlink_connection(self.address)
            self.connection = connection
            while True:
                msg = connection.recv_match(blocking=True, timeout=0.25)
                if msg is None:
                    continue
                if msg.get_type() == "BAD_DATA":
                    continue
                self.state.update_message(msg)
                if msg.get_type() == "HEARTBEAT":
                    self.target_system = msg.get_srcSystem()
                    self.target_component = msg.get_srcComponent()
                self.decode_special(msg)
        except Exception as exc:
            with self.state.lock:
                self.state.mavlink_error = repr(exc)

    def send_command_long(self, command):
        if self.connection is None or self.target_system is None:
            raise BackendError("MAVLink target is not ready")
        with self.send_lock:
            self.connection.mav.command_long_send(
                self.target_system,
                self.target_component or 0,
                command,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )

    def decode_special(self, msg):
        msg_type = msg.get_type()
        if msg_type == "DATA_TRANSMISSION_HANDSHAKE":
            transfer_id = int(msg.width)
            self.track_chunks[transfer_id] = {}
            self.track_expected[transfer_id] = int(msg.packets)
            with self.state.lock:
                self.state.track["transfers"][str(transfer_id)] = {
                    "expected_packets": int(msg.packets),
                    "received_packets": 0,
                }
            return
        if msg_type != "ENCAPSULATED_DATA":
            return
        payload = bytes(msg.data)
        if not payload:
            return
        data_type = payload[0]
        if data_type == 1 and len(payload) >= RACE_STATUS.size:
            values = RACE_STATUS.unpack_from(payload)
            race = {
                "data_type": values[0],
                "sim_boot_time_ms": values[1],
                "race_start_boot_time_ms": values[2],
                "race_finish_time_ns": values[3],
                "active_gate_index": values[4],
                "last_gate_race_time": values[5],
                "started": values[2] >= 0,
                "finished": values[3] >= 0,
            }
            with self.state.lock:
                self.state.race = race
        elif data_type == 2 and len(payload) >= TRACK_HEADER.size:
            _, transfer_id = TRACK_HEADER.unpack_from(payload)
            if transfer_id not in self.track_expected:
                return
            self.track_chunks[transfer_id][int(msg.seqnr)] = payload[TRACK_HEADER.size :]
            expected = self.track_expected[transfer_id]
            with self.state.lock:
                self.state.track["transfers"][str(transfer_id)] = {
                    "expected_packets": expected,
                    "received_packets": len(self.track_chunks[transfer_id]),
                }
            if len(self.track_chunks[transfer_id]) == expected and all(
                index in self.track_chunks[transfer_id] for index in range(expected)
            ):
                assembled = b"".join(self.track_chunks[transfer_id][index] for index in range(expected))
                self.decode_track(assembled)
                del self.track_chunks[transfer_id]
                del self.track_expected[transfer_id]

    def decode_track(self, payload):
        if len(payload) < 2:
            return
        gate_count = struct.unpack_from("<H", payload)[0]
        offset = 2
        gates = []
        for _ in range(gate_count):
            if len(payload) < offset + TRACK_GATE.size:
                break
            values = TRACK_GATE.unpack_from(payload, offset)
            gates.append(
                {
                    "gate_id": values[0],
                    "position_ned": list(values[1:4]),
                    "orientation_ned_wxyz": list(values[4:8]),
                    "width": values[8],
                    "height": values[9],
                }
            )
            offset += TRACK_GATE.size
        with self.state.lock:
            self.state.track["gates"] = gates


class VisionReceiver(threading.Thread):
    def __init__(self, state, address):
        super().__init__(name="aigp-vision", daemon=True)
        self.state = state
        self.address = address

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        frames = {}
        try:
            sock.bind(self.address)
            while True:
                packet, _ = sock.recvfrom(65536)
                if len(packet) < VISION_HEADER.size:
                    continue
                frame_id, chunk_id, total_chunks, jpeg_size, payload_size, sim_time_ns = (
                    VISION_HEADER.unpack_from(packet)
                )
                payload = packet[VISION_HEADER.size :]
                frame = frames.setdefault(
                    frame_id,
                    {
                        "chunks": {},
                        "total": total_chunks,
                        "size": jpeg_size,
                        "sim_time_ns": sim_time_ns,
                        "created": time.monotonic(),
                    },
                )
                frame["chunks"][chunk_id] = payload
                if len(frame["chunks"]) == frame["total"]:
                    if all(index in frame["chunks"] for index in range(frame["total"])):
                        jpeg = b"".join(frame["chunks"][index] for index in range(frame["total"]))
                        if len(jpeg) == frame["size"]:
                            image = cv2.imdecode(
                                np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
                            )
                            if image is not None:
                                self.state.update_frame(
                                    frame_id, frame["sim_time_ns"], jpeg, image
                                )
                    del frames[frame_id]
                cutoff = time.monotonic() - 2.0
                for stale_id in [
                    key for key, value in frames.items() if value["created"] < cutoff
                ]:
                    del frames[stale_id]
        except Exception as exc:
            with self.state.lock:
                self.state.camera_error = repr(exc)
        finally:
            sock.close()


class MonitorHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, handler, state, telemetry, index_path):
        self.state = state
        self.telemetry = telemetry
        self.index_path = index_path
        super().__init__(address, handler)


class Handler(BaseHTTPRequestHandler):
    server_version = "AIGPMonitor/1"

    def do_POST(self):
        if self.path == "/api/sim/stop":
            try:
                self.server.state.stop_backend()
            except Exception as exc:
                self.send_json(
                    HTTPStatus.INTERNAL_SERVER_ERROR,
                    {"error": str(exc)},
                )
                return
            self.send_json(
                HTTPStatus.OK,
                self.server.state.snapshot()["simulator"],
            )
            return

        if self.path == "/api/sim/reset":
            try:
                self.server.state.reset_episode(self.server.telemetry)
            except BackendError as exc:
                self.send_json(HTTPStatus.CONFLICT, {"error": str(exc)})
                return
            except Exception as exc:
                self.send_json(
                    HTTPStatus.INTERNAL_SERVER_ERROR,
                    {"error": str(exc)},
                )
                return
            self.send_json(
                HTTPStatus.ACCEPTED,
                self.server.state.snapshot()["simulator"],
            )
            return

        if self.path != "/api/sim/start":
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.send_json(HTTPStatus.BAD_REQUEST, {"error": "Invalid Content-Length"})
            return
        if content_length > 65536:
            self.send_json(
                HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
                {"error": "Request body is too large"},
            )
            return
        raw = self.rfile.read(content_length) if content_length else b"{}"
        try:
            config = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self.send_json(HTTPStatus.BAD_REQUEST, {"error": "Invalid JSON"})
            return
        if not isinstance(config, dict):
            self.send_json(
                HTTPStatus.BAD_REQUEST,
                {"error": "Request body must be a JSON object"},
            )
            return

        try:
            self.server.state.start_backend(config)
        except BackendError as exc:
            self.send_json(HTTPStatus.CONFLICT, {"error": str(exc)})
            return
        except Exception as exc:
            self.send_json(
                HTTPStatus.INTERNAL_SERVER_ERROR,
                {"error": str(exc)},
            )
            return
        self.send_json(
            HTTPStatus.ACCEPTED,
            self.server.state.snapshot()["simulator"],
        )

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index.html"):
            with open(self.server.index_path, "rb") as handle:
                self.send_bytes("text/html; charset=utf-8", handle.read())
        elif self.path.startswith("/api/state"):
            payload = json.dumps(self.server.state.snapshot(), separators=(",", ":")).encode()
            self.send_bytes("application/json", payload)
        elif self.path.startswith("/api/sim/status"):
            snapshot = self.server.state.snapshot()
            payload = json.dumps(snapshot["simulator"], separators=(",", ":")).encode()
            self.send_bytes("application/json", payload)
        elif self.path.startswith("/camera.jpg") or self.path.startswith("/api/frame.jpg"):
            lifecycle = self.server.state.lifecycle()
            if lifecycle not in {"physics_ready", "running"}:
                self.send_json(
                    HTTPStatus.CONFLICT,
                    {
                        "error": "Camera is not public before physics readiness",
                        "lifecycle": lifecycle,
                    },
                )
                return
            with self.server.state.lock:
                frame = self.server.state.camera_jpeg
            if frame is None:
                self.send_error(HTTPStatus.NOT_FOUND, "No camera frame received")
            else:
                self.send_bytes("image/jpeg", frame)
        elif self.path.startswith("/stream.mjpg"):
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last_frame_id = None
            try:
                while True:
                    with self.server.state.frame_ready:
                        self.server.state.frame_ready.wait(timeout=1.0)
                        frame = self.server.state.camera_jpeg
                        frame_id = self.server.state.camera["frame_id"]
                    if self.server.state.lifecycle() not in {"physics_ready", "running"}:
                        continue
                    if frame is None or frame_id == last_frame_id:
                        continue
                    last_frame_id = frame_id
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(("Content-Length: %d\r\n\r\n" % len(frame)).encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def send_bytes(self, content_type, payload):
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def send_json(self, status, value):
        payload = json.dumps(value, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, _format, *_args):
        return


def parse_args():
    parser = argparse.ArgumentParser(description="Simulator-neutral AI-GP gateway")
    parser.add_argument("--backend", default=os.getenv("AIGP_SIM_BACKEND", "vq1"))
    parser.add_argument("--http-host", default=DEFAULT_HTTP_HOST)
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT)
    parser.add_argument("--mavlink-host", default=DEFAULT_MAVLINK_HOST)
    parser.add_argument("--mavlink-port", type=int, default=DEFAULT_MAVLINK_PORT)
    parser.add_argument("--vision-host", default=DEFAULT_VISION_HOST)
    parser.add_argument("--vision-port", type=int, default=DEFAULT_VISION_PORT)
    return parser.parse_args()


def main():
    args = parse_args()
    root = os.path.dirname(os.path.abspath(__file__))
    try:
        backend = create_backend(args.backend)
    except BackendError as exc:
        raise SystemExit(str(exc)) from exc
    state = MonitorState(backend)
    mavlink_address = f"udpin:{args.mavlink_host}:{args.mavlink_port}"
    vision_address = (args.vision_host, args.vision_port)
    telemetry = TelemetryReceiver(state, mavlink_address)
    telemetry.start()
    VisionReceiver(state, vision_address).start()
    server = MonitorHTTPServer(
        (args.http_host, args.http_port),
        Handler,
        state,
        telemetry,
        os.path.join(root, "web", "index.html"),
    )
    print(f"AI-GP gateway: http://{args.http_host}:{args.http_port}", flush=True)
    print(
        f"Backend {backend.name} • MAVLink UDP {args.mavlink_port} • "
        f"Camera UDP {args.vision_port}",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        state.backend.stop()


if __name__ == "__main__":
    main()
