from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Any


class BackendError(RuntimeError):
    pass


class SimulatorBackend:
    name = "unknown"

    def start(self, config: dict[str, Any]) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def reset(self, transport: Any) -> None:
        raise NotImplementedError

    def is_alive(self) -> bool:
        raise NotImplementedError

    def is_ready(self, snapshot: dict[str, Any]) -> bool:
        raise NotImplementedError


class VQ1Backend(SimulatorBackend):
    name = "vq1"
    REQUIRED_MARKERS = (
        "step=assign_grid_slot run_ok=true result=true",
        "step=confirm_ready_popup run_ok=true result=true",
        "step=force_start run_ok=true result=true",
        "complete physics_ready=true",
    )

    def __init__(self) -> None:
        core_root = Path(__file__).resolve().parent
        lab_value = os.environ.get("MINIPILOT_VQ1_LAB")
        self.launcher = Path(
            os.environ.get("MINIPILOT_VQ1_LAUNCHER", core_root / "launch_vq1.sh")
        )
        self.lab = Path(lab_value).expanduser() if lab_value else None
        self.ue4ss_log = (
            self.lab / "FlightSim/Binaries/Win64/UE4SS.log"
            if self.lab is not None else None
        )
        self.backend_log = Path(
            os.environ.get("MINIPILOT_VQ1_LOG", "/tmp/minipilot-vq1-backend.log")
        ).expanduser()
        self.process: subprocess.Popen[bytes] | None = None
        self.log_offset = 0
        self.log_prefix = b""

    def start(self, config: dict[str, Any]) -> None:
        if self.is_alive():
            raise BackendError("VQ1 is already active")
        if self.lab is None or self.ue4ss_log is None:
            raise BackendError("MINIPILOT_VQ1_LAB is not set")
        if not self.launcher.is_file():
            raise BackendError(f"VQ1 launcher does not exist: {self.launcher}")

        old_log = self.ue4ss_log.read_bytes() if self.ue4ss_log.exists() else b""
        self.log_offset = len(old_log)
        self.log_prefix = old_log[:256]
        self.backend_log.parent.mkdir(parents=True, exist_ok=True)
        with self.backend_log.open("ab") as output:
            output.write(f"\n--- start {time.time():.6f} ---\n".encode())
            output.flush()
            self.process = subprocess.Popen(
                [str(self.launcher)],
                stdin=subprocess.DEVNULL,
                stdout=output,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )

    def stop(self) -> None:
        if not self.is_alive():
            self.process = None
            return

        assert self.process is not None
        process_group = os.getpgid(self.process.pid)
        os.killpg(process_group, signal.SIGTERM)
        try:
            self.process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process_group, signal.SIGKILL)
            self.process.wait(timeout=3)
        self.process = None

    def reset(self, transport: Any) -> None:
        if not self.is_alive():
            raise BackendError("VQ1 is not active")
        transport.send_command_long(31000)

    def is_alive(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def is_ready(self, snapshot: dict[str, Any]) -> bool:
        if not self.is_alive():
            return False
        monitor = snapshot["monitor"]
        if not monitor["mavlink_live"] or not monitor["camera_live"]:
            return False
        if not snapshot["race"] or self.ue4ss_log is None or not self.ue4ss_log.exists():
            return False

        current_log = self.ue4ss_log.read_bytes()
        same_file_prefix = current_log[:256] == self.log_prefix
        if same_file_prefix and len(current_log) >= self.log_offset:
            recent_bytes = current_log[self.log_offset :]
        else:
            recent_bytes = current_log
        recent_log = recent_bytes.decode("utf-8", errors="replace")
        return all(marker in recent_log for marker in self.REQUIRED_MARKERS)


class VQ2Backend(SimulatorBackend):
    name = "vq2"

    def start(self, config: dict[str, Any]) -> None:
        raise BackendError("VQ2 backend is not configured")

    def stop(self) -> None:
        return

    def reset(self, transport: Any) -> None:
        raise BackendError("VQ2 reset is not configured")

    def is_alive(self) -> bool:
        return False

    def is_ready(self, snapshot: dict[str, Any]) -> bool:
        return False


def create_backend(name: str) -> SimulatorBackend:
    if name == "vq1":
        return VQ1Backend()
    if name == "vq2":
        return VQ2Backend()
    raise BackendError(f"Unsupported simulator backend: {name}")
