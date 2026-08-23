# VQ1 simulator core

This directory starts VQ1 directly in R1 physics. It does not use the simulator menus.

The Python gateway owns the simulator lifecycle and the official UDP ports. The browser is only an HTTP client of that gateway.

## Included files

- `gateway.py`: lifecycle API, MAVLink receiver, camera receiver, and HTTP service.
- `backends.py`: simulator-specific process adapter.
- `launch_vq1.sh`: headless Wine launcher for macOS and Linux.
- `local_network_only.sb`: macOS local-network sandbox profile.
- `ue4ss/DirectVQ1/Scripts/main.lua`: direct R1 startup sequence.
- `ue4ss/mods.txt`: minimal UE4SS mod configuration.
- `web/index.html`: optional browser observer.

Simulator binaries, Unreal assets, UE4SS binaries, Wine, and Python packages are external dependencies.

## Laboratory layout

Set `MINIPILOT_VQ1_LAB` to a directory with this layout:

```text
FlightSim.exe
FlightSim/
  Content/
  Binaries/Win64/
    DCGame-Win64-Shipping.exe
    UE4SS.dll
    UE4SS-settings.ini
    dwmapi.dll
    Mods -> <this directory>/ue4ss
```

The launcher verifies the known VQ1 shipping executable hash before it starts the process.

## Start

Install `opencv-python`, `numpy`, and `pymavlink` in the selected Python environment.

```sh
export MINIPILOT_VQ1_LAB=/path/to/vq1-lab
export MINIPILOT_WINE64=/path/to/wine64
export MINIPILOT_WINE_PREFIX=/path/to/wine-prefix
python openpilot/tools/sim/vq1/core/gateway.py --backend vq1
curl -X POST http://127.0.0.1:8767/api/sim/start
```

Open `http://127.0.0.1:8767/` only when a visual observer is necessary.

## API

- `POST /api/sim/start`
- `POST /api/sim/reset`
- `POST /api/sim/stop`
- `GET /api/sim/status`
- `GET /api/state`
- `GET /api/frame.jpg`
- `GET /stream.mjpg`

The frame endpoints remain unavailable until the lifecycle reaches `physics_ready`.

## Verified VQ1 behavior

Three cold starts reached R1 gate `0` in 11.2 to 12.3 seconds. One reset rewound simulator boot time and returned to gate `0`.
