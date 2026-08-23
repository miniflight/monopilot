#!/bin/zsh
set -eu

readonly CORE_DIR="${0:A:h}"
readonly LAB_DIR="${MINIPILOT_VQ1_LAB:?Set MINIPILOT_VQ1_LAB to the unpacked VQ1 laboratory}"
readonly WINE_HOST="${MINIPILOT_WINE64:-wine64}"
readonly WINE_PREFIX="${MINIPILOT_WINE_PREFIX:-$LAB_DIR/.wine-prefix}"
readonly NETWORK_PROFILE="$CORE_DIR/local_network_only.sb"
readonly SHIPPING_SHA256="507a7b5444e8e715b1a210a62b8731284015cce4f4f6878adf1825ca4e40d567"
readonly ARENA_URL="/Game/levelsMaster/MAP_anduril_master?game=/Script/DCGame.GameModeRaceBase"

shipping="$LAB_DIR/FlightSim/Binaries/Win64/DCGame-Win64-Shipping.exe"
mod="$LAB_DIR/FlightSim/Binaries/Win64/Mods/DirectVQ1/Scripts/main.lua"

[[ -f "$shipping" ]] || { print -u2 -- "missing VQ1 shipping binary"; exit 1; }
[[ -f "$mod" ]] || { print -u2 -- "missing DirectVQ1 mod"; exit 1; }
[[ "$(shasum -a 256 "$shipping" | awk '{print $1}')" == "$SHIPPING_SHA256" ]] || {
  print -u2 -- "VQ1 shipping binary identity mismatch"
  exit 1
}

owners="$(ps ax -o pid=,ppid=,command= | rg '[w]ine64-preloader .*[\\/]DCGame-Win64-Shipping\.exe' || true)"
if [[ -n "$owners" ]]; then
  print -u2 -- "$owners"
  print -u2 -- "another simulator owns the Wine lane"
  exit 1
fi

export WINEPREFIX="$WINE_PREFIX"
export WINE_DO_NOT_CREATE_DXGI_DEVICE_MANAGER=1
export WINEDLLOVERRIDES="dwmapi=n,b;winegstreamer="

cd "$LAB_DIR"
args=(
  "$WINE_HOST" "$shipping" "$ARENA_URL"
  -RenderOffscreen -Unattended -ResX=640 -ResY=360 -nosound -NoSplash "$@"
)
if [[ "$(uname -s)" == Darwin && -x /usr/bin/sandbox-exec ]]; then
  exec /usr/bin/sandbox-exec -f "$NETWORK_PROFILE" "${args[@]}"
fi
exec "${args[@]}"
