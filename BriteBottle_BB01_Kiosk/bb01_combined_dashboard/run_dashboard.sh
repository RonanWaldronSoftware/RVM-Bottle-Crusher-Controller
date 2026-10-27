#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

if [[ ! -x .venv/bin/python ]]; then
  echo "Missing .venv. Run ./install.sh first." >&2
  exit 1
fi

export JETSON_MODEL_NAME="${JETSON_MODEL_NAME:-JETSON_ORIN_NANO}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export QT_QPA_PLATFORMTHEME=""

if [[ ${EUID} -eq 0 ]]; then
  exec .venv/bin/python controller_kiosk.py --fullscreen "$@"
fi

# The proven pinmux routine uses devmem and therefore requires root. Preserve
# the desktop display/session values so the root process can create the Qt Widgets UI.
DISPLAY_VALUE="${DISPLAY:-:0}"
XAUTH_VALUE="${XAUTHORITY:-$HOME/.Xauthority}"
USER_HOME="$HOME"
RUNTIME_VALUE="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
WAYLAND_VALUE="${WAYLAND_DISPLAY:-}"

if command -v xhost >/dev/null 2>&1; then
  xhost +SI:localuser:root >/dev/null 2>&1 || true
fi

exec sudo -E env \
  HOME="$USER_HOME" \
  DISPLAY="$DISPLAY_VALUE" \
  XAUTHORITY="$XAUTH_VALUE" \
  XDG_RUNTIME_DIR="$RUNTIME_VALUE" \
  WAYLAND_DISPLAY="$WAYLAND_VALUE" \
  JETSON_MODEL_NAME="$JETSON_MODEL_NAME" \
  QT_QPA_PLATFORM="$QT_QPA_PLATFORM" \
  QT_QPA_PLATFORMTHEME="" \
  "$PWD/.venv/bin/python" "$PWD/controller_kiosk.py" --fullscreen "$@"
