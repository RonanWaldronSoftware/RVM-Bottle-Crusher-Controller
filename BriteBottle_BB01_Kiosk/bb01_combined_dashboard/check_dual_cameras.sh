#!/usr/bin/env bash
set -u

find_camera() {
  local needle="$1"
  local device info
  for device in /dev/video*; do
    [[ -e "$device" ]] || continue
    info="$(v4l2-ctl -d "$device" --info 2>/dev/null || true)"
    if grep -qi "$needle" <<<"$info"; then
      printf '%s\n' "$device"
      return 0
    fi
  done
  return 1
}

IMX708="$(find_camera imx708 || true)"
IMX219="$(find_camera imx219 || true)"

[[ -n "$IMX708" ]] || { echo "ERROR: IMX708 V4L2 node not found" >&2; exit 2; }
[[ -n "$IMX219" ]] || { echo "ERROR: IMX219 V4L2 node not found" >&2; exit 2; }

echo "IMX708: $IMX708"
echo "IMX219: $IMX219"
echo "Starting simultaneous 40-frame tests..."

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

v4l2-ctl -d "$IMX708" \
  --set-fmt-video=width=4608,height=2592,pixelformat=RG10 \
  --set-parm=8 \
  --stream-mmap=4 --stream-skip=3 --stream-count=40 --stream-to=/dev/null \
  >"$TMP/imx708.log" 2>&1 &
PID708=$!

v4l2-ctl -d "$IMX219" \
  --set-fmt-video=width=1280,height=720,pixelformat=RG10 \
  --set-parm=10 \
  --stream-mmap=4 --stream-skip=3 --stream-count=40 --stream-to=/dev/null \
  >"$TMP/imx219.log" 2>&1 &
PID219=$!

wait "$PID708"; STATUS708=$?
wait "$PID219"; STATUS219=$?

echo "IMX708 exit status: $STATUS708"
echo "IMX219 exit status: $STATUS219"

if [[ "$STATUS708" -ne 0 ]]; then
  echo "===== IMX708 LOG ====="
  cat "$TMP/imx708.log"
fi
if [[ "$STATUS219" -ne 0 ]]; then
  echo "===== IMX219 LOG ====="
  cat "$TMP/imx219.log"
fi

if [[ "$STATUS708" -eq 0 && "$STATUS219" -eq 0 ]]; then
  echo "PASS: both CSI cameras streamed simultaneously."
  exit 0
fi

echo "FAIL: at least one camera could not stream concurrently." >&2
exit 1
