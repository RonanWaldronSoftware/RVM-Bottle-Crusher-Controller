#!/usr/bin/env bash
set -u

echo "=== BB01 direct dual IMX708 check ==="
echo "Expected physical wiring: CAM0 = IMX708, CAM1 = IMX708, no UC-475."
echo

echo "=== Active boot entry ==="
grep -n "DEFAULT\|LABEL" /boot/extlinux/extlinux.conf 2>/dev/null || true

echo
echo "=== Device-tree camera declarations ==="
sudo bash -c '
for f in $(find /proc/device-tree -name compatible); do
  txt=$(tr "\0" "\n" < "$f")
  echo "$txt" | grep -qiE "imx708|imx477|rbpcv3|imx219" && {
    echo
    echo "FILE: $f"
    echo "$txt"
  }
done
' 2>/dev/null || true

echo
echo "=== V4L2 devices ==="
ls -l /dev/video* 2>/dev/null || echo "No /dev/video* devices"
v4l2-ctl --list-devices 2>/dev/null || true

echo
echo "=== Candidate video nodes ==="
mapfile -t DEVICES < <(for d in /dev/video*; do [[ -e "$d" ]] && echo "$d"; done | sort -V)
if [[ ${#DEVICES[@]} -lt 2 ]]; then
  echo "ERROR: fewer than two /dev/video* nodes are present." >&2
  echo "Linux must detect both IMX708 cameras before the dashboard can scan." >&2
  exit 2
fi

for d in "${DEVICES[@]}"; do
  echo "----- $d -----"
  v4l2-ctl -d "$d" --info 2>/dev/null || true
  v4l2-ctl -d "$d" --list-formats-ext 2>/dev/null | sed -n '1,80p' || true
done

echo
echo "=== Parallel raw stream smoke test ==="
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

D0="${DEVICES[0]}"
D1="${DEVICES[1]}"
echo "Testing $D0 and $D1 simultaneously as IMX708 raw RG10 4608x2592 @ 8 fps"

v4l2-ctl -d "$D0" \
  --set-fmt-video=width=4608,height=2592,pixelformat=RG10 \
  --set-parm=8 \
  --stream-mmap=4 --stream-skip=3 --stream-count=20 --stream-to=/dev/null \
  >"$TMP/cam0.log" 2>&1 &
PID0=$!

v4l2-ctl -d "$D1" \
  --set-fmt-video=width=4608,height=2592,pixelformat=RG10 \
  --set-parm=8 \
  --stream-mmap=4 --stream-skip=3 --stream-count=20 --stream-to=/dev/null \
  >"$TMP/cam1.log" 2>&1 &
PID1=$!

wait "$PID0"; S0=$?
wait "$PID1"; S1=$?

echo "$D0 exit status: $S0"
echo "$D1 exit status: $S1"

if [[ "$S0" -ne 0 ]]; then
  echo "===== $D0 log ====="
  cat "$TMP/cam0.log"
fi
if [[ "$S1" -ne 0 ]]; then
  echo "===== $D1 log ====="
  cat "$TMP/cam1.log"
fi

if [[ "$S0" -eq 0 && "$S1" -eq 0 ]]; then
  echo
  echo "PASS: two independent V4L2 cameras streamed simultaneously."
  echo "You can now run: ./run_dashboard.sh --no-camera-autostart"
  exit 0
fi

echo "FAIL: one or both cameras did not stream." >&2
exit 1
