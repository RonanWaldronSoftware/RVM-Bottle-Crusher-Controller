#!/usr/bin/env bash
set -euo pipefail

find_imx708() {
  local device info
  for device in /dev/video*; do
    [[ -e "$device" ]] || continue
    info="$(v4l2-ctl -d "$device" --info 2>/dev/null || true)"
    if grep -qi 'imx708' <<<"$info"; then
      printf '%s\n' "$device"
      return 0
    fi
  done
  return 1
}

echo "=== Jetson release ==="
cat /etc/nv_tegra_release 2>/dev/null || true
uname -a

echo
echo "=== Active BB01 camera boot entry ==="
grep -nE 'DEFAULT|LABEL BB01DualCam|FDT .*bb01-imx219-imx708' \
  /boot/extlinux/extlinux.conf || true

echo
echo "=== Camera devices ==="
v4l2-ctl --list-devices

IMX708_DEVICE="$(find_imx708 || true)"
if [[ -z "$IMX708_DEVICE" ]]; then
  echo "ERROR: no V4L2 card containing imx708 was found." >&2
  exit 1
fi

echo
echo "IMX708 device: $IMX708_DEVICE"
v4l2-ctl -d "$IMX708_DEVICE" --info
v4l2-ctl -d "$IMX708_DEVICE" --list-formats-ext

echo
echo "=== IMX708 module and live device tree ==="
lsmod | grep '^nv_imx708' || {
  echo "ERROR: nv_imx708 is not loaded." >&2
  exit 1
}
NODE=/sys/firmware/devicetree/base/bus@0/cam_i2cmux/i2c@1/rbpcv3_imx477_c@1a
if [[ -r "$NODE/compatible" ]]; then
  printf 'compatible: '
  tr '\0' ' ' < "$NODE/compatible"
  echo
fi

echo
echo "=== One-frame raw V4L2 stream test ==="
RAW_FILE="$(mktemp --suffix=.rg10)"
trap 'rm -f "$RAW_FILE"' EXIT

timeout 35s v4l2-ctl -d "$IMX708_DEVICE" \
  --set-fmt-video=width=4608,height=2592,pixelformat=RG10 \
  --stream-mmap=4 \
  --stream-count=1 \
  --stream-to="$RAW_FILE"

ACTUAL_BYTES="$(stat -c '%s' "$RAW_FILE")"
EXPECTED_BYTES=$((4608 * 2592 * 2))
echo "Captured bytes: $ACTUAL_BYTES"
echo "Expected bytes: $EXPECTED_BYTES"
if (( ACTUAL_BYTES < EXPECTED_BYTES )); then
  echo "ERROR: incomplete IMX708 frame." >&2
  exit 1
fi

echo
echo "=== Recent camera messages ==="
sudo dmesg | grep -Ei \
  'imx708|imx219|nvcsi|tegra-capture|vi5|corr_err|uncorr_err|timeout' \
  | tail -120 || true

echo
echo "PASS: IMX708 raw V4L2 streaming is working on $IMX708_DEVICE."
