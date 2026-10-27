#!/usr/bin/env bash
set -u
OUT="bb01_camera_debug_$(date +%Y%m%d_%H%M%S).txt"
{
  echo "=== Jetson release ==="
  cat /etc/nv_tegra_release 2>/dev/null || true
  uname -a
  echo
  echo "=== extlinux ==="
  cat /boot/extlinux/extlinux.conf 2>/dev/null || true
  echo
  echo "=== cmdline ==="
  cat /proc/cmdline 2>/dev/null || true
  echo
  echo "=== /dev/video ==="
  ls -l /dev/video* 2>/dev/null || echo "No /dev/video* devices"
  v4l2-ctl --list-devices 2>&1 || true
  echo
  echo "=== V4L2 per device ==="
  for d in /dev/video*; do
    [[ -e "$d" ]] || continue
    echo "----- $d info -----"
    v4l2-ctl -d "$d" --info 2>&1 || true
    echo "----- $d formats -----"
    v4l2-ctl -d "$d" --list-formats-ext 2>&1 | sed -n '1,160p' || true
  done
  echo
  echo "=== device-tree camera compatible nodes ==="
  sudo bash -c '
  for f in $(find /proc/device-tree -name compatible); do
    txt=$(tr "\0" "\n" < "$f")
    echo "$txt" | grep -qiE "imx708|imx477|rbpcv3|imx219|tegra-camera" && {
      echo
      echo "FILE: $f"
      echo "$txt"
    }
  done
  ' 2>&1 || true
  echo
  echo "=== dmesg camera tail ==="
  sudo dmesg -T 2>/dev/null | grep -iE "imx708|imx219|imx477|rbpcv3|camera|tegra-cam|camrtc|vi5|csi|nvcsi|probe|fail|error|i2c" | tail -n 260 || true
} | tee "$OUT"
echo "Wrote $OUT"
