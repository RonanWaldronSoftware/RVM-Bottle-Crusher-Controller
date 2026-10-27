#!/usr/bin/env bash
set -euo pipefail
if [ "${EUID}" -ne 0 ]; then
  echo "Run with sudo: sudo ./restore_bb01_dualcam_boot.sh"
  exit 1
fi
EXTLINUX=/boot/extlinux/extlinux.conf
if [ ! -f "$EXTLINUX" ]; then
  echo "ERROR: $EXTLINUX not found"
  exit 1
fi
cp -a "$EXTLINUX" "/boot/extlinux/extlinux.conf.before-restore-$(date +%Y%m%d-%H%M%S)"
sed -i 's/^DEFAULT .*/DEFAULT BB01DualCam/' "$EXTLINUX"
echo "Restored DEFAULT BB01DualCam. Reboot to return to the previous mixed IMX219+IMX708 DTB."
