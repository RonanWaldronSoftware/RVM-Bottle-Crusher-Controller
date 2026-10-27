#!/usr/bin/env bash
set -euo pipefail

echo "=== BOOT ENTRY ==="
grep -n "DEFAULT\|LABEL\|FDT" /boot/extlinux/extlinux.conf || true

echo
echo "=== ACTIVE RUNNING DTS CAMERA SUMMARY ==="
sudo dtc -I fs -O dts /proc/device-tree > /tmp/bb01-running-check.dts 2>/tmp/bb01-running-dtc-warnings.txt || true
if grep -q 'BB01 v30' /tmp/bb01-running-check.dts; then
  echo "v30 marker found in active running DTB."
else
  echo "WARNING: v30 marker not found in active running DTB. You may not have rebooted into the new DTB."
fi

grep -niE 'sensor_model|compatible = "sony,imx|devnode|devname = "imx|rbpcv2_imx219_a@10|rbpcv3_imx477_c@1a|tegra-camera-platform|tegra_sinterface' /tmp/bb01-running-check.dts | head -n 240 || true

echo
echo "=== EXPECTED KEY VALUES ==="
echo "Need to see:"
echo '  compatible = "sony,imx708" in the CAM0 node'
echo '  devname = "imx708 10-001a" for CAM0'
echo '  compatible = "sony,imx708" in the CAM1 node'
echo
echo "Counts in active DTB:"
echo -n 'sony,imx708 compatible count: '
grep -c 'compatible = "sony,imx708"' /tmp/bb01-running-check.dts || true
echo -n 'imx708 devname count: '
grep -c 'devname = "imx708' /tmp/bb01-running-check.dts || true

echo
echo "=== VIDEO DEVICES ==="
ls -l /dev/video* 2>/dev/null || echo "No /dev/video* devices"
v4l2-ctl --list-devices || true

echo
echo "=== LIMITED CAMERA I2C SCAN ==="
echo "Looking for IMX708 at 0x1a on camera mux buses."
for b in 9 10; do
  echo "--- bus $b, address 0x1a only ---"
  sudo i2cdetect -y -r "$b" 0x1a 0x1a || true
done

echo
echo "=== RECENT CAMERA DMESG ==="
sudo dmesg -T | grep -iE 'imx708|imx219|imx477|rbpcv|camera|tegra-cam|camrtc|vi5|csi|nvcsi|probe|fail|error|i2c' | tail -n 160 || true
