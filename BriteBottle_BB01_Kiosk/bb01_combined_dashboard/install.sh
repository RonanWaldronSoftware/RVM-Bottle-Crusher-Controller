#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

if [[ ! -f /etc/nv_tegra_release ]]; then
  echo "WARNING: This package is intended for NVIDIA Jetson/JetPack." >&2
fi

sudo apt update
sudo apt install -y \
  busybox \
  i2c-tools \
  libxcb-cursor0 \
  python3-dev \
  python3-numpy \
  python3-opencv \
  python3-pip \
  python3-smbus \
  python3-venv \
  v4l-utils \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly

# Always build the virtual environment locally on the Jetson.
# Prebuilt virtual environments are not portable between x86_64 and aarch64.
rm -rf .venv
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt

# Jetson.GPIO is often preinstalled by JetPack. Install it only if the venv
# cannot already import the vendor/system copy.
if ! .venv/bin/python -c 'import Jetson.GPIO' >/dev/null 2>&1; then
  .venv/bin/python -m pip install Jetson.GPIO
fi

[[ -f barcode_products.csv ]] || cp barcode_products.example.csv barcode_products.csv
[[ -f product_options.csv ]] || cp product_options.example.csv product_options.csv
[[ -f bottle_transactions.csv ]] || touch bottle_transactions.csv
chmod +x ./*.sh ./*.py

cat <<'MSG'

Installation complete.

The dashboard is installed for the BB01DualCam configuration:
  CAM1 IMX708: auto-detected by V4L2 card name
  CAM0 IMX219: concurrent raw-V4L2 barcode scanner
  First-camera-wins scan interlock with flap lock/reed cycle control

This package does not replace the kernel module or combined DTB already installed.
Run ./check_imx708.sh and ./check_dual_cameras.sh before starting the hardware dashboard.
MSG
