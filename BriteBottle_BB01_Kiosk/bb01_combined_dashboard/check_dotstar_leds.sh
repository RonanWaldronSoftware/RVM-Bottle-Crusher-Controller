#!/usr/bin/env bash
set -euo pipefail

DEVICE="${1:-/dev/spidev0.0}"

echo "=== DotStar LED SPI check ==="
echo "Expected wiring:"
echo "  Jetson pin 19 MOSI -> LED DI / DATA IN"
echo "  Jetson pin 23 SCLK -> LED CI / CLOCK IN"
echo "  Jetson GND         -> LED GND / PSU -"
echo "  External 5V PSU +  -> LED 5V"
echo

if [ ! -e "$DEVICE" ]; then
  echo "ERROR: $DEVICE does not exist."
  echo "SPI may not be enabled on the Jetson header."
  echo "Check Jetson-IO / pinmux for SPI on pins 19 and 23."
  exit 1
fi

if [ ! -w "$DEVICE" ]; then
  echo "WARNING: $DEVICE exists but is not writable by this user."
  echo "The dashboard normally runs with sudo, so this may still be okay."
fi

echo "$DEVICE exists."

sudo -E env DOTSTAR_SPI_DEVICE="$DEVICE" python3 - <<'PY'
from pathlib import Path
import os
import time

DEVICE = os.environ.get("DOTSTAR_SPI_DEVICE", "/dev/spidev0.0")
TOTAL = 72
BRIGHTNESS = 0xE0 | 12

def frame(rgb, lit):
    r, g, b = rgb
    data = bytearray(b"\x00\x00\x00\x00")
    on = bytes([BRIGHTNESS, b, g, r])
    off = bytes([0xE0, 0, 0, 0])
    for i in range(TOTAL):
        data.extend(on if i < lit else off)
    data.extend(b"\xff" * max(4, (TOTAL + 15) // 16))
    return bytes(data)

with open(DEVICE, "wb", buffering=0) as spi:
    print("Turning first 12 LEDs RED for 1 second")
    spi.write(frame((255, 0, 0), 12))
    time.sleep(1)
    print("Turning first 12 LEDs GREEN for 1 second")
    spi.write(frame((0, 255, 0), 12))
    time.sleep(1)
    print("Turning first 12 LEDs BLUE for 1 second")
    spi.write(frame((0, 0, 255), 12))
    time.sleep(1)
    print("Turning LEDs OFF")
    spi.write(frame((0, 0, 0), 0))
PY
