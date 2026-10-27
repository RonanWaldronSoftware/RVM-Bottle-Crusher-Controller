#!/usr/bin/env bash
set -euo pipefail

echo "This test toggles Jetson J12 physical pin 31 as the Crydom motor request output."
echo "Disconnect the Crydom/MS6 control wire from pin 31 before measuring the bare GPIO."
echo "Measure red probe on physical pin 31 and black probe on Jetson GND."
echo

sudo -E env JETSON_MODEL_NAME="${JETSON_MODEL_NAME:-JETSON_ORIN_NANO}" python3 -u - <<'PY'
import time
import Jetson.GPIO as GPIO

PIN = 31
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN, GPIO.OUT, initial=GPIO.LOW)

try:
    while True:
        print("PIN 31 HIGH", flush=True)
        GPIO.output(PIN, GPIO.HIGH)
        time.sleep(5)
        print("PIN 31 LOW", flush=True)
        GPIO.output(PIN, GPIO.LOW)
        time.sleep(5)
finally:
    GPIO.output(PIN, GPIO.LOW)
    GPIO.cleanup()
PY
