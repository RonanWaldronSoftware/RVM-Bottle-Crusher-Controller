#!/usr/bin/env bash
set -euo pipefail

echo "Stop the dashboard before running this test."
echo "Start with the flap CLOSED, then open and close it several times."
echo

sudo -E env JETSON_MODEL_NAME="${JETSON_MODEL_NAME:-JETSON_ORIN_NANO}"       python3 - <<'PY'
import time
import Jetson.GPIO as GPIO

PIN = 29
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(PIN, GPIO.IN)

initial = int(GPIO.input(PIN))
open_level = GPIO.LOW if initial == GPIO.HIGH else GPIO.HIGH

print(f"Startup CLOSED level: {initial}")
print(f"Expected OPEN level: {open_level}")
print("Move the flap. Ctrl+C stops the test.")

previous = initial
try:
    while True:
        current = int(GPIO.input(PIN))
        if current != previous:
            state = "OPEN" if current == open_level else "CLOSED"
            print(f"Pin 29 changed: {previous} -> {current} ({state})")
            previous = current
        time.sleep(0.02)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
PY
