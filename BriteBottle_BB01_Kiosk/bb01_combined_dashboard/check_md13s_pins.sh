#!/usr/bin/env bash
set -euo pipefail

sudo -E env JETSON_MODEL_NAME="${JETSON_MODEL_NAME:-JETSON_ORIN_NANO}"   python3 -u - <<'PY'
import time
import Jetson.GPIO as GPIO

PWM_PIN = 32
DIR_PIN = 33

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PWM_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(DIR_PIN, GPIO.OUT, initial=GPIO.LOW)

print("Measure pin 32 and pin 33 to Jetson GND. Ctrl+C to stop.")
try:
    while True:
        print("DIR LOW, PWM HIGH for 3 seconds")
        GPIO.output(DIR_PIN, GPIO.LOW)
        GPIO.output(PWM_PIN, GPIO.HIGH)
        time.sleep(3)
        print("PWM LOW for 2 seconds")
        GPIO.output(PWM_PIN, GPIO.LOW)
        time.sleep(2)
        print("DIR HIGH, PWM HIGH for 3 seconds")
        GPIO.output(DIR_PIN, GPIO.HIGH)
        GPIO.output(PWM_PIN, GPIO.HIGH)
        time.sleep(3)
        print("PWM LOW for 2 seconds")
        GPIO.output(PWM_PIN, GPIO.LOW)
        time.sleep(2)
finally:
    GPIO.output(PWM_PIN, GPIO.LOW)
    GPIO.output(DIR_PIN, GPIO.LOW)
    GPIO.cleanup()
PY
