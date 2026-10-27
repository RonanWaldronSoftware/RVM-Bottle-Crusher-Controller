#!/usr/bin/env python3
import time
import Jetson.GPIO as GPIO

PWM_PIN = 32
DIR_PIN = 33
PULSE_SECONDS = 0.6
UNLOCK_DIR = GPIO.LOW
LOCK_DIR = GPIO.HIGH

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PWM_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(DIR_PIN, GPIO.OUT, initial=UNLOCK_DIR)

def pulse(name, direction):
    print(f"{name}: DIR={direction}, PWM pulse {PULSE_SECONDS:.1f}s")
    GPIO.output(PWM_PIN, GPIO.LOW)
    GPIO.output(DIR_PIN, direction)
    time.sleep(0.02)
    GPIO.output(PWM_PIN, GPIO.HIGH)
    time.sleep(PULSE_SECONDS)
    GPIO.output(PWM_PIN, GPIO.LOW)
    print(f"{name}: MD13S PWM OFF")

try:
    while True:
        input("Press Enter to UNLOCK...")
        pulse("UNLOCK", UNLOCK_DIR)
        input("Press Enter to LOCK...")
        pulse("LOCK", LOCK_DIR)
finally:
    GPIO.output(PWM_PIN, GPIO.LOW)
    GPIO.cleanup()
