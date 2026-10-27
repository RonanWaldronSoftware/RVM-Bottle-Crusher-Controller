#!/usr/bin/env bash
set -euo pipefail

BUS="${1:-1}"
ADDR="${2:-0x27}"

python3 - <<PY
import time
try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus

BUS = int("$BUS", 0)
ADDR = int("$ADDR", 0)
IODIRA = 0x00
GPPUA  = 0x0C
GPIOA  = 0x12
MASK   = 0x01

bus = SMBus(BUS)
try:
    iodira = bus.read_byte_data(ADDR, IODIRA)
    bus.write_byte_data(ADDR, IODIRA, iodira | MASK)
    gppua = bus.read_byte_data(ADDR, GPPUA)
    bus.write_byte_data(ADDR, GPPUA, gppua | MASK)

    print(f"Reading Waveshare MCP23017 reed on bus {BUS}, address 0x{ADDR:02X}, PA0/GPA0")
    print("Expected for BB01: reed/flap open = 1, reed/flap closed = 0")
    print("Ctrl+C to stop")
    while True:
        value = 1 if (bus.read_byte_data(ADDR, GPIOA) & MASK) else 0
        print("PA0 =", value)
        time.sleep(0.25)
finally:
    try:
        bus.close()
    except Exception:
        pass
PY
