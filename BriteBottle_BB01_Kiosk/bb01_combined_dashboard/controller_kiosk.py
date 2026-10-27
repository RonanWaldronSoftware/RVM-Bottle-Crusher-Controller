#!/usr/bin/env python3
"""
BB01 Jetson Orin Nano combined machine and barcode dashboard.

Integrated hardware:
- Crydom SSR motor request on physical BOARD pin 31. HIGH = SSR input ON,
  LOW = SSR input OFF. The MS6 door magnet switch should be wired in series
  with the Crydom input as a hard motor-enable interlock.
- IBT-2 / BTS7960 latching-solenoid driver:
    physical BOARD pin 31 -> RPWM / unlock direction
    physical BOARD pin 32 -> LPWM / lock direction
  Each command energises one direction for 600 ms and then switches both
  outputs LOW. No GPIO enable pin and no cooldown/lockout are used.
- Flap reed switch on Waveshare MCP23017 I2C expander: Jetson I2C bus 1,
  address 0x27, PA0/GPA0. PA0=1 means flap OPEN and PA0=0 means
  flap CLOSED. A valid scan arms exactly one flap-open motor trigger; idle
  flap movement cannot start the crusher.
- Two DFRobot A02YYUW ultrasonic sensors through a DFRobot DFR0627
  I2C-to-dual-UART module on I2C bus 7.
- Adafruit DotStar / APA102-compatible RGB LED strip control over SPI
  using Jetson pin 19 MOSI and pin 23 SCLK.
- Qt Widgets kiosk dashboard (no QML/Qt Quick engine).
- Dual independent IMX708 raw V4L2 barcode scanning through ZXing-C++.
- Concurrent CAM0/CAM1 barcode scanning and preview.
- CSV product lookup and touch assignment for unknown barcodes.

Each camera scanner runs in a subprocess so a camera failure cannot stop the
machine-control timers or safe-output cleanup. The GUI uses Qt Widgets to avoid
Jetson/aarch64 Qt Quick engine crashes observed with PySide6 6.8.

Hardware run:
    sudo -E python3 controller_kiosk.py --fullscreen

Simulation run:
    python3 controller_kiosk.py --simulate

Dependencies:
    sudo apt install -y python3-smbus i2c-tools libxcb-cursor0
    python3 -m pip install PySide6 smbus2
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import queue
import random
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional


# =============================================================================
# OPTIONAL I2C IMPORT
# =============================================================================

SMBus = None
try:
    from smbus2 import SMBus as SMBus2

    SMBus = SMBus2
except ImportError:
    try:
        from smbus import SMBus as SMBus1

        SMBus = SMBus1
    except ImportError:
        SMBus = None


# =============================================================================
# GPIO LOADER / SIMULATION
# =============================================================================


class SimulatedGPIO:
    BOARD = "BOARD"
    OUT = "OUT"
    IN = "IN"
    HIGH = 1
    LOW = 0

    def __init__(self):
        self._pins: dict[int, int] = {}

    def setwarnings(self, _flag: bool):
        pass

    def setmode(self, _mode):
        pass

    def setup(self, pin: int, _mode, initial=None):
        self._pins[pin] = self.LOW if initial is None else initial

    def input(self, pin: int):
        return self._pins.get(pin, self.LOW)

    def output(self, pin: int, value):
        self._pins[pin] = value

    def cleanup(self):
        pass

    def simulate_set_pin(self, pin: int, value: int):
        self._pins[pin] = value


GPIO = None


def load_gpio(simulate: bool):
    global GPIO

    if simulate:
        GPIO = SimulatedGPIO()
        return

    os.environ.setdefault("JETSON_MODEL_NAME", "JETSON_ORIN_NANO")

    try:
        import Jetson.GPIO as jetson_gpio
    except ImportError as exc:
        raise SystemExit(
            "Jetson.GPIO is missing. Run on the Jetson or use --simulate."
        ) from exc
    except Exception as exc:
        raise SystemExit(
            "Jetson.GPIO failed to initialise. Try:\n"
            "  sudo -E env JETSON_MODEL_NAME=JETSON_ORIN_NANO "
            "python3 controller_kiosk.py --fullscreen\n\n"
            f"Original error: {exc}"
        ) from exc

    GPIO = jetson_gpio


# =============================================================================
# PHYSICAL BOARD PINS — JETSON ORIN NANO J12
# =============================================================================

# Crydom / DFR0457 motor-request control. Physical BOARD pin 31.
# Wiring: pin 31 -> DFR0457 Signal/S; DFR0457 output drives Crydom input.
PIN_MOTOR_SSR = 31

# MD13S latching-solenoid driver. Physical BOARD numbering.
# MD13S PWM receives the short energising pulse. MD13S DIR selects lock/unlock.
PIN_SOL_PWM = 32
PIN_SOL_DIR = 33
PIN_SOL_NC = None

# Set these after physical testing if lock/unlock are reversed.
SOLENOID_UNLOCK_DIR_VALUE = 0
SOLENOID_LOCK_DIR_VALUE = 1

# Simulation-only reed input. In real hardware the flap reed is read from
# the Waveshare MCP23017 I2C expander on PA0/GPA0, not from a Jetson GPIO pin.
PIN_REED = 29

# Jetson Orin Nano pad-control registers.
# Values are calculated from each register's current value so pull-up/down and
# other unrelated pad settings are preserved.
PINMUX_BUSYBOX = "/usr/bin/busybox"
# Verified pinmux path for the motor request output on pin 31.
# Pin 32/33 are configured through Jetson.GPIO and should be verified with
# check_md13s_pins.sh before connecting the solenoid.
PINMUX_SOLENOID_REGISTERS = (
    ("physical pin 31 / motor request output", "0x2430070"),
)
# Reed input is now on MCP23017 PA0, so no Jetson GPIO reed pinmux is needed.
PINMUX_REED_REGISTER = None
PINMUX_IO_LOCK = threading.Lock()

# Orin PADCTL fields used by Jetson.GPIO's own pinmux checks.
PINMUX_GPIO_SFIO_BIT = 1 << 10
PINMUX_INPUT_ENABLE_BIT = 1 << 6
PINMUX_TRISTATE_BIT = 1 << 4
PINMUX_DIRECTION_MASK = PINMUX_INPUT_ENABLE_BIT | PINMUX_TRISTATE_BIT


def _read_pinmux_register(register: str) -> int:
    """Read one Orin pad-control register through BusyBox devmem."""
    result = subprocess.run(
        [PINMUX_BUSYBOX, "devmem", register],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    output = result.stdout.strip().splitlines()
    if not output:
        raise RuntimeError(f"No readback returned for pinmux register {register}")
    try:
        return int(output[-1].strip(), 0)
    except ValueError as exc:
        raise RuntimeError(
            f"Unexpected readback for pinmux register {register}: {output[-1]!r}"
        ) from exc


def _write_pinmux_register(register: str, value: int) -> None:
    """Write one Orin pad-control register through BusyBox devmem."""
    try:
        subprocess.run(
            [PINMUX_BUSYBOX, "devmem", register, "w", hex(value)],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
    except subprocess.CalledProcessError as exc:
        detail = (exc.stderr or str(exc)).strip()
        raise RuntimeError(
            f"Failed to write pinmux register {register}: {detail}"
        ) from exc


def _gpio_output_value(current: int) -> int:
    """Return PADCTL value for GPIO output while preserving unrelated bits."""
    return current & ~(PINMUX_GPIO_SFIO_BIT | PINMUX_DIRECTION_MASK)


def _gpio_input_value(current: int) -> int:
    """Return PADCTL value for GPIO input while preserving unrelated bits."""
    value = current & ~PINMUX_GPIO_SFIO_BIT
    return value | PINMUX_DIRECTION_MASK


def configure_solenoid_pinmux(
    simulate: bool = False,
    reason: str = "startup",
) -> None:
    """Force and verify pin 31 as a GPIO output.

    Pin 31 is the DFR0457/Crydom motor request. Pin 32/33 are MD13S solenoid
    logic outputs and are configured by Jetson.GPIO at setup. The flap reed is
    read through the MCP23017 I2C expander, so no Jetson reed GPIO pinmux is
    configured here.
    """
    if simulate:
        return

    if os.geteuid() != 0:
        raise PermissionError(
            "GPIO pinmux setup requires root. Run with: "
            "sudo -E python3 controller_kiosk.py --fullscreen"
        )

    if not Path(PINMUX_BUSYBOX).exists():
        raise FileNotFoundError(
            f"BusyBox was not found at {PINMUX_BUSYBOX}; cannot configure "
            "GPIO pinmux."
        )

    with PINMUX_IO_LOCK:
        last_values: dict[str, int] = {}
        targets: dict[str, int] = {}

        for attempt in range(1, 4):
            targets = {}

            for _label, register in PINMUX_SOLENOID_REGISTERS:
                current = _read_pinmux_register(register)
                target = _gpio_output_value(current)
                targets[register] = target
                _write_pinmux_register(register, target)

            if PINMUX_REED_REGISTER is not None:
                reed_label, reed_register = PINMUX_REED_REGISTER
                reed_current = _read_pinmux_register(reed_register)
                reed_target = _gpio_input_value(reed_current)
                targets[reed_register] = reed_target
                _write_pinmux_register(reed_register, reed_target)

            time.sleep(0.005)

            all_registers = tuple(PINMUX_SOLENOID_REGISTERS)
            if PINMUX_REED_REGISTER is not None:
                all_registers = (*all_registers, PINMUX_REED_REGISTER)
            last_values = {
                register: _read_pinmux_register(register)
                for _label, register in all_registers
            }

            if all(
                last_values[register] == targets[register]
                for _label, register in all_registers
            ):
                values = ", ".join(
                    f"{label}=0x{last_values[register]:08X}"
                    for label, register in all_registers
                )
                print(f"[GPIO] Pinmux verified ({reason}): {values}")
                return

            print(
                f"[GPIO] Pinmux verification attempt {attempt}/3 failed "
                f"({reason}); retrying"
            )

        details = ", ".join(
            f"{label}: read 0x{last_values.get(register, 0):08X}, "
            f"expected 0x{targets.get(register, 0):08X}"
            for label, register in (
                *PINMUX_SOLENOID_REGISTERS,
                *((PINMUX_REED_REGISTER,) if PINMUX_REED_REGISTER is not None else ()),
            )
        )
        raise RuntimeError(
            f"GPIO pinmux did not remain configured ({reason}): {details}"
        )


# =============================================================================
# MACHINE CONFIGURATION
# =============================================================================

MOTOR_RUN_SECONDS = 3.0
DEBOUNCE_SECONDS = 0.08
MACHINE_TICK_MS = 20
GUI_REFRESH_MS = 200

# Crydom SSR input logic.
# HIGH energises the SSR input; LOW de-energises it.
MOTOR_ON_LEVEL_VALUE = 1
MOTOR_OFF_LEVEL_VALUE = 0

# Reed input through Waveshare MCP23017.
# Hardware wiring: PA0/GPA0 -> reed -> MCP23017 GND.
# MCP23017 internal pull-up is enabled in software.
# Required behaviour from physical test: PA0=1 means flap OPEN, PA0=0 means flap CLOSED.
MCP_REED_I2C_BUS = 1
MCP_REED_I2C_ADDR = 0x27
MCP23017_IODIRA = 0x00
MCP23017_GPPUA = 0x0C
MCP23017_GPIOA = 0x12
MCP_REED_PA0_MASK = 0x01
FLAP_CLOSED_LEVEL_VALUE = 0
FLAP_OPEN_LEVEL_VALUE = 1

# Verified solenoid energisation time from solenoid_test.py.
SOLENOID_PULSE_SECONDS = 0.600

# DFRobot DFR0627 and A02YYUW configuration.
I2C_BUS_NUMBER = 7
IIC_UART_IA1 = 1
IIC_UART_IA0 = 1
A02YYUW_BAUD = 9600

# Adjust after physical calibration.
BOX_PRESENT_DISTANCE_MM = 400
EMPTY_BOX_DISTANCE_MM = 600
FULL_BOX_DISTANCE_MM = 100

# Serialise DFR0627 transactions from both reader threads.
I2C_IO_LOCK = threading.RLock()


# DotStar / APA102 LED strip configuration.
# Wiring: Jetson pin 19 MOSI -> LED DI, pin 23 SCLK -> LED CI,
# Jetson GND -> LED PSU/strip GND. Use external 5 V PSU for the strip.
DOTSTAR_SPI_DEVICE_DEFAULT = "/dev/spidev0.0"
DOTSTAR_TOTAL_LEDS_DEFAULT = 72
DOTSTAR_ACTIVE_LEDS_DEFAULT = 72
DOTSTAR_BRIGHTNESS_DEFAULT = 12  # DotStar global brightness, 0..31.
DOTSTAR_MAX_LEDS = 300
DOTSTAR_LAST_COLOR_DEFAULT = (0, 0, 0)  # RGB




# =============================================================================
# DOTSTAR / APA102 LED STRIP DRIVER
# =============================================================================


class DotStarLedStrip:
    """Small dependency-free DotStar/APA102 LED driver using Linux spidev.

    DotStar LED frame order is:
        start frame, then one frame per LED: 0b111xxxxx, blue, green, red,
        then an end frame.

    The hardware strip is 5 V, but the Jetson SPI lines are 3.3 V. For robust
    wiring use a 74AHCT125/74AHCT245 level shifter on DATA and CLOCK.
    """

    def __init__(
        self,
        spi_device: str,
        total_leds: int = DOTSTAR_TOTAL_LEDS_DEFAULT,
        active_leds: int = DOTSTAR_ACTIVE_LEDS_DEFAULT,
        brightness: int = DOTSTAR_BRIGHTNESS_DEFAULT,
        simulate: bool = False,
    ):
        self.spi_device = str(spi_device)
        self.simulate = bool(simulate)
        self.total_leds = self._clamp_count(total_leds, minimum=1)
        self.active_leds = max(0, min(self._clamp_count(active_leds), self.total_leds))
        self.brightness = self._clamp_brightness(brightness)
        self.color_rgb = DOTSTAR_LAST_COLOR_DEFAULT
        self.error: Optional[str] = None
        self.available = bool(simulate)
        self._handle = None

        if self.simulate:
            return

        try:
            # A plain write to /dev/spidevX.Y is enough for APA102/DotStar.
            # SPI speed/mode should be normal mode 0; Jetson defaults are
            # typically fine for this simple stream.
            self._handle = open(self.spi_device, "wb", buffering=0)
            self.available = True
        except OSError as exc:
            self.error = f"{self.spi_device}: {exc}"
            self.available = False

    @staticmethod
    def _clamp_count(value: int, minimum: int = 0) -> int:
        try:
            count = int(value)
        except (TypeError, ValueError):
            count = minimum
        return max(minimum, min(DOTSTAR_MAX_LEDS, count))

    @staticmethod
    def _clamp_brightness(value: int) -> int:
        try:
            brightness = int(value)
        except (TypeError, ValueError):
            brightness = DOTSTAR_BRIGHTNESS_DEFAULT
        return max(0, min(31, brightness))

    @staticmethod
    def _clamp_color(value: int) -> int:
        try:
            channel = int(value)
        except (TypeError, ValueError):
            channel = 0
        return max(0, min(255, channel))

    def set_total_leds(self, total_leds: int):
        self.total_leds = self._clamp_count(total_leds, minimum=1)
        self.active_leds = max(0, min(self.active_leds, self.total_leds))
        self._write_current_frame()

    def set_active_leds(self, active_leds: int):
        self.active_leds = max(0, min(self._clamp_count(active_leds), self.total_leds))
        self._write_current_frame()

    def set_brightness(self, brightness: int):
        self.brightness = self._clamp_brightness(brightness)
        self._write_current_frame()

    def set_color(self, red: int, green: int, blue: int, active_leds: Optional[int] = None):
        if active_leds is not None:
            self.set_active_leds(active_leds)
        self.color_rgb = (
            self._clamp_color(red),
            self._clamp_color(green),
            self._clamp_color(blue),
        )
        self._write_current_frame()

    def off(self):
        # Keep the configured active LED count, but send black to the entire
        # strip so no previously-lit LEDs remain on.
        self.color_rgb = (0, 0, 0)
        self._write_current_frame(force_all_off=True)

    def _build_frame(self, force_all_off: bool = False) -> bytes:
        red, green, blue = self.color_rgb
        brightness_byte = 0xE0 | self.brightness
        lit_count = 0 if force_all_off else self.active_leds
        off_frame = bytes([0xE0, 0, 0, 0])
        on_frame = bytes([brightness_byte, blue, green, red])

        data = bytearray()
        data.extend(b"\x00\x00\x00\x00")
        for index in range(self.total_leds):
            data.extend(on_frame if index < lit_count else off_frame)

        # End frame: enough clocks to flush data through the strip.
        data.extend(b"\xFF" * max(4, (self.total_leds + 15) // 16))
        return bytes(data)

    def _write_current_frame(self, force_all_off: bool = False):
        frame = self._build_frame(force_all_off=force_all_off)

        if self.simulate:
            self.available = True
            self.error = None
            return

        if not self.available or self._handle is None:
            self.error = self.error or f"{self.spi_device} is not open"
            return

        try:
            self._handle.write(frame)
            try:
                self._handle.flush()
            except OSError:
                pass
            self.error = None
        except OSError as exc:
            self.error = f"{self.spi_device}: {exc}"
            self.available = False

    def close(self):
        if self._handle is not None:
            try:
                self.off()
            except Exception:
                pass
            try:
                self._handle.close()
            except OSError:
                pass
            self._handle = None


# =============================================================================
# DFR0627 I2C-TO-DUAL-UART DRIVER
# =============================================================================


class DFR0627Channel:
    """Minimal DFRobot DFR0627 driver for one UART channel."""

    REG_GENA = 0x00
    REG_GRST = 0x01
    REG_GIER = 0x10
    REG_SPAGE = 0x03

    REG_SCR = 0x04
    REG_LCR = 0x05
    REG_FCR = 0x06
    REG_SIER = 0x07
    REG_RFCNT = 0x0A
    REG_FSR = 0x0B
    REG_FDAT = 0x0D

    REG_BAUD1 = 0x04
    REG_BAUD0 = 0x05
    REG_PRES = 0x06

    FOSC = 14_745_600
    OBJECT_REGISTER = 0

    def __init__(self, bus, channel: int, ia1: int = 1, ia0: int = 1):
        if channel not in (1, 2):
            raise ValueError("DFR0627 channel must be 1 or 2")
        self.bus = bus
        self.channel = channel
        self.ia1 = ia1
        self.ia0 = ia0
        self.sub_uart = channel - 1

    def _addr(self, channel_override: Optional[int] = None) -> int:
        sub_uart = self.sub_uart if channel_override is None else channel_override
        return (
            (self.ia1 << 6)
            | (self.ia0 << 5)
            | 0x10
            | ((sub_uart & 0x03) << 1)
            | self.OBJECT_REGISTER
        )

    def _read_reg(
        self,
        reg: int,
        size: int = 1,
        channel_override: Optional[int] = None,
    ) -> list[int]:
        addr = self._addr(channel_override)
        try:
            with I2C_IO_LOCK:
                return list(self.bus.read_i2c_block_data(addr, reg, size))
        except OSError as exc:
            raise OSError(
                f"I2C read failed addr=0x{addr:02X} reg=0x{reg:02X}: {exc}"
            ) from exc

    def _write_reg(
        self,
        reg: int,
        values,
        channel_override: Optional[int] = None,
    ):
        addr = self._addr(channel_override)
        if isinstance(values, int):
            values = [values & 0xFF]
        payload = [value & 0xFF for value in values]
        try:
            with I2C_IO_LOCK:
                self.bus.write_i2c_block_data(addr, reg, payload)
        except OSError as exc:
            raise OSError(
                f"I2C write failed addr=0x{addr:02X} reg=0x{reg:02X}: {exc}"
            ) from exc

    def _read_fifo_byte(self) -> int:
        # This FDAT register method is the version already proven on the Jetson.
        return self._read_reg(self.REG_FDAT, 1)[0] & 0xFF

    def _set_page(self, page: int):
        value = self._read_reg(self.REG_SPAGE, 1)[0]
        value = (value & 0xFE) if page == 0 else (value | 0x01)
        self._write_reg(self.REG_SPAGE, value)

    def _enable_global_bit(self, reg: int):
        # Global registers are accessed through UART1 register address.
        value = self._read_reg(reg, 1, channel_override=0)[0]
        value |= 0x01 if self.channel == 1 else 0x02
        self._write_reg(reg, value, channel_override=0)

    def begin(self, baud: int = 9600):
        # Confirm module responds.
        self._read_reg(self.REG_GENA, 1, channel_override=0)

        self._enable_global_bit(self.REG_GENA)
        self._enable_global_bit(self.REG_GRST)
        self._enable_global_bit(self.REG_GIER)

        self._set_page(0)
        self._write_reg(self.REG_SIER, 0x8F)
        self._write_reg(self.REG_FCR, 0x0D)
        self._write_reg(self.REG_SCR, 0x03)
        self.set_baudrate(baud)
        self._set_page(0)
        self._write_reg(self.REG_LCR, 0x00)  # 8N1

    def set_baudrate(self, baud: int):
        integer = int(self.FOSC / (baud * 16) - 1)
        remainder = self.FOSC % (baud * 16)
        pres = int(remainder / (baud * 16))
        while pres > 0x0A:
            pres //= 0x0A

        self._set_page(1)
        self._write_reg(self.REG_BAUD1, (integer >> 8) & 0xFF)
        self._write_reg(self.REG_BAUD0, integer & 0xFF)
        self._write_reg(self.REG_PRES, pres)
        self._set_page(0)

    def available(self) -> int:
        self._set_page(0)
        count = self._read_reg(self.REG_RFCNT, 1)[0]
        if count == 0:
            fsr = self._read_reg(self.REG_FSR, 1)[0]
            if (fsr >> 3) & 0x01:
                return 256
        return count

    def read_available(self, max_bytes: int = 64) -> list[int]:
        count = min(max_bytes, self.available())
        return [self._read_fifo_byte() for _ in range(count)]


# =============================================================================
# A02YYUW SENSOR READER
# =============================================================================


@dataclass
class SensorReading:
    distance_mm: Optional[int] = None
    timestamp: float = 0.0
    valid: bool = False


class A02YYUWI2CSensor:
    def __init__(self, name: str, channel: int, simulate: bool = False):
        self.name = name
        self.channel_number = channel
        self.simulate = simulate
        self.reading = SensorReading()
        self.reading_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.uart: Optional[DFR0627Channel] = None
        self.bus = None
        self.rx_buffer: list[int] = []
        self.failed_reason = ""

    def start(self):
        if self.simulate:
            self.thread = threading.Thread(target=self._simulate_run, daemon=True)
            self.thread.start()
            return

        if SMBus is None:
            self.failed_reason = "python smbus/smbus2 missing"
            print(f"[{self.name}] WARNING: {self.failed_reason}")
            return

        try:
            self.bus = SMBus(I2C_BUS_NUMBER)
            self.uart = DFR0627Channel(
                self.bus,
                self.channel_number,
                IIC_UART_IA1,
                IIC_UART_IA0,
            )
            self.uart.begin(A02YYUW_BAUD)
        except Exception as exc:
            self.failed_reason = str(exc)
            print(
                f"[{self.name}] WARNING: could not start DFR0627 "
                f"UART{self.channel_number}: {exc}"
            )
            return

        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print(f"[{self.name}] Started on DFR0627 UART{self.channel_number}")

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass

    def get_distance_mm(self) -> Optional[int]:
        with self.reading_lock:
            if not self.reading.valid:
                return None
            if time.monotonic() - self.reading.timestamp > 2.0:
                return None
            return self.reading.distance_mm

    def is_healthy(self) -> bool:
        return self.get_distance_mm() is not None

    def _set_distance(self, distance_mm: int):
        with self.reading_lock:
            self.reading = SensorReading(distance_mm, time.monotonic(), True)

    def _simulate_run(self):
        base = 250 if self.channel_number == 1 else 350
        while not self.stop_event.is_set():
            self._set_distance(max(30, min(4500, base + random.randint(-35, 35))))
            time.sleep(0.25)

    def _run(self):
        assert self.uart is not None
        while not self.stop_event.is_set():
            try:
                data = self.uart.read_available(64)
                if data:
                    self.rx_buffer.extend(data)
                    self._parse_frames()
                else:
                    time.sleep(0.02)
            except Exception as exc:
                print(f"[{self.name}] read error: {exc}")
                time.sleep(0.5)

    def _parse_frames(self):
        # A02YYUW frame: FF, high byte, low byte, checksum.
        while len(self.rx_buffer) >= 4:
            if self.rx_buffer[0] != 0xFF:
                self.rx_buffer.pop(0)
                continue

            frame = self.rx_buffer[:4]
            checksum = (frame[0] + frame[1] + frame[2]) & 0xFF
            if checksum != frame[3]:
                self.rx_buffer.pop(0)
                continue

            distance = (frame[1] << 8) | frame[2]
            del self.rx_buffer[:4]
            if 30 <= distance <= 4500:
                self._set_distance(distance)

        if len(self.rx_buffer) > 512:
            self.rx_buffer = self.rx_buffer[-128:]


# =============================================================================
# MACHINE CONTROL
# =============================================================================


@dataclass
class MachineStatus:
    timestamp: float = field(default_factory=time.monotonic)
    flap_open: bool = False
    motor_on: bool = False
    motor_automation_enabled: bool = True
    motor_trigger_armed: bool = False
    motor_remaining_seconds: float = 0.0
    flap_open_count: int = 0
    solenoid_enabled: bool = True
    solenoid_busy: bool = False
    led_available: bool = False
    led_total_count: int = DOTSTAR_TOTAL_LEDS_DEFAULT
    led_active_count: int = DOTSTAR_ACTIVE_LEDS_DEFAULT
    led_brightness: int = DOTSTAR_BRIGHTNESS_DEFAULT
    led_color_text: str = "OFF"
    led_error: str = ""
    box_distance_mm: Optional[int] = None
    box_present: Optional[bool] = None
    fill_distance_mm: Optional[int] = None
    fill_percent: Optional[float] = None
    box_sensor_healthy: bool = False
    fill_sensor_healthy: bool = False
    last_event: str = "Starting"
    simulation_mode: bool = False


class MachineController:
    def __init__(
        self,
        simulate: bool = False,
        led_spi_device: str = DOTSTAR_SPI_DEVICE_DEFAULT,
        led_total_count: int = DOTSTAR_TOTAL_LEDS_DEFAULT,
        led_active_count: int = DOTSTAR_ACTIVE_LEDS_DEFAULT,
        led_brightness: int = DOTSTAR_BRIGHTNESS_DEFAULT,
    ):
        self.simulate = simulate
        self.led_spi_device = led_spi_device
        self.led_strip: Optional[DotStarLedStrip] = None
        self.led_total_count = DotStarLedStrip._clamp_count(led_total_count, minimum=1)
        self.led_active_count = max(0, min(DotStarLedStrip._clamp_count(led_active_count), self.led_total_count))
        self.led_brightness = DotStarLedStrip._clamp_brightness(led_brightness)
        self.led_color_rgb = DOTSTAR_LAST_COLOR_DEFAULT
        self.motor_automation_enabled = True
        self.motor_trigger_armed = False
        self.flap_open_count = 0
        self.solenoid_enabled = True
        self.motor_on = False
        self.solenoid_busy = False
        self._solenoid_state_lock = threading.Lock()
        self.motor_off_deadline = 0.0
        self.last_reed_raw: Optional[int] = None
        self.last_reed_change_time = time.monotonic()
        self.reed_i2c_bus = None
        self._last_reed_i2c_error = 0.0
        self.stable_reed_state: Optional[int] = None
        self.reed_closed_level: Optional[int] = None
        self.reed_open_level: Optional[int] = None
        self.last_event = "Starting"
        self.status_queue: queue.Queue[str] = queue.Queue()
        self._cleaned = False

        self.box_sensor = A02YYUWI2CSensor("Box presence sensor", 1, simulate)
        self.fill_sensor = A02YYUWI2CSensor("Fill level sensor", 2, simulate)

    def setup(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        # Safe motor state: DFR0457/Crydom motor request LOW.
        GPIO.setup(PIN_MOTOR_SSR, GPIO.OUT, initial=MOTOR_OFF_LEVEL_VALUE)
        configure_solenoid_pinmux(
            self.simulate,
            reason="after motor GPIO.setup",
        )
        GPIO.output(PIN_MOTOR_SSR, MOTOR_OFF_LEVEL_VALUE)
        print("[GPIO] Motor request configured: physical pin 31 / DFR0457-Crydom")

        # MD13S solenoid outputs. PWM must default LOW so the latching
        # solenoid is never held energised.
        GPIO.setup(PIN_SOL_PWM, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_SOL_DIR, GPIO.OUT, initial=SOLENOID_UNLOCK_DIR_VALUE)
        GPIO.output(PIN_SOL_PWM, GPIO.LOW)
        GPIO.output(PIN_SOL_DIR, SOLENOID_UNLOCK_DIR_VALUE)
        print("[GPIO] MD13S solenoid configured: pin 32 PWM, pin 33 DIR")

        initial = self._setup_reed_input()
        self.last_reed_raw = initial
        self.stable_reed_state = initial

        # Fixed MCP23017 polarity from the physical PA0 test:
        # PA0=0 is flap CLOSED, PA0=1 is flap OPEN.
        self.reed_closed_level = FLAP_CLOSED_LEVEL_VALUE
        self.reed_open_level = FLAP_OPEN_LEVEL_VALUE

        self.box_sensor.start()
        self.fill_sensor.start()
        self._setup_led_strip()
        self.log_event(
            "MCP23017 reed ready: I2C bus "
            f"{MCP_REED_I2C_BUS}, address 0x{MCP_REED_I2C_ADDR:02X}, PA0/GPA0; "
            f"CLOSED={self.reed_closed_level}, OPEN={self.reed_open_level}, "
            f"current={initial}"
        )
        if initial == self.reed_open_level:
            self.log_event("Startup warning: flap currently reports OPEN")
        self.log_event("Controller started")

    def cleanup(self):
        if self._cleaned:
            return
        self._cleaned = True

        self.log_event("Shutting down safely")
        self.cancel_motor_trigger(log=False)
        self.motor_off(log=False)
        self.solenoid_off()
        self.led_off(log=False)
        if self.led_strip is not None:
            self.led_strip.close()
        self.box_sensor.stop()
        self.fill_sensor.stop()
        if self.reed_i2c_bus is not None:
            try:
                self.reed_i2c_bus.close()
            except Exception:
                pass
            self.reed_i2c_bus = None
        GPIO.cleanup()

    def log_event(self, message: str):
        self.last_event = message
        line = f"[{time.strftime('%H:%M:%S')}] {message}"
        print(line)
        self.status_queue.put(line)

    # ------------------------------------------------------------------
    # DotStar / APA102 LED strip
    # ------------------------------------------------------------------

    def _setup_led_strip(self):
        self.led_strip = DotStarLedStrip(
            self.led_spi_device,
            total_leds=self.led_total_count,
            active_leds=self.led_active_count,
            brightness=self.led_brightness,
            simulate=self.simulate,
        )
        if self.led_strip.available:
            self.log_event(
                "DotStar LEDs ready: "
                f"{self.led_strip.spi_device}, total={self.led_total_count}, "
                f"active={self.led_active_count}, brightness={self.led_brightness}"
            )
        else:
            self.log_event(
                "DotStar LEDs not available: "
                f"{self.led_strip.error or self.led_strip.spi_device}"
            )

    def _led_status_text(self) -> str:
        red, green, blue = self.led_color_rgb
        if red == 0 and green == 0 and blue == 0:
            return "OFF"
        return f"RGB({red},{green},{blue})"

    def set_led_total_count(self, count: int):
        self.led_total_count = DotStarLedStrip._clamp_count(count, minimum=1)
        self.led_active_count = max(0, min(self.led_active_count, self.led_total_count))
        if self.led_strip is not None:
            self.led_strip.set_total_leds(self.led_total_count)
        self.log_event(f"LED total count set to {self.led_total_count}")

    def set_led_active_count(self, count: int):
        self.led_active_count = max(
            0,
            min(DotStarLedStrip._clamp_count(count), self.led_total_count),
        )
        if self.led_strip is not None:
            self.led_strip.set_active_leds(self.led_active_count)
        self.log_event(f"LEDs to light set to {self.led_active_count}")

    def set_led_brightness(self, brightness: int):
        self.led_brightness = DotStarLedStrip._clamp_brightness(brightness)
        if self.led_strip is not None:
            self.led_strip.set_brightness(self.led_brightness)
        self.log_event(f"LED brightness set to {self.led_brightness}/31")

    def led_set_color(self, red: int, green: int, blue: int, label: str = "custom"):
        self.led_color_rgb = (
            DotStarLedStrip._clamp_color(red),
            DotStarLedStrip._clamp_color(green),
            DotStarLedStrip._clamp_color(blue),
        )
        if self.led_strip is None:
            self.log_event("LED command ignored; LED strip was not initialised")
            return
        self.led_strip.set_color(
            self.led_color_rgb[0],
            self.led_color_rgb[1],
            self.led_color_rgb[2],
        )
        if self.led_strip.error:
            self.log_event(f"LED {label} command failed: {self.led_strip.error}")
        else:
            self.log_event(
                f"LEDs {label}: {self.led_active_count}/{self.led_total_count} "
                f"at brightness {self.led_brightness}/31"
            )

    def led_off(self, log: bool = True):
        self.led_color_rgb = (0, 0, 0)
        if self.led_strip is not None:
            self.led_strip.off()
        if log:
            if self.led_strip is not None and self.led_strip.error:
                self.log_event(f"LED OFF command failed: {self.led_strip.error}")
            else:
                self.log_event("LEDs OFF")


    # ------------------------------------------------------------------
    # Motor / Crydom SSR
    # ------------------------------------------------------------------

    def set_motor_automation_enabled(self, enabled: bool):
        self.motor_automation_enabled = enabled
        if enabled:
            self.log_event("Motor automation enabled")
        else:
            self.cancel_motor_trigger(log=False)
            self.motor_off(log=False)
            self.log_event("Motor automation disabled; motor stopped")

    def _setup_reed_input(self) -> int:
        """Initialise the flap reed input and return its current 0/1 level."""
        if self.simulate:
            GPIO.setup(PIN_REED, GPIO.IN)
            GPIO.simulate_set_pin(PIN_REED, FLAP_CLOSED_LEVEL_VALUE)
            return int(GPIO.input(PIN_REED))

        if SMBus is None:
            raise RuntimeError(
                "smbus/smbus2 is missing; install with: "
                "sudo apt install -y python3-smbus or python3 -m pip install smbus2"
            )

        self.reed_i2c_bus = SMBus(MCP_REED_I2C_BUS)

        # Configure PA0/GPA0 as an input and enable the MCP23017 internal pull-up.
        iodira = self.reed_i2c_bus.read_byte_data(MCP_REED_I2C_ADDR, MCP23017_IODIRA)
        self.reed_i2c_bus.write_byte_data(
            MCP_REED_I2C_ADDR,
            MCP23017_IODIRA,
            iodira | MCP_REED_PA0_MASK,
        )
        gppua = self.reed_i2c_bus.read_byte_data(MCP_REED_I2C_ADDR, MCP23017_GPPUA)
        self.reed_i2c_bus.write_byte_data(
            MCP_REED_I2C_ADDR,
            MCP23017_GPPUA,
            gppua | MCP_REED_PA0_MASK,
        )

        initial = self._read_reed_level()
        print(
            "[I2C] MCP23017 reed configured: "
            f"bus {MCP_REED_I2C_BUS}, addr 0x{MCP_REED_I2C_ADDR:02X}, PA0/GPA0={initial}"
        )
        return initial

    def _read_reed_level(self) -> int:
        """Read flap reed level. Returns 1 for open, 0 for closed."""
        if self.simulate:
            return int(GPIO.input(PIN_REED))

        if self.reed_i2c_bus is None:
            raise RuntimeError("MCP23017 reed I2C bus is not initialised")

        gpioa = self.reed_i2c_bus.read_byte_data(MCP_REED_I2C_ADDR, MCP23017_GPIOA)
        return FLAP_OPEN_LEVEL_VALUE if (gpioa & MCP_REED_PA0_MASK) else FLAP_CLOSED_LEVEL_VALUE

    def _reed_is_open(self, value: Optional[int] = None) -> bool:
        level = self.stable_reed_state if value is None else int(value)
        return (
            self.reed_open_level is not None
            and level is not None
            and int(level) == int(self.reed_open_level)
        )

    def arm_motor_for_next_flap_open(self) -> bool:
        """Arm one reed-open event after a valid barcode transaction.

        The arm is consumed by the first debounced CLOSED->OPEN transition.
        This prevents an idle/manual flap movement from starting the crusher.
        """
        if not self.motor_automation_enabled:
            self.log_event("Motor trigger not armed; automation is disabled")
            return False
        if self._reed_is_open():
            self.log_event("Motor trigger not armed; flap already reports OPEN")
            return False
        self.motor_trigger_armed = True
        self.log_event("Motor armed for the next valid flap OPEN transition")
        return True

    def cancel_motor_trigger(self, log: bool = True):
        was_armed = self.motor_trigger_armed
        self.motor_trigger_armed = False
        if log and was_armed:
            self.log_event("Pending flap-open motor trigger cancelled")

    def motor_on_for_timed_cycle(self):
        if not self.motor_automation_enabled:
            self.log_event("Flap opened; motor automation is disabled")
            return

        self.motor_off_deadline = time.monotonic() + MOTOR_RUN_SECONDS
        GPIO.output(PIN_MOTOR_SSR, MOTOR_ON_LEVEL_VALUE)

        if not self.motor_on:
            self.motor_on = True
            self.log_event(
                f"Motor ON — reed-triggered bottle cycle ({MOTOR_RUN_SECONDS:.0f} s)"
            )
        else:
            self.log_event(f"Motor timer reset to {MOTOR_RUN_SECONDS:.0f} seconds")

    def manual_motor_on(self):
        # Manual ON cancels the timed deadline and holds until Motor OFF.
        self.motor_off_deadline = 0.0
        GPIO.output(PIN_MOTOR_SSR, MOTOR_ON_LEVEL_VALUE)
        if not self.motor_on:
            self.motor_on = True
            self.log_event("Motor ON — manual")

    def motor_off(self, log: bool = True):
        GPIO.output(PIN_MOTOR_SSR, MOTOR_OFF_LEVEL_VALUE)
        was_on = self.motor_on
        self.motor_on = False
        self.motor_off_deadline = 0.0
        if log and was_on:
            self.log_event("Motor OFF")

    def manual_motor_off(self):
        was_on = self.motor_on
        self.motor_off(log=False)
        self.log_event("Motor OFF — manual" if was_on else "Motor already OFF")

    # ------------------------------------------------------------------
    # IBT-2 flap lock — verified solenoid_test.py pulse logic
    # ------------------------------------------------------------------

    def set_solenoid_enabled(self, enabled: bool):
        self.solenoid_enabled = bool(enabled)
        if enabled:
            self.log_event("MD13S flap-lock control enabled")
        else:
            self.solenoid_off()
            self.log_event("MD13S flap-lock control disabled")

    def bridge_off(self):
        """Ensure the MD13S output is de-energised."""
        GPIO.output(PIN_SOL_PWM, GPIO.LOW)

    def solenoid_off(self):
        """Immediately de-energise the MD13S output."""
        self.bridge_off()

    def solenoid_unlock_pulse(self) -> bool:
        return self._start_solenoid_pulse(
            direction="unlock",
            dir_value=SOLENOID_UNLOCK_DIR_VALUE,
        )

    def solenoid_lock_pulse(self) -> bool:
        return self._start_solenoid_pulse(
            direction="lock",
            dir_value=SOLENOID_LOCK_DIR_VALUE,
        )

    def _start_solenoid_pulse(self, direction: str, dir_value: int) -> bool:
        if not self.solenoid_enabled:
            self.log_event(
                f"{direction.title()} ignored; MD13S flap-lock control disabled"
            )
            return False

        # The pulse is not a cooldown. This guard only prevents two directions
        # from being energised at the same instant.
        with self._solenoid_state_lock:
            if self.solenoid_busy:
                self.log_event(
                    "Flap-lock command ignored; MD13S pulse already in progress"
                )
                return False
            self.solenoid_busy = True

        threading.Thread(
            target=self._md13s_solenoid_pulse_worker,
            args=(direction, dir_value),
            daemon=True,
        ).start()
        return True

    def _md13s_solenoid_pulse_worker(self, direction: str, dir_value: int):
        self.log_event(
            f"Flap {direction}: MD13S DIR={dir_value}, PWM pulse for "
            f"{SOLENOID_PULSE_SECONDS * 1000:.0f} ms"
        )

        try:
            # Always de-energise before changing direction.
            GPIO.output(PIN_SOL_PWM, GPIO.LOW)
            GPIO.output(PIN_SOL_DIR, dir_value)
            time.sleep(0.02)

            GPIO.output(PIN_SOL_PWM, GPIO.HIGH)
            try:
                time.sleep(SOLENOID_PULSE_SECONDS)
            finally:
                GPIO.output(PIN_SOL_PWM, GPIO.LOW)

            self.log_event(f"Flap {direction} pulse complete; MD13S PWM OFF")

        except Exception as exc:
            GPIO.output(PIN_SOL_PWM, GPIO.LOW)
            self.log_event(f"MD13S solenoid {direction} error: {exc}")

        finally:
            with self._solenoid_state_lock:
                self.solenoid_busy = False

    # ------------------------------------------------------------------
    # Reed and timers
    # ------------------------------------------------------------------

    def tick(self):
        if self.simulate:
            self._simulate_reed_activity()
        self._update_reed_switch()
        self._update_motor_timer()

    def _simulate_reed_activity(self):
        open_level = (
            self.reed_open_level
            if self.reed_open_level is not None
            else FLAP_OPEN_LEVEL_VALUE
        )
        closed_level = (
            self.reed_closed_level
            if self.reed_closed_level is not None
            else (GPIO.LOW if open_level == GPIO.HIGH else GPIO.HIGH)
        )
        if random.random() < 0.0005:
            GPIO.simulate_set_pin(PIN_REED, open_level)
        elif self._reed_is_open() and random.random() < 0.04:
            GPIO.simulate_set_pin(PIN_REED, closed_level)

    def _update_reed_switch(self):
        now = time.monotonic()
        try:
            raw = self._read_reed_level()
        except Exception as exc:
            if now - self._last_reed_i2c_error > 5.0:
                self._last_reed_i2c_error = now
                self.log_event(f"MCP23017 reed read error: {exc}")
            return

        if raw != self.last_reed_raw:
            self.last_reed_raw = raw
            self.last_reed_change_time = now

        if now - self.last_reed_change_time < DEBOUNCE_SECONDS:
            return

        if raw == self.stable_reed_state:
            return

        old_state = self.stable_reed_state
        self.stable_reed_state = raw
        is_open = self._reed_is_open(raw)
        self.log_event("Flap OPEN" if is_open else "Flap CLOSED")

        if not self._reed_is_open(old_state) and is_open:
            self.flap_open_count += 1
            if self.motor_trigger_armed:
                # Consume first, so contact bounce or a second opening cannot
                # restart/reset the 15-second timer without another valid scan.
                self.motor_trigger_armed = False
                self.log_event("Accepted bottle opened flap; starting crusher motor")
                self.motor_on_for_timed_cycle()
            elif self.motor_on:
                self.log_event(
                    "Flap opened while motor already running; timer unchanged"
                )
            else:
                self.log_event(
                    "Flap opened without a valid scanned bottle; motor remains OFF"
                )

    def _update_motor_timer(self):
        if (
            self.motor_on
            and self.motor_off_deadline > 0.0
            and time.monotonic() >= self.motor_off_deadline
        ):
            self.motor_off()

    # ------------------------------------------------------------------
    # Sensor-derived state
    # ------------------------------------------------------------------

    def get_box_present(self) -> Optional[bool]:
        distance = self.box_sensor.get_distance_mm()
        return None if distance is None else distance <= BOX_PRESENT_DISTANCE_MM

    def get_fill_percent(self) -> Optional[float]:
        distance = self.fill_sensor.get_distance_mm()
        if distance is None:
            return None

        span = EMPTY_BOX_DISTANCE_MM - FULL_BOX_DISTANCE_MM
        if span <= 0:
            return None

        fill = (EMPTY_BOX_DISTANCE_MM - distance) / span * 100.0
        return max(0.0, min(100.0, fill))

    def all_outputs_off(self):
        self.cancel_motor_trigger(log=False)
        self.motor_off(log=False)
        self.solenoid_off()
        self.led_off(log=False)
        self.log_event("All outputs, LEDs, and pending motor trigger forced OFF")

    def get_status(self) -> MachineStatus:
        now = time.monotonic()
        remaining = 0.0
        if self.motor_on and self.motor_off_deadline > 0:
            remaining = max(0.0, self.motor_off_deadline - now)

        return MachineStatus(
            timestamp=now,
            flap_open=self._reed_is_open(),
            motor_on=self.motor_on,
            motor_automation_enabled=self.motor_automation_enabled,
            motor_trigger_armed=self.motor_trigger_armed,
            motor_remaining_seconds=remaining,
            flap_open_count=self.flap_open_count,
            solenoid_enabled=self.solenoid_enabled,
            solenoid_busy=self.solenoid_busy,
            led_available=bool(self.led_strip and self.led_strip.available),
            led_total_count=self.led_total_count,
            led_active_count=self.led_active_count,
            led_brightness=self.led_brightness,
            led_color_text=self._led_status_text(),
            led_error="" if not self.led_strip else (self.led_strip.error or ""),
            box_distance_mm=self.box_sensor.get_distance_mm(),
            box_present=self.get_box_present(),
            fill_distance_mm=self.fill_sensor.get_distance_mm(),
            fill_percent=self.get_fill_percent(),
            box_sensor_healthy=self.box_sensor.is_healthy(),
            fill_sensor_healthy=self.fill_sensor.is_healthy(),
            last_event=self.last_event,
            simulation_mode=self.simulate,
        )



try:
    from PySide6.QtCore import QObject, Property, QTimer, Signal, Slot, Qt
    from PySide6.QtGui import QFont, QImage, QPixmap
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QDialog,
        QFrame,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QListWidget,
        QMainWindow,
        QMessageBox,
        QProgressBar,
        QPushButton,
        QSpinBox,
        QSizePolicy,
        QTabWidget,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    raise SystemExit(
        "PySide6 is missing. Install it for the user that runs the GUI:\n"
        "  python3 -m pip install PySide6\n\n"
        "When GPIO needs sudo, run with:\n"
        "  sudo -E python3 controller_kiosk.py --fullscreen"
    ) from exc


# =============================================================================
# BARCODE LOOKUP / IMX708 SCANNER SERVICE
# =============================================================================

APP_DIR = Path(__file__).resolve().parent
BARCODE_SCANNER = APP_DIR / "fast_barcode_scanner_jetson.py"
DEFAULT_BARCODE_PRODUCTS = APP_DIR / "barcode_products.csv"
DEFAULT_PRODUCT_OPTIONS = APP_DIR / "product_options.csv"
BARCODE_PREVIEW_HEADER = struct.Struct("!HH")


class ProductCatalogue:
    """CSV-backed barcode-to-product lookup with safe atomic updates."""

    FIELDNAMES = ["barcode", "product_name", "brand", "subbrand", "capacity"]

    def __init__(self, path: Path):
        self.path = path
        self._modified_ns: Optional[int] = None
        self._products: dict[str, str] = {}
        self.reload(force=True)

    @staticmethod
    def clean_barcode(value: str) -> str:
        return "".join(ch for ch in str(value).strip() if ch.isdigit())

    def _read_rows(self) -> dict[str, dict[str, str]]:
        rows: dict[str, dict[str, str]] = {}
        if not self.path.exists():
            return rows
        with self.path.open(newline="", encoding="utf-8-sig") as source:
            reader = csv.DictReader(source)
            if not reader.fieldnames or not {"barcode", "product_name"}.issubset(
                reader.fieldnames
            ):
                raise ValueError(
                    f"{self.path.name} must contain barcode and product_name columns"
                )
            for source_row in reader:
                barcode = self.clean_barcode(source_row.get("barcode", ""))
                product_name = (source_row.get("product_name") or "").strip()
                if not barcode or not product_name:
                    continue
                rows[barcode] = {
                    "barcode": barcode,
                    "product_name": product_name,
                    "brand": (source_row.get("brand") or "").strip(),
                    "subbrand": (source_row.get("subbrand") or "").strip(),
                    "capacity": (source_row.get("capacity") or "").strip(),
                }
        return rows

    def reload(self, force: bool = False):
        try:
            modified_ns = self.path.stat().st_mtime_ns
        except FileNotFoundError:
            self._products = {}
            self._modified_ns = None
            return
        if not force and modified_ns == self._modified_ns:
            return
        rows = self._read_rows()
        self._products = {
            barcode: row["product_name"] for barcode, row in rows.items()
        }
        self._modified_ns = modified_ns

    def find(self, scanned_barcode: str) -> Optional[str]:
        try:
            self.reload()
        except (OSError, ValueError, csv.Error):
            return None
        barcode = self.clean_barcode(scanned_barcode)
        candidates = [barcode]
        if len(barcode) == 12:
            candidates.append("0" + barcode)
        elif len(barcode) == 13 and barcode.startswith("0"):
            candidates.append(barcode[1:])
        for candidate in candidates:
            if candidate in self._products:
                return self._products[candidate]
        return None

    def save_mapping(
        self,
        barcode: str,
        product_name: str,
        brand: str = "",
        subbrand: str = "",
        capacity: str = "",
    ):
        cleaned = self.clean_barcode(barcode)
        name = product_name.strip()
        if not cleaned:
            raise ValueError("Cannot save an empty barcode")
        if not name:
            raise ValueError("Cannot save an empty product name")

        rows = self._read_rows()
        rows[cleaned] = {
            "barcode": cleaned,
            "product_name": name,
            "brand": brand.strip(),
            "subbrand": subbrand.strip(),
            "capacity": capacity.strip(),
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        with temporary.open("w", newline="", encoding="utf-8") as target:
            writer = csv.DictWriter(target, fieldnames=self.FIELDNAMES)
            writer.writeheader()
            for row_barcode in sorted(rows):
                writer.writerow(rows[row_barcode])
        temporary.replace(self.path)

        # The kiosk normally runs through sudo. Restore ownership to the desktop
        # user so the CSV remains editable without sudo.
        sudo_uid = os.environ.get("SUDO_UID")
        sudo_gid = os.environ.get("SUDO_GID")
        if sudo_uid and sudo_gid:
            try:
                os.chown(self.path, int(sudo_uid), int(sudo_gid))
            except (OSError, ValueError):
                pass
        self.reload(force=True)


class ProductOptions:
    """Editable brand -> subbrand -> capacity hierarchy."""

    def __init__(self, path: Path):
        self.path = path
        self._modified_ns: Optional[int] = None
        self._rows: list[dict[str, str]] = []
        self.reload(force=True)

    def reload(self, force: bool = False):
        try:
            modified_ns = self.path.stat().st_mtime_ns
        except FileNotFoundError:
            self._rows = []
            self._modified_ns = None
            return
        if not force and modified_ns == self._modified_ns:
            return

        rows: list[dict[str, str]] = []
        with self.path.open(newline="", encoding="utf-8-sig") as source:
            reader = csv.DictReader(source)
            required = {"brand", "subbrand", "capacity"}
            if not reader.fieldnames or not required.issubset(reader.fieldnames):
                raise ValueError(
                    f"{self.path.name} must contain brand, subbrand and capacity columns"
                )
            for source_row in reader:
                brand = (source_row.get("brand") or "").strip()
                subbrand = (source_row.get("subbrand") or "").strip()
                capacity = (source_row.get("capacity") or "").strip()
                if not brand or not subbrand or not capacity:
                    continue
                product_name = (
                    (source_row.get("product_name") or "").strip()
                    or f"{brand} {subbrand} {capacity}".strip()
                )
                rows.append(
                    {
                        "brand": brand,
                        "subbrand": subbrand,
                        "capacity": capacity,
                        "product_name": product_name,
                    }
                )
        self._rows = rows
        self._modified_ns = modified_ns

    @staticmethod
    def _unique(values: list[str]) -> list[str]:
        return sorted(set(values), key=str.casefold)

    def brands(self) -> list[str]:
        self.reload()
        return self._unique([row["brand"] for row in self._rows])

    def subbrands(self, brand: str) -> list[str]:
        self.reload()
        return self._unique(
            [row["subbrand"] for row in self._rows if row["brand"] == brand]
        )

    def capacities(self, brand: str, subbrand: str) -> list[str]:
        self.reload()
        return self._unique(
            [
                row["capacity"]
                for row in self._rows
                if row["brand"] == brand and row["subbrand"] == subbrand
            ]
        )

    def resolve(self, brand: str, subbrand: str, capacity: str) -> str:
        self.reload()
        for row in self._rows:
            if (
                row["brand"] == brand
                and row["subbrand"] == subbrand
                and row["capacity"] == capacity
            ):
                return row["product_name"]
        return f"{brand} {subbrand} {capacity}".strip()


class BarcodeImageProvider:
    """Thread-safe in-memory image store used by the Qt Widgets previews."""

    def __init__(self):
        self._lock = threading.Lock()
        self._image = QImage(400, 225, QImage.Format.Format_Grayscale8)
        self._image.fill(24)

    def set_image(self, image: QImage):
        with self._lock:
            self._image = image.copy()

    def get_image(self) -> QImage:
        with self._lock:
            return self._image.copy()



class CameraScannerService(QObject):
    """One isolated raw-V4L2 barcode scanner process and preview stream."""

    changed = Signal()
    previewChanged = Signal()
    barcodeDetected = Signal(str, str, str)

    _scanLineReceived = Signal(str)
    _diagnosticLineReceived = Signal(str)
    _processStopped = Signal(int)
    _previewPacketReceived = Signal(bytes, int, int)

    def __init__(
        self,
        args,
        image_provider: BarcodeImageProvider,
        sensor_name: str,
        display_name: str,
        *,
        secondary_config: bool = False,
        auto_index: int = 0,
        preview_provider_key: str = "barcode",
    ):
        super().__init__()
        self.args = args
        self.image_provider = image_provider
        self.sensor_name = sensor_name.lower()
        self.display_name = display_name
        self.secondary_config = secondary_config
        self.auto_index = int(auto_index)
        self.preview_provider_key = preview_provider_key
        self._process: Optional[subprocess.Popen] = None
        self._preview_socket: Optional[socket.socket] = None
        self._closing = False
        self._running = False
        self._cycle_hold = False
        self._hold_detail = "Scanning is disabled by the bottle cycle"
        self._status = "STOPPED"
        self._detail = f"{display_name} scanner has not started"
        self._preview_revision = 0

        self._scanLineReceived.connect(self._handle_scan_line)
        self._diagnosticLineReceived.connect(self._handle_diagnostic)
        self._processStopped.connect(self._handle_process_stopped)
        self._previewPacketReceived.connect(self._handle_preview_packet)

    @Property(bool, notify=changed)
    def running(self) -> bool:
        return self._running

    @Property(str, notify=changed)
    def status(self) -> str:
        return self._status

    @Property(str, notify=changed)
    def detail(self) -> str:
        return self._detail

    @Property(str, notify=previewChanged)
    def previewSource(self) -> str:
        return f"image://{self.preview_provider_key}/preview-{self._preview_revision}"

    def _config(self) -> dict[str, object]:
        if not self.secondary_config:
            return {
                "backend": self.args.camera_backend,
                "device": self.args.v4l2_device,
                "sensor_width": self.args.sensor_width,
                "sensor_height": self.args.sensor_height,
                "sensor_fps": self.args.sensor_fps,
                "width": self.args.barcode_width,
                "height": self.args.barcode_height,
                "fps": self.args.barcode_fps,
                "preview_width": self.args.preview_width,
                "preview_height": self.args.preview_height,
                "preview_fps": self.args.preview_fps,
            }
        return {
            "backend": "raw",
            "device": self.args.secondary_v4l2_device,
            "sensor_width": self.args.secondary_sensor_width,
            "sensor_height": self.args.secondary_sensor_height,
            "sensor_fps": self.args.secondary_sensor_fps,
            "width": self.args.secondary_barcode_width,
            "height": self.args.secondary_barcode_height,
            "fps": self.args.secondary_barcode_fps,
            "preview_width": self.args.secondary_preview_width,
            "preview_height": self.args.secondary_preview_height,
            "preview_fps": self.args.secondary_preview_fps,
        }

    def _scanner_command(self, preview_fd: int) -> list[str]:
        cfg = self._config()
        selected_device = str(cfg["device"])
        if selected_device.strip().lower() == "auto":
            selected_device = f"auto:{self.auto_index}"
        cmd = [
            sys.executable,
            "-u",
            str(BARCODE_SCANNER),
            "--camera-backend",
            str(cfg["backend"]),
            "--raw-sensor-name",
            self.sensor_name,
            "--sensor-id",
            str(self.args.sensor_id),
            "--sensor-width",
            str(cfg["sensor_width"]),
            "--sensor-height",
            str(cfg["sensor_height"]),
            "--sensor-fps",
            str(cfg["sensor_fps"]),
            "--v4l2-device",
            selected_device,
            "--width",
            str(cfg["width"]),
            "--height",
            str(cfg["height"]),
            "--fps",
            str(cfg["fps"]),
            "--formats",
            self.args.barcode_formats,
            "--roi",
            self.args.barcode_roi,
            "--binarizer",
            self.args.barcode_binarizer,
            "--cooldown",
            str(self.args.barcode_cooldown),
            "--output-mode",
            "json",
            "--stats-interval",
            "0",
            "--embedded-preview-fd",
            str(preview_fd),
            "--embedded-preview-width",
            str(cfg["preview_width"]),
            "--embedded-preview-height",
            str(cfg["preview_height"]),
            "--embedded-preview-fps",
            str(cfg["preview_fps"]),
        ]
        if self.sensor_name == "imx708" and self.args.gstreamer_pipeline:
            cmd.extend(["--gstreamer-pipeline", self.args.gstreamer_pipeline])
        if self.sensor_name == "imx708" and self.args.exposure_us is not None:
            cmd.extend(
                [
                    "--exposure-us",
                    str(self.args.exposure_us),
                    "--gain",
                    str(self.args.camera_gain),
                ]
            )
        if self.args.barcode_rotate:
            cmd.append("--rotate")
        if self.args.barcode_downscale:
            cmd.append("--downscale")
        if self.args.barcode_invert:
            cmd.append("--invert")
        # Child processes deliberately do not beep. The coordinator beeps once
        # after it atomically accepts the first detection from either camera.
        return cmd

    @Slot()
    def startScanner(self):
        if self._cycle_hold:
            self._status = "SCANNING DISABLED"
            self._detail = self._hold_detail
            self.changed.emit()
            return
        if self._process is not None and self._process.poll() is None:
            return
        if not BARCODE_SCANNER.exists():
            self._set_error(f"Scanner file missing: {BARCODE_SCANNER}")
            return

        parent_socket, child_socket = socket.socketpair(
            socket.AF_UNIX, socket.SOCK_DGRAM
        )
        parent_socket.settimeout(0.5)
        child_socket.setblocking(False)
        preview_fd = child_socket.fileno()

        try:
            process = subprocess.Popen(
                self._scanner_command(preview_fd),
                cwd=APP_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                start_new_session=True,
                pass_fds=(preview_fd,),
            )
        except OSError as exc:
            parent_socket.close()
            child_socket.close()
            self._set_error(f"Could not start {self.display_name} scanner: {exc}")
            return
        finally:
            child_socket.close()

        self._process = process
        self._preview_socket = parent_socket
        self._running = True
        self._status = "STARTING CAMERA"
        self._detail = f"Opening the auto-detected {self.display_name} raw V4L2 stream"
        self.changed.emit()

        threading.Thread(target=self._read_stdout, args=(process,), daemon=True).start()
        threading.Thread(target=self._read_stderr, args=(process,), daemon=True).start()
        threading.Thread(
            target=self._read_preview, args=(parent_socket,), daemon=True
        ).start()
        threading.Thread(target=self._watch_process, args=(process,), daemon=True).start()

    @Slot()
    def stopScanner(self):
        self._stop_process(managed=False)

    def holdForCycle(self, detail: str):
        self._cycle_hold = True
        self._hold_detail = detail
        self._status = "SCANNING DISABLED"
        self._detail = detail
        self.changed.emit()
        self._stop_process(managed=True)

    def releaseCycleHold(self):
        self._cycle_hold = False
        self.startScanner()

    def _stop_process(self, managed: bool):
        process = self._process
        if process is None or process.poll() is not None:
            self._running = False
            if managed or self._cycle_hold:
                self._status = "SCANNING DISABLED"
                self._detail = self._hold_detail
            else:
                self._status = "STOPPED"
            self.changed.emit()
            return
        if not managed:
            self._status = "STOPPING"
            self.changed.emit()
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError):
            pass

    def _read_stdout(self, process: subprocess.Popen):
        if process.stdout is None:
            return
        for line in process.stdout:
            payload = line.strip()
            if payload:
                self._scanLineReceived.emit(payload)

    def _read_stderr(self, process: subprocess.Popen):
        if process.stderr is None:
            return
        for line in process.stderr:
            payload = line.strip()
            if payload:
                print(f"[{self.sensor_name.upper()}] {payload}")
                self._diagnosticLineReceived.emit(payload)

    def _read_preview(self, preview_socket: socket.socket):
        while not self._closing and preview_socket is self._preview_socket:
            try:
                packet = preview_socket.recv(1_000_000)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(packet) < BARCODE_PREVIEW_HEADER.size:
                continue
            width, height = BARCODE_PREVIEW_HEADER.unpack_from(packet)
            payload = packet[BARCODE_PREVIEW_HEADER.size :]
            if len(payload) != width * height:
                continue
            self._previewPacketReceived.emit(bytes(payload), width, height)

    def _watch_process(self, process: subprocess.Popen):
        return_code = process.wait()
        if process is self._process:
            self._processStopped.emit(return_code)

    @Slot(bytes, int, int)
    def _handle_preview_packet(self, payload: bytes, width: int, height: int):
        image = QImage(
            payload,
            width,
            height,
            width,
            QImage.Format.Format_Grayscale8,
        ).copy()
        self.image_provider.set_image(image)
        self._preview_revision += 1
        if self._status == "STARTING CAMERA":
            self._status = "READY TO SCAN"
            self._detail = f"{self.display_name} barcode stream is active"
            self.changed.emit()
        self.previewChanged.emit()

    @Slot(str)
    def _handle_scan_line(self, payload: str):
        try:
            event = json.loads(payload)
            barcode = str(event.get("text", "")).strip()
            barcode_format = str(event.get("format", "Unknown"))
        except (json.JSONDecodeError, TypeError):
            prefix = "BARCODE SCANNED:"
            barcode = (
                payload[len(prefix) :].strip()
                if payload.startswith(prefix)
                else payload
            )
            barcode_format = "Unknown"
        if barcode:
            self.barcodeDetected.emit(
                self.sensor_name.upper(), barcode, barcode_format
            )

    @Slot(str)
    def _handle_diagnostic(self, payload: str):
        lowered = payload.lower()
        if payload.startswith("ERROR:") or "traceback" in lowered or "error:" in lowered:
            self._set_error(payload)
        elif "camera backend:" in lowered or payload.startswith("Camera:"):
            self._detail = payload
            self.changed.emit()

    @Slot(int)
    def _handle_process_stopped(self, return_code: int):
        self._close_preview_socket()
        self._process = None
        self._running = False
        if self._closing:
            return
        if self._cycle_hold:
            self._status = "SCANNING DISABLED"
            self._detail = self._hold_detail
        elif return_code in (0, -signal.SIGTERM):
            self._status = "STOPPED"
            self._detail = f"{self.display_name} scanner stopped"
        else:
            self._status = "CAMERA ERROR"
            if not self._detail or self._detail.startswith("Opening the auto-detected"):
                self._detail = (
                    f"{self.display_name} scanner exited with status {return_code}"
                )
        self.changed.emit()

    def _close_preview_socket(self):
        preview_socket = self._preview_socket
        self._preview_socket = None
        if preview_socket is not None:
            try:
                preview_socket.close()
            except OSError:
                pass

    def _set_error(self, message: str):
        self._status = "CAMERA ERROR"
        self._detail = message
        self.changed.emit()

    def shutdown(self):
        if self._closing:
            return
        self._closing = True
        process = self._process
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=2.0)
            except (ProcessLookupError, PermissionError, OSError, subprocess.TimeoutExpired):
                try:
                    process.kill()
                except OSError:
                    pass
        self._close_preview_socket()


class BarcodeService(CameraScannerService):
    """Primary IMX708 scanner plus shared accepted-scan/product UI model."""

    assignmentChanged = Signal()
    assignmentResolved = Signal()

    def __init__(self, args, image_provider: BarcodeImageProvider):
        super().__init__(
            args,
            image_provider,
            "imx708",
            "CAM1 IMX708",
            secondary_config=False,
            auto_index=0,
            preview_provider_key="barcode",
        )
        self.catalogue = ProductCatalogue(Path(args.barcode_products).expanduser())
        self.options = ProductOptions(Path(args.product_options).expanduser())
        self._product_name = "Waiting for a scan…"
        self._barcode = ""
        self._barcode_format = ""
        self._scan_time = ""
        self._source_camera = ""
        self._scan_count = 0
        self._history: deque[tuple[str, str, str, str, str]] = deque(maxlen=20)
        self._pending_unknown = ""
        self._assignment_visible = False
        self._assignment_stage = ""
        self._assignment_title = ""
        self._assignment_choices: list[str] = []
        self._selected_brand = ""
        self._selected_subbrand = ""
        self._selected_capacity = ""
        self._selected_product_name = ""

    @Property(str, notify=CameraScannerService.changed)
    def productName(self) -> str:
        return self._product_name

    @Property(str, notify=CameraScannerService.changed)
    def barcodeNumber(self) -> str:
        return self._barcode

    @Property(str, notify=CameraScannerService.changed)
    def barcodeFormat(self) -> str:
        return self._barcode_format

    @Property(str, notify=CameraScannerService.changed)
    def scanTime(self) -> str:
        return self._scan_time

    @Property(str, notify=CameraScannerService.changed)
    def sourceCamera(self) -> str:
        return self._source_camera

    @Property(int, notify=CameraScannerService.changed)
    def scanCount(self) -> int:
        return self._scan_count

    @Property(str, notify=CameraScannerService.changed)
    def historyText(self) -> str:
        if not self._history:
            return "No scans yet"
        return "\n".join(
            f"{time_text}   {product_name}   [{camera}/{barcode_format}]   {barcode}"
            for time_text, product_name, barcode_format, barcode, camera in self._history
        )

    @Property(bool, notify=CameraScannerService.changed)
    def canAssignUnknown(self) -> bool:
        return bool(self._pending_unknown)

    @Property(bool, notify=assignmentChanged)
    def assignmentVisible(self) -> bool:
        return self._assignment_visible

    @Property(str, notify=assignmentChanged)
    def assignmentStage(self) -> str:
        return self._assignment_stage

    @Property(str, notify=assignmentChanged)
    def assignmentTitle(self) -> str:
        return self._assignment_title

    @Property(str, notify=assignmentChanged)
    def assignmentBreadcrumb(self) -> str:
        parts = [
            value
            for value in (
                self._selected_brand,
                self._selected_subbrand,
                self._selected_capacity,
            )
            if value
        ]
        return "  ›  ".join(parts) if parts else f"Barcode: {self._pending_unknown}"

    @Property("QVariantList", notify=assignmentChanged)
    def assignmentChoices(self):
        return list(self._assignment_choices)

    @Property(str, notify=assignmentChanged)
    def assignmentProductName(self) -> str:
        return self._selected_product_name

    def recordAcceptedScan(
        self,
        source_camera: str,
        barcode: str,
        barcode_format: str,
    ) -> str:
        now_text = time.strftime("%H:%M:%S")
        product_name = self.catalogue.find(barcode)
        display_name = product_name or "Unknown product"
        self._barcode = barcode
        self._barcode_format = barcode_format
        self._scan_time = now_text
        self._source_camera = source_camera
        self._product_name = display_name
        self._scan_count += 1
        self._history.appendleft(
            (now_text, display_name, barcode_format, barcode, source_camera)
        )
        self._status = "BARCODE ACCEPTED" if product_name else "UNKNOWN BARCODE"
        self._detail = (
            f"Accepted by {source_camera}; recorded before flap unlock"
            if product_name
            else f"Accepted by {source_camera}; assign this barcode before the next bottle"
        )
        self.changed.emit()
        if product_name is None:
            self._pending_unknown = barcode
            self.beginUnknownAssignment()
        return display_name

    def recordRejectedUnknownScan(
        self,
        source_camera: str,
        barcode: str,
        barcode_format: str,
    ):
        """Display/assign an unknown code without opening the machine."""
        now_text = time.strftime("%H:%M:%S")
        self._barcode = barcode
        self._barcode_format = barcode_format
        self._scan_time = now_text
        self._source_camera = source_camera
        self._product_name = "Unknown product — not accepted"
        self._history.appendleft(
            (
                now_text,
                "Unknown product — not accepted",
                barcode_format,
                barcode,
                source_camera,
            )
        )
        self._status = "UNKNOWN BARCODE"
        self._detail = (
            f"Detected by {source_camera}; flap remains locked until the "
            "barcode is assigned or cancelled"
        )
        self._pending_unknown = barcode
        self.changed.emit()
        self.beginUnknownAssignment()

    @Slot()
    def beginUnknownAssignment(self):
        if not self._pending_unknown:
            return
        self._selected_brand = ""
        self._selected_subbrand = ""
        self._selected_capacity = ""
        self._selected_product_name = ""
        try:
            choices = self.options.brands()
        except (OSError, ValueError, csv.Error) as exc:
            choices = []
            self._detail = f"Product options error: {exc}"
            self.changed.emit()
        self._assignment_visible = True
        self._assignment_stage = "brand" if choices else "manual"
        self._assignment_title = "Select brand" if choices else "Enter product name"
        self._assignment_choices = choices
        self.assignmentChanged.emit()

    @Slot(str)
    def chooseAssignmentOption(self, choice: str):
        if self._assignment_stage == "brand":
            self._selected_brand = choice
            self._assignment_stage = "subbrand"
            self._assignment_title = "Select subbrand"
            self._assignment_choices = self.options.subbrands(choice)
        elif self._assignment_stage == "subbrand":
            self._selected_subbrand = choice
            self._assignment_stage = "capacity"
            self._assignment_title = "Select capacity"
            self._assignment_choices = self.options.capacities(
                self._selected_brand, self._selected_subbrand
            )
        elif self._assignment_stage == "capacity":
            self._selected_capacity = choice
            self._selected_product_name = self.options.resolve(
                self._selected_brand,
                self._selected_subbrand,
                self._selected_capacity,
            )
            self._assignment_stage = "confirm"
            self._assignment_title = "Confirm product"
            self._assignment_choices = []
        self.assignmentChanged.emit()

    @Slot()
    def assignmentBack(self):
        if self._assignment_stage in ("brand", "manual"):
            self.cancelAssignment()
            return
        if self._assignment_stage == "subbrand":
            self._selected_brand = ""
            self._assignment_stage = "brand"
            self._assignment_title = "Select brand"
            self._assignment_choices = self.options.brands()
        elif self._assignment_stage == "capacity":
            self._selected_subbrand = ""
            self._assignment_stage = "subbrand"
            self._assignment_title = "Select subbrand"
            self._assignment_choices = self.options.subbrands(self._selected_brand)
        elif self._assignment_stage == "confirm":
            self._selected_capacity = ""
            self._selected_product_name = ""
            self._assignment_stage = "capacity"
            self._assignment_title = "Select capacity"
            self._assignment_choices = self.options.capacities(
                self._selected_brand, self._selected_subbrand
            )
        self.assignmentChanged.emit()

    @Slot()
    def confirmAssignment(self):
        if self._selected_product_name:
            self._save_assignment(
                self._selected_product_name,
                self._selected_brand,
                self._selected_subbrand,
                self._selected_capacity,
            )

    @Slot(str)
    def saveManualAssignment(self, product_name: str):
        name = product_name.strip()
        if not name:
            self._detail = "Enter a product name before saving"
            self.changed.emit()
            return
        self._save_assignment(name, "", "", "")

    def _save_assignment(
        self,
        product_name: str,
        brand: str,
        subbrand: str,
        capacity: str,
    ):
        barcode = self._pending_unknown
        if not barcode:
            return
        try:
            self.catalogue.save_mapping(
                barcode,
                product_name,
                brand=brand,
                subbrand=subbrand,
                capacity=capacity,
            )
        except (OSError, ValueError, csv.Error) as exc:
            self._set_error(f"Could not save product mapping: {exc}")
            return

        updated: deque[tuple[str, str, str, str, str]] = deque(maxlen=20)
        for time_text, old_name, barcode_format, history_barcode, camera in self._history:
            if history_barcode == barcode and old_name == "Unknown product":
                old_name = product_name
            updated.append(
                (time_text, old_name, barcode_format, history_barcode, camera)
            )
        self._history = updated
        self._product_name = product_name
        self._pending_unknown = ""
        self._assignment_visible = False
        self._assignment_stage = ""
        self._assignment_choices = []
        self._status = "PRODUCT SAVED"
        self._detail = f"Saved {barcode} to {self.catalogue.path.name}"
        self.assignmentChanged.emit()
        self.changed.emit()
        self.assignmentResolved.emit()

    @Slot()
    def cancelAssignment(self):
        self._assignment_visible = False
        self._assignment_stage = ""
        self._assignment_choices = []
        self._pending_unknown = ""
        self._status = "UNKNOWN NOT ACCEPTED"
        self._detail = "Flap stayed locked; scanning can resume"
        self.assignmentChanged.emit()
        self.changed.emit()
        self.assignmentResolved.emit()

    @Slot()
    def clearHistory(self):
        self._history.clear()
        self._scan_count = 0
        self._product_name = "Waiting for a scan…"
        self._barcode = ""
        self._barcode_format = ""
        self._scan_time = ""
        self._source_camera = ""
        self.changed.emit()


class SecondaryCameraService(CameraScannerService):
    """CAM0 IMX708 barcode scanner and independent preview service."""

    def __init__(self, args, image_provider: BarcodeImageProvider):
        super().__init__(
            args,
            image_provider,
            "imx708",
            "CAM0 IMX708",
            secondary_config=True,
            auto_index=1,
            preview_provider_key="secondary",
        )

    @Slot()
    def startCamera(self):
        self.startScanner()

    @Slot()
    def stopCamera(self):
        self.stopScanner()


class BottleTransactionLog:
    """Durable append-only record written before the flap is unlocked."""

    FIELDNAMES = [
        "transaction_id",
        "scanned_at_utc",
        "scanned_at_local",
        "camera",
        "barcode_format",
        "barcode",
        "product_name",
        "passage_confirmation",
        "cycle_state",
    ]

    def __init__(self, path: Path):
        self.path = path
        self._lock = threading.Lock()

    def append(
        self,
        camera: str,
        barcode: str,
        barcode_format: str,
        product_name: str,
    ) -> str:
        transaction_id = uuid.uuid4().hex
        now_utc = datetime.now(timezone.utc)
        row = {
            "transaction_id": transaction_id,
            "scanned_at_utc": now_utc.isoformat(timespec="milliseconds").replace(
                "+00:00", "Z"
            ),
            "scanned_at_local": datetime.now().astimezone().isoformat(
                timespec="milliseconds"
            ),
            "camera": camera,
            "barcode_format": barcode_format,
            "barcode": barcode,
            "product_name": product_name,
            "passage_confirmation": "pending_ir_sensor",
            "cycle_state": "scan_recorded_before_unlock",
        }
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            needs_header = not self.path.exists() or self.path.stat().st_size == 0
            with self.path.open("a", newline="", encoding="utf-8") as target:
                writer = csv.DictWriter(target, fieldnames=self.FIELDNAMES)
                if needs_header:
                    writer.writeheader()
                writer.writerow(row)
                target.flush()
                os.fsync(target.fileno())
        return transaction_id


class BottleCycleCoordinator(QObject):
    """Atomic first-camera-wins scan gate and flap lock transaction state."""

    changed = Signal()

    def __init__(
        self,
        controller: MachineController,
        barcode: BarcodeService,
        secondary: SecondaryCameraService,
        transaction_log: BottleTransactionLog,
        beep: bool = True,
        allow_unknown_barcodes: bool = False,
    ):
        super().__init__()
        self.controller = controller
        self.barcode = barcode
        self.secondary = secondary
        self.transaction_log = transaction_log
        self.beep = beep
        self.allow_unknown_barcodes = allow_unknown_barcodes
        self._state = "STOPPED"
        self._detail = "Bottle cycle is not armed"
        self._scanning_enabled = False
        self._last_transaction_id = ""
        self._last_camera = ""
        self._state_started = time.monotonic()
        self._locking_for_rearm = False
        self._saw_flap_open = False
        self._expected_open_count = 0
        self._shutdown = False

        barcode.barcodeDetected.connect(self._barcode_detected)
        secondary.barcodeDetected.connect(self._barcode_detected)
        barcode.assignmentResolved.connect(self._assignment_resolved)

        self._timer = QTimer(self)
        self._timer.setInterval(50)
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    @Property(str, notify=changed)
    def state(self) -> str:
        return self._state

    @Property(str, notify=changed)
    def detail(self) -> str:
        return self._detail

    @Property(bool, notify=changed)
    def scanningEnabled(self) -> bool:
        return self._scanning_enabled

    @Property(str, notify=changed)
    def lastTransactionId(self) -> str:
        return self._last_transaction_id

    @Property(str, notify=changed)
    def lastCamera(self) -> str:
        return self._last_camera

    @Property(bool, notify=changed)
    def canForceRearm(self) -> bool:
        status = self.controller.get_status()
        return (
            not self._scanning_enabled
            and not status.flap_open
            and not status.solenoid_busy
        )

    def _set_state(self, state: str, detail: str):
        changed = state != self._state or detail != self._detail
        self._state = state
        self._detail = detail
        self._state_started = time.monotonic()
        if changed:
            print(f"[CYCLE] {state}: {detail}")
            self.changed.emit()

    def _hold_scanners(self, detail: str):
        self._scanning_enabled = False
        self.barcode.holdForCycle(detail)
        self.secondary.holdForCycle(detail)
        self.changed.emit()

    def _arm_scanners(self):
        self.controller.cancel_motor_trigger(log=False)
        if self.barcode.assignmentVisible:
            self._set_state(
                "WAITING FOR PRODUCT ASSIGNMENT",
                "Resolve or cancel the unknown barcode; the flap remains locked",
            )
            return
        self._scanning_enabled = True
        status = self.controller.get_status()
        detail = "Flap locked; first catalogue-valid barcode from either camera wins"
        if status.motor_on:
            detail += "; crusher timer is still completing"
        self._set_state("SCANNING ENABLED", detail)
        self.barcode.releaseCycleHold()
        self.secondary.releaseCycleHold()
        self.changed.emit()

    @Slot()
    def startSystem(self):
        if self._shutdown:
            return
        self.controller.cancel_motor_trigger(log=False)
        if not self.controller.motor_automation_enabled:
            self.controller.set_motor_automation_enabled(True)
        self._hold_scanners("Preparing and locking the flap")
        status = self.controller.get_status()
        if status.flap_open:
            self._set_state(
                "WAITING FOR FLAP CLOSE",
                "Close the flap; the system will lock it before enabling scanners",
            )
            self._locking_for_rearm = True
            return
        self._begin_lock(rearm=True)

    @Slot()
    def stopSystem(self):
        self.controller.cancel_motor_trigger(log=False)
        self._hold_scanners("Bottle cycle stopped by operator")
        self._set_state("STOPPED", "Scanners disabled; flap control remains available")

    @Slot()
    def forceRelockAndRearm(self):
        self.controller.cancel_motor_trigger(log=False)
        self._hold_scanners("Manual relock and rearm requested")
        status = self.controller.get_status()
        if status.flap_open:
            self._set_state(
                "WAITING FOR FLAP CLOSE",
                "Cannot relock while the flap reed reports OPEN",
            )
            self._locking_for_rearm = True
            return
        self._begin_lock(rearm=True)

    def _begin_lock(self, rearm: bool):
        self._locking_for_rearm = rearm
        accepted = self.controller.solenoid_lock_pulse()
        if not accepted:
            self._set_state(
                "CYCLE ERROR",
                "Lock pulse was not accepted; check solenoid enable/busy state",
            )
            return
        self._set_state(
            "RELOCKING FLAP" if rearm else "LOCKING FLAP",
            "Scanners remain disabled until the lock pulse completes",
        )

    @Slot(str, str, str)
    def _barcode_detected(
        self,
        source_camera: str,
        barcode_value: str,
        barcode_format: str,
    ):
        # Qt delivers both camera signals serially in this thread. Clearing the
        # shared gate first makes the first detection atomic even if both
        # cameras decoded the same bottle within one frame interval.
        if not self._scanning_enabled or self._state != "SCANNING ENABLED":
            print(
                f"[CYCLE] Ignored late scan from {source_camera}: {barcode_value}"
            )
            return

        self._scanning_enabled = False
        self._last_camera = source_camera
        self._saw_flap_open = False
        self.changed.emit()

        product_name = self.barcode.catalogue.find(barcode_value)
        if product_name is None and not self.allow_unknown_barcodes:
            self._hold_scanners(
                "Unknown barcode detected; flap remains locked and motor is not armed"
            )
            self.barcode.recordRejectedUnknownScan(
                source_camera, barcode_value, barcode_format
            )
            self._set_state(
                "WAITING FOR PRODUCT ASSIGNMENT",
                "Unknown barcode was not accepted; assign it or cancel to resume scanning",
            )
            return

        status = self.controller.get_status()
        if status.flap_open:
            self._hold_scanners("Unsafe scan state: flap reed reports OPEN")
            self._set_state(
                "CYCLE ERROR",
                "Barcode ignored because the flap was not closed and locked",
            )
            return
        if not status.motor_automation_enabled:
            self._hold_scanners("Crusher motor automation is disabled")
            self._set_state(
                "CYCLE ERROR",
                "Enable motor automation before accepting a bottle",
            )
            return

        recorded_name = product_name or "Unknown product"
        try:
            transaction_id = self.transaction_log.append(
                source_camera,
                barcode_value,
                barcode_format,
                recorded_name,
            )
        except (OSError, csv.Error) as exc:
            self._hold_scanners("Transaction recording failed")
            self._set_state(
                "CYCLE ERROR",
                f"Barcode was detected but could not be recorded: {exc}",
            )
            return

        self._last_transaction_id = transaction_id
        self.barcode.recordAcceptedScan(
            source_camera,
            barcode_value,
            barcode_format,
        )
        self._hold_scanners(
            f"Transaction {transaction_id[:8]} recorded; bottle cycle in progress"
        )
        self._expected_open_count = status.flap_open_count

        if not self.controller.arm_motor_for_next_flap_open():
            self._set_state(
                "CYCLE ERROR",
                "Scan was recorded but the reed-triggered motor cycle could not be armed",
            )
            return

        self._set_state(
            "SCAN RECORDED",
            f"{source_camera} accepted {barcode_value}; motor armed and unlocking flap",
        )
        if self.beep:
            print("\a", end="", flush=True)

        accepted = self.controller.solenoid_unlock_pulse()
        if not accepted:
            self.controller.cancel_motor_trigger(log=False)
            self._set_state(
                "CYCLE ERROR",
                "Scan was recorded but the unlock pulse was not accepted",
            )
            return
        self._set_state(
            "UNLOCKING FLAP",
            "Both scanners disabled; pushing flap OPEN starts one 15-second motor cycle",
        )

    @Slot()
    def _assignment_resolved(self):
        if self._state == "WAITING FOR PRODUCT ASSIGNMENT":
            self._arm_scanners()

    def _tick(self):
        if self._shutdown:
            return
        status = self.controller.get_status()
        flap_open_seen = (
            status.flap_open_count > self._expected_open_count or status.flap_open
        )

        if self._state == "UNLOCKING FLAP":
            if flap_open_seen:
                self._saw_flap_open = True
            if not status.solenoid_busy and (
                time.monotonic() - self._state_started >= SOLENOID_PULSE_SECONDS
            ):
                if self._saw_flap_open or flap_open_seen:
                    self._set_state(
                        "WAITING FOR FLAP CLOSE",
                        "Flap opened; crusher motor timer started; waiting for flap CLOSED",
                    )
                    self._locking_for_rearm = True
                else:
                    self._set_state(
                        "WAITING FOR FLAP OPEN",
                        "Push bottle through: reed OPEN will start the 15-second motor cycle",
                    )
            return

        if self._state == "WAITING FOR FLAP OPEN":
            if flap_open_seen:
                self._saw_flap_open = True
                self._set_state(
                    "WAITING FOR FLAP CLOSE",
                    "Flap opened; motor running on its 15-second timer; waiting for CLOSED",
                )
                self._locking_for_rearm = True
            return

        if self._state == "WAITING FOR FLAP CLOSE":
            if not status.flap_open:
                self._begin_lock(rearm=True)
            return

        if self._state in ("LOCKING FLAP", "RELOCKING FLAP"):
            if not status.solenoid_busy and (
                time.monotonic() - self._state_started >= SOLENOID_PULSE_SECONDS
            ):
                if status.flap_open:
                    self._set_state(
                        "WAITING FOR FLAP CLOSE",
                        "Flap still reports OPEN; scanners remain disabled",
                    )
                    return
                if self.barcode.assignmentVisible:
                    self._set_state(
                        "WAITING FOR PRODUCT ASSIGNMENT",
                        "Flap is locked; finish the unknown-product assignment",
                    )
                    return
                self._arm_scanners()

    def shutdown(self):
        if self._shutdown:
            return
        self._shutdown = True
        self._timer.stop()
        self.controller.cancel_motor_trigger(log=False)
        self._hold_scanners("Application shutting down")


# =============================================================================
# QT QUICK BACKEND
# =============================================================================


class QtMachineBackend(QObject):
    statusChanged = Signal()
    logChanged = Signal()

    def __init__(self, controller: MachineController):
        super().__init__()
        self.controller = controller
        self._status = controller.get_status()
        self._event_lines: list[str] = []
        self._shutdown_complete = False

        self.machine_timer = QTimer(self)
        self.machine_timer.setInterval(MACHINE_TICK_MS)
        self.machine_timer.timeout.connect(self.controller.tick)
        self.machine_timer.start()

        self.status_timer = QTimer(self)
        self.status_timer.setInterval(GUI_REFRESH_MS)
        self.status_timer.timeout.connect(self._refresh_status)
        self.status_timer.start()

        self.log_timer = QTimer(self)
        self.log_timer.setInterval(150)
        self.log_timer.timeout.connect(self._drain_logs)
        self.log_timer.start()

    # ------------------------------------------------------------------
    # Status properties
    # ------------------------------------------------------------------

    @Property(bool, notify=statusChanged)
    def simulationMode(self) -> bool:
        return self._status.simulation_mode

    @Property(bool, notify=statusChanged)
    def motorOn(self) -> bool:
        return self._status.motor_on

    @Property(str, notify=statusChanged)
    def motorText(self) -> str:
        return "RUNNING" if self._status.motor_on else "STOPPED"

    @Property(bool, notify=statusChanged)
    def motorAutoEnabled(self) -> bool:
        return self._status.motor_automation_enabled

    @Property(bool, notify=statusChanged)
    def motorTriggerArmed(self) -> bool:
        return self._status.motor_trigger_armed

    @Property(float, notify=statusChanged)
    def motorRemaining(self) -> float:
        return float(self._status.motor_remaining_seconds)

    @Property(str, notify=statusChanged)
    def motorRemainingText(self) -> str:
        if self._status.motor_on and self._status.motor_remaining_seconds <= 0:
            return "MANUAL"
        return f"{self._status.motor_remaining_seconds:.1f} s"

    @Property(bool, notify=statusChanged)
    def flapOpen(self) -> bool:
        return self._status.flap_open

    @Property(str, notify=statusChanged)
    def flapText(self) -> str:
        return "OPEN" if self._status.flap_open else "CLOSED"

    @Property(bool, notify=statusChanged)
    def solenoidEnabled(self) -> bool:
        return self._status.solenoid_enabled

    @Property(bool, notify=statusChanged)
    def solenoidBusy(self) -> bool:
        return self._status.solenoid_busy

    @Property(str, notify=statusChanged)
    def solenoidText(self) -> str:
        if self._status.solenoid_busy:
            return "ACTUATING"
        return "READY" if self._status.solenoid_enabled else "DISABLED"

    @Property(str, notify=statusChanged)
    def ledText(self) -> str:
        return self._status.led_color_text

    @Property(str, notify=statusChanged)
    def ledDetailText(self) -> str:
        detail = (
            f"{self._status.led_active_count}/{self._status.led_total_count} LEDs · "
            f"brightness {self._status.led_brightness}/31"
        )
        if self._status.led_error:
            detail += f" · {self._status.led_error}"
        return detail

    @Property(int, notify=statusChanged)
    def ledTotalCount(self) -> int:
        return int(self._status.led_total_count)

    @Property(int, notify=statusChanged)
    def ledActiveCount(self) -> int:
        return int(self._status.led_active_count)

    @Property(int, notify=statusChanged)
    def ledBrightness(self) -> int:
        return int(self._status.led_brightness)

    @Property(bool, notify=statusChanged)
    def ledAvailable(self) -> bool:
        return bool(self._status.led_available)

    @Property(str, notify=statusChanged)
    def boxPresentText(self) -> str:
        if self._status.box_present is None:
            return "UNKNOWN"
        return "PRESENT" if self._status.box_present else "NOT PRESENT"

    @Property(str, notify=statusChanged)
    def boxDistanceText(self) -> str:
        value = self._status.box_distance_mm
        return "-- mm" if value is None else f"{value} mm"

    @Property(float, notify=statusChanged)
    def fillPercent(self) -> float:
        value = self._status.fill_percent
        return -1.0 if value is None else float(value)

    @Property(str, notify=statusChanged)
    def fillPercentText(self) -> str:
        value = self._status.fill_percent
        return "-- %" if value is None else f"{value:.1f}%"

    @Property(str, notify=statusChanged)
    def fillDistanceText(self) -> str:
        value = self._status.fill_distance_mm
        return "-- mm" if value is None else f"{value} mm"

    @Property(bool, notify=statusChanged)
    def boxSensorHealthy(self) -> bool:
        return self._status.box_sensor_healthy

    @Property(bool, notify=statusChanged)
    def fillSensorHealthy(self) -> bool:
        return self._status.fill_sensor_healthy

    @Property(str, notify=statusChanged)
    def lastEvent(self) -> str:
        return self._status.last_event

    @Property(str, notify=logChanged)
    def eventLog(self) -> str:
        return "\n".join(self._event_lines[-30:])

    # ------------------------------------------------------------------
    # Controls exposed to QML
    # ------------------------------------------------------------------

    @Slot(bool)
    def setMotorAutoEnabled(self, enabled: bool):
        self.controller.set_motor_automation_enabled(bool(enabled))
        self._refresh_status()

    @Slot(bool)
    def setSolenoidEnabled(self, enabled: bool):
        self.controller.set_solenoid_enabled(bool(enabled))
        self._refresh_status()

    @Slot()
    def manualMotorOn(self):
        self.controller.manual_motor_on()
        self._refresh_status()

    @Slot()
    def manualMotorOff(self):
        self.controller.manual_motor_off()
        self._refresh_status()

    @Slot()
    def solenoidLock(self):
        self.controller.solenoid_lock_pulse()
        self._refresh_status()

    @Slot()
    def solenoidUnlock(self):
        self.controller.solenoid_unlock_pulse()
        self._refresh_status()

    @Slot()
    def allOutputsOff(self):
        self.controller.all_outputs_off()
        self._refresh_status()

    @Slot(int)
    def setLedTotalCount(self, count: int):
        self.controller.set_led_total_count(int(count))
        self._refresh_status()

    @Slot(int)
    def setLedActiveCount(self, count: int):
        self.controller.set_led_active_count(int(count))
        self._refresh_status()

    @Slot(int)
    def setLedBrightness(self, brightness: int):
        self.controller.set_led_brightness(int(brightness))
        self._refresh_status()

    @Slot(str)
    def setLedPreset(self, preset: str):
        preset = str(preset).strip().lower()
        colors = {
            "red": (255, 0, 0),
            "green": (0, 255, 0),
            "blue": (0, 0, 255),
            "white": (255, 255, 255),
            "amber": (255, 120, 0),
            "purple": (120, 0, 255),
        }
        if preset == "off":
            self.controller.led_off()
        elif preset in colors:
            self.controller.led_set_color(*colors[preset], label=preset.upper())
        else:
            self.controller.led_set_color(255, 255, 255, label="WHITE")
        self._refresh_status()

    @Slot()
    def simulateFlapOpen(self):
        if not self.controller.simulate:
            return
        open_level = (
            self.controller.reed_open_level
            if self.controller.reed_open_level is not None
            else FLAP_OPEN_LEVEL_VALUE
        )
        closed_level = (
            self.controller.reed_closed_level
            if self.controller.reed_closed_level is not None
            else (GPIO.LOW if open_level == GPIO.HIGH else GPIO.HIGH)
        )
        GPIO.simulate_set_pin(PIN_REED, open_level)
        QTimer.singleShot(
            350,
            lambda: GPIO.simulate_set_pin(PIN_REED, closed_level),
        )

    def _refresh_status(self):
        self._status = self.controller.get_status()
        self.statusChanged.emit()

    def _drain_logs(self):
        changed = False
        while True:
            try:
                line = self.controller.status_queue.get_nowait()
            except queue.Empty:
                break
            self._event_lines.append(line)
            self._event_lines = self._event_lines[-120:]
            changed = True
        if changed:
            self.logChanged.emit()

    def shutdown(self):
        if self._shutdown_complete:
            return
        self._shutdown_complete = True
        self.machine_timer.stop()
        self.status_timer.stop()
        self.log_timer.stop()
        self.controller.cleanup()


# =============================================================================
# QT WIDGETS DASHBOARD
# =============================================================================


class StatusCard(QFrame):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setObjectName("statusCard")
        self.setFrameShape(QFrame.Shape.StyledPanel)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(5)
        self.title_label = QLabel(title)
        self.title_label.setObjectName("cardTitle")
        self.value_label = QLabel("--")
        self.value_label.setObjectName("cardValue")
        self.detail_label = QLabel("")
        self.detail_label.setObjectName("cardDetail")
        self.detail_label.setWordWrap(True)
        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)
        layout.addWidget(self.detail_label)

    def update_values(self, value: str, detail: str = "", state: str = "normal"):
        self.value_label.setText(value)
        self.detail_label.setText(detail)
        colors = {
            "good": "#34d399",
            "warn": "#fbbf24",
            "bad": "#fb7185",
            "normal": "#e5edf7",
            "info": "#60a5fa",
        }
        self.value_label.setStyleSheet(f"color: {colors.get(state, colors['normal'])};")


class CameraPanel(QGroupBox):
    def __init__(self, title: str, service: CameraScannerService, provider: BarcodeImageProvider, parent=None):
        super().__init__(title, parent)
        self.service = service
        self.provider = provider
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout = QVBoxLayout(self)
        self.preview = QLabel("Waiting for camera")
        self.preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview.setMinimumSize(400, 225)
        self.preview.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.preview.setStyleSheet("background:#050b14; border:1px solid #334155; border-radius:8px;")
        self.status = QLabel("STOPPED")
        self.status.setObjectName("cameraStatus")
        self.detail = QLabel("")
        self.detail.setWordWrap(True)
        self.detail.setObjectName("mutedText")
        layout.addWidget(self.preview, 1)
        layout.addWidget(self.status)
        layout.addWidget(self.detail)

    def refresh(self):
        self.status.setText(self.service.status)
        self.detail.setText(self.service.detail)
        if self.service.status in ("READY TO SCAN", "STARTING CAMERA"):
            self.status.setStyleSheet("color:#34d399; font-weight:700;")
        elif self.service.status in ("CAMERA ERROR", "UNKNOWN BARCODE"):
            self.status.setStyleSheet("color:#fb7185; font-weight:700;")
        elif self.service.status == "SCANNING DISABLED":
            self.status.setStyleSheet("color:#fbbf24; font-weight:700;")
        else:
            self.status.setStyleSheet("color:#e5edf7; font-weight:700;")

        image = self.provider.get_image()
        if image.isNull() or self.preview.width() < 2 or self.preview.height() < 2:
            return
        pixmap = QPixmap.fromImage(image)
        pixmap = pixmap.scaled(
            self.preview.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.preview.setPixmap(pixmap)


class AssignmentDialog(QDialog):
    """Non-QML unknown-product assignment dialog."""

    def __init__(self, barcode: BarcodeService, parent=None):
        super().__init__(parent)
        self.barcode = barcode
        self.setWindowTitle("Assign Unknown Barcode")
        self.setModal(False)
        self.resize(560, 520)
        layout = QVBoxLayout(self)
        self.title = QLabel("")
        self.title.setStyleSheet("font-size:20px; font-weight:700;")
        self.breadcrumb = QLabel("")
        self.breadcrumb.setWordWrap(True)
        self.breadcrumb.setObjectName("mutedText")
        self.choices = QListWidget()
        self.manual = QLineEdit()
        self.manual.setPlaceholderText("Enter full product name")
        buttons = QHBoxLayout()
        self.back_button = QPushButton("BACK")
        self.primary_button = QPushButton("SELECT")
        self.cancel_button = QPushButton("CANCEL / RESUME")
        buttons.addWidget(self.back_button)
        buttons.addStretch(1)
        buttons.addWidget(self.cancel_button)
        buttons.addWidget(self.primary_button)
        layout.addWidget(self.title)
        layout.addWidget(self.breadcrumb)
        layout.addWidget(self.choices, 1)
        layout.addWidget(self.manual)
        layout.addLayout(buttons)

        self.choices.itemDoubleClicked.connect(lambda _item: self._primary())
        self.primary_button.clicked.connect(self._primary)
        self.back_button.clicked.connect(self.barcode.assignmentBack)
        self.cancel_button.clicked.connect(self._cancel)
        self.barcode.assignmentChanged.connect(self.refresh)

    def _cancel(self):
        self.barcode.cancelAssignment()
        self.hide()

    def reject(self):
        self._cancel()

    def _primary(self):
        stage = self.barcode.assignmentStage
        if stage in ("brand", "subbrand", "capacity"):
            item = self.choices.currentItem()
            if item is None:
                return
            self.barcode.chooseAssignmentOption(item.text())
        elif stage == "confirm":
            self.barcode.confirmAssignment()
        elif stage == "manual":
            name = self.manual.text().strip()
            if name:
                self.barcode.saveManualAssignment(name)

    def refresh(self):
        if not self.barcode.assignmentVisible:
            self.hide()
            return
        self.title.setText(self.barcode.assignmentTitle)
        self.breadcrumb.setText(self.barcode.assignmentBreadcrumb)
        stage = self.barcode.assignmentStage
        self.choices.clear()
        self.choices.addItems([str(value) for value in self.barcode.assignmentChoices])
        self.choices.setVisible(stage in ("brand", "subbrand", "capacity"))
        self.manual.setVisible(stage == "manual")
        if stage == "confirm":
            self.breadcrumb.setText(
                self.barcode.assignmentBreadcrumb
                + "\n\nProduct: "
                + self.barcode.assignmentProductName
            )
            self.primary_button.setText("CONFIRM")
        elif stage == "manual":
            self.primary_button.setText("SAVE")
        else:
            self.primary_button.setText("SELECT")
        if not self.isVisible():
            self.show()
            self.raise_()
            self.activateWindow()


class BB01Dashboard(QMainWindow):
    def __init__(
        self,
        backend: QtMachineBackend,
        barcode: BarcodeService,
        secondary: SecondaryCameraService,
        cycle: BottleCycleCoordinator,
        preview_provider: BarcodeImageProvider,
        secondary_provider: BarcodeImageProvider,
        args,
    ):
        super().__init__()
        self.backend = backend
        self.barcode = barcode
        self.secondary = secondary
        self.cycle = cycle
        self.args = args
        self.setWindowTitle("BB01 Machine Control")
        self.resize(1280, 800)
        self.setMinimumSize(1024, 680)
        self.setStyleSheet(self._style_sheet())

        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(10)

        header = QFrame()
        header.setObjectName("header")
        header_layout = QHBoxLayout(header)
        title_box = QVBoxLayout()
        title = QLabel("BRITEBOTTLE BB01")
        title.setObjectName("appTitle")
        subtitle = QLabel("Dual-camera barcode control · reed-gated crusher cycle")
        subtitle.setObjectName("mutedText")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        state_box = QVBoxLayout()
        self.cycle_state = QLabel("STOPPED")
        self.cycle_state.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.cycle_state.setObjectName("cycleState")
        self.cycle_detail = QLabel("")
        self.cycle_detail.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.cycle_detail.setWordWrap(True)
        self.cycle_detail.setObjectName("mutedText")
        state_box.addWidget(self.cycle_state)
        state_box.addWidget(self.cycle_detail)
        header_layout.addLayout(title_box, 1)
        header_layout.addLayout(state_box, 1)
        root.addWidget(header)

        self.tabs = QTabWidget()
        root.addWidget(self.tabs, 1)
        self.setCentralWidget(central)

        self._build_overview()
        self._build_cameras(preview_provider, secondary_provider)
        self._build_controls()
        self._build_logs()

        self.assignment_dialog = AssignmentDialog(barcode, self)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(150)
        self.refresh_timer.timeout.connect(self.refresh)
        self.refresh_timer.start()

        self.backend.statusChanged.connect(self.refresh)
        self.backend.logChanged.connect(self.refresh)
        self.barcode.changed.connect(self.refresh)
        self.secondary.changed.connect(self.refresh)
        self.cycle.changed.connect(self.refresh)
        self.refresh()

    @staticmethod
    def _style_sheet() -> str:
        return """
        QWidget { background:#0f172a; color:#e5edf7; font-size:14px; }
        QMainWindow, QDialog { background:#0f172a; }
        #header, #statusCard, QGroupBox, QTextEdit, QListWidget, QLineEdit {
            background:#172235; border:1px solid #2b3a52; border-radius:10px;
        }
        #header { padding:6px; }
        #appTitle { font-size:25px; font-weight:800; color:#ffffff; }
        #cycleState { font-size:22px; font-weight:800; color:#60a5fa; }
        #cardTitle { color:#93a4b8; font-size:12px; font-weight:700; }
        #cardValue { font-size:21px; font-weight:800; }
        #cardDetail, #mutedText { color:#93a4b8; }
        QTabWidget::pane { border:1px solid #2b3a52; border-radius:8px; }
        QTabBar::tab { background:#172235; padding:10px 18px; margin-right:3px; }
        QTabBar::tab:selected { background:#2563eb; color:white; }
        QGroupBox { margin-top:12px; padding-top:14px; font-weight:700; }
        QGroupBox::title { subcontrol-origin:margin; left:12px; padding:0 5px; }
        QPushButton { background:#2563eb; color:white; border:none; border-radius:7px; padding:10px 14px; font-weight:700; }
        QPushButton:hover { background:#3b82f6; }
        QPushButton:disabled { background:#334155; color:#94a3b8; }
        QPushButton#danger { background:#b91c1c; }
        QPushButton#warning { background:#b45309; }
        QCheckBox { spacing:8px; }
        QTextEdit, QListWidget, QLineEdit { padding:8px; }
        QProgressBar { border:1px solid #334155; border-radius:6px; text-align:center; background:#0b1220; }
        QProgressBar::chunk { background:#22c55e; border-radius:5px; }
        """

    def _build_overview(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        grid = QGridLayout()
        grid.setSpacing(12)
        self.motor_card = StatusCard("CRUSHER MOTOR")
        self.flap_card = StatusCard("FLAP / LOCK")
        self.barcode_card = StatusCard("LAST BARCODE")
        self.box_card = StatusCard("BOX SENSOR")
        self.fill_card = StatusCard("FILL LEVEL")
        self.led_card = StatusCard("LED STRIP")
        self.transaction_card = StatusCard("TRANSACTION")
        cards = [
            self.motor_card, self.flap_card, self.barcode_card,
            self.box_card, self.fill_card, self.led_card,
            self.transaction_card,
        ]
        for index, card in enumerate(cards):
            grid.addWidget(card, index // 3, index % 3)
        layout.addLayout(grid)
        self.last_event = QLabel("")
        self.last_event.setWordWrap(True)
        self.last_event.setObjectName("mutedText")
        layout.addWidget(self.last_event)
        layout.addStretch(1)
        self.tabs.addTab(page, "Overview")

    def _build_cameras(self, preview_provider, secondary_provider):
        page = QWidget()
        layout = QVBoxLayout(page)
        controls = QHBoxLayout()
        start = QPushButton("START SYSTEM")
        stop = QPushButton("STOP SCANNING")
        rearm = QPushButton("RELOCK + REARM")
        start.clicked.connect(self.cycle.startSystem)
        stop.clicked.connect(self.cycle.stopSystem)
        rearm.clicked.connect(self.cycle.forceRelockAndRearm)
        controls.addWidget(start)
        controls.addWidget(stop)
        controls.addWidget(rearm)
        controls.addStretch(1)
        note = QLabel("First catalogue-valid barcode wins; both scanners stop immediately")
        note.setObjectName("mutedText")
        controls.addWidget(note)
        layout.addLayout(controls)
        camera_row = QHBoxLayout()
        self.primary_camera = CameraPanel("CAM1 · IMX708 WIDE", self.barcode, preview_provider)
        self.secondary_camera = CameraPanel("CAM0 · IMX708 WIDE", self.secondary, secondary_provider)
        camera_row.addWidget(self.primary_camera, 1)
        camera_row.addWidget(self.secondary_camera, 1)
        layout.addLayout(camera_row, 1)
        self.tabs.addTab(page, "Cameras")

    def _build_controls(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        row = QHBoxLayout()
        motor_group = QGroupBox("Crusher motor")
        motor_layout = QVBoxLayout(motor_group)
        motor_on = QPushButton("MOTOR ON")
        motor_off = QPushButton("MOTOR OFF")
        motor_off.setObjectName("warning")
        motor_on.clicked.connect(self.backend.manualMotorOn)
        motor_off.clicked.connect(self.backend.manualMotorOff)
        self.motor_auto = QCheckBox("Reed-triggered automation enabled")
        self.motor_auto.clicked.connect(self.backend.setMotorAutoEnabled)
        self.motor_armed = QLabel("")
        self.motor_armed.setObjectName("mutedText")
        motor_layout.addWidget(motor_on)
        motor_layout.addWidget(motor_off)
        motor_layout.addWidget(self.motor_auto)
        motor_layout.addWidget(self.motor_armed)

        flap_group = QGroupBox("Flap lock")
        flap_layout = QVBoxLayout(flap_group)
        lock = QPushButton("LOCK PULSE")
        unlock = QPushButton("UNLOCK PULSE")
        lock.clicked.connect(self.backend.solenoidLock)
        unlock.clicked.connect(self.backend.solenoidUnlock)
        self.solenoid_enabled = QCheckBox("MD13S solenoid enabled")
        self.solenoid_enabled.clicked.connect(self.backend.setSolenoidEnabled)
        self.flap_status = QLabel("")
        self.flap_status.setObjectName("mutedText")
        flap_layout.addWidget(lock)
        flap_layout.addWidget(unlock)
        flap_layout.addWidget(self.solenoid_enabled)
        flap_layout.addWidget(self.flap_status)

        led_group = QGroupBox("DotStar LED strip")
        led_layout = QVBoxLayout(led_group)
        self.led_total_spin = QSpinBox()
        self.led_total_spin.setRange(1, DOTSTAR_MAX_LEDS)
        self.led_total_spin.setPrefix("Total LEDs: ")
        self.led_total_spin.valueChanged.connect(self.backend.setLedTotalCount)
        self.led_active_spin = QSpinBox()
        self.led_active_spin.setRange(0, DOTSTAR_MAX_LEDS)
        self.led_active_spin.setPrefix("LEDs to light: ")
        self.led_active_spin.valueChanged.connect(self.backend.setLedActiveCount)
        self.led_brightness_spin = QSpinBox()
        self.led_brightness_spin.setRange(0, 31)
        self.led_brightness_spin.setPrefix("Brightness: ")
        self.led_brightness_spin.valueChanged.connect(self.backend.setLedBrightness)
        led_layout.addWidget(self.led_total_spin)
        led_layout.addWidget(self.led_active_spin)
        led_layout.addWidget(self.led_brightness_spin)

        led_buttons1 = QHBoxLayout()
        red = QPushButton("RED")
        green = QPushButton("GREEN")
        blue = QPushButton("BLUE")
        red.clicked.connect(lambda: self.backend.setLedPreset("red"))
        green.clicked.connect(lambda: self.backend.setLedPreset("green"))
        blue.clicked.connect(lambda: self.backend.setLedPreset("blue"))
        led_buttons1.addWidget(red)
        led_buttons1.addWidget(green)
        led_buttons1.addWidget(blue)
        led_layout.addLayout(led_buttons1)

        led_buttons2 = QHBoxLayout()
        white = QPushButton("WHITE")
        amber = QPushButton("AMBER")
        purple = QPushButton("PURPLE")
        off = QPushButton("LED OFF")
        off.setObjectName("warning")
        white.clicked.connect(lambda: self.backend.setLedPreset("white"))
        amber.clicked.connect(lambda: self.backend.setLedPreset("amber"))
        purple.clicked.connect(lambda: self.backend.setLedPreset("purple"))
        off.clicked.connect(lambda: self.backend.setLedPreset("off"))
        led_buttons2.addWidget(white)
        led_buttons2.addWidget(amber)
        led_buttons2.addWidget(purple)
        led_buttons2.addWidget(off)
        led_layout.addLayout(led_buttons2)

        self.led_status = QLabel("")
        self.led_status.setWordWrap(True)
        self.led_status.setObjectName("mutedText")
        led_layout.addWidget(self.led_status)

        row.addWidget(motor_group, 1)
        row.addWidget(flap_group, 1)
        row.addWidget(led_group, 1)
        layout.addLayout(row)
        all_off = QPushButton("ALL OUTPUTS OFF")
        all_off.setObjectName("danger")
        all_off.setMinimumHeight(55)
        all_off.clicked.connect(self._all_off)
        layout.addWidget(all_off)
        if self.backend.simulationMode:
            simulate = QPushButton("SIMULATE FLAP OPEN/CLOSE")
            simulate.clicked.connect(self.backend.simulateFlapOpen)
            layout.addWidget(simulate)
        layout.addStretch(1)
        self.tabs.addTab(page, "Controls")

    def _build_logs(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(QLabel("Machine events"))
        self.event_log = QTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setFont(QFont("monospace"))
        layout.addWidget(self.event_log, 2)
        layout.addWidget(QLabel("Barcode history"))
        self.barcode_history = QTextEdit()
        self.barcode_history.setReadOnly(True)
        self.barcode_history.setFont(QFont("monospace"))
        layout.addWidget(self.barcode_history, 1)
        self.tabs.addTab(page, "Logs")

    def _all_off(self):
        self.cycle.stopSystem()
        self.backend.allOutputsOff()

    def refresh(self):
        self.cycle_state.setText(self.cycle.state)
        self.cycle_detail.setText(self.cycle.detail)
        state_color = "#34d399" if self.cycle.scanningEnabled else "#fbbf24"
        if "ERROR" in self.cycle.state:
            state_color = "#fb7185"
        self.cycle_state.setStyleSheet(f"color:{state_color}; font-size:22px; font-weight:800;")

        motor_detail = self.backend.motorRemainingText
        if self.backend.motorTriggerArmed:
            motor_detail += " · flap trigger armed"
        self.motor_card.update_values(
            self.backend.motorText,
            motor_detail,
            "good" if self.backend.motorOn else "normal",
        )
        flap_detail = self.backend.solenoidText
        self.flap_card.update_values(
            self.backend.flapText,
            flap_detail,
            "warn" if self.backend.flapOpen else "good",
        )
        barcode_detail = self.barcode.barcodeNumber
        if self.barcode.sourceCamera:
            barcode_detail += f" · {self.barcode.sourceCamera}"
        self.barcode_card.update_values(
            self.barcode.productName,
            barcode_detail,
            "info" if self.barcode.barcodeNumber else "normal",
        )
        self.box_card.update_values(
            self.backend.boxPresentText,
            self.backend.boxDistanceText,
            "good" if self.backend.boxSensorHealthy else "warn",
        )
        self.fill_card.update_values(
            self.backend.fillPercentText,
            self.backend.fillDistanceText,
            "good" if self.backend.fillSensorHealthy else "warn",
        )
        self.led_card.update_values(
            self.backend.ledText,
            self.backend.ledDetailText,
            "good" if self.backend.ledAvailable else "warn",
        )
        transaction = self.cycle.lastTransactionId[:12] if self.cycle.lastTransactionId else "NONE"
        self.transaction_card.update_values(transaction, self.cycle.lastCamera, "info")
        self.last_event.setText("Last event: " + self.backend.lastEvent)

        self.primary_camera.refresh()
        self.secondary_camera.refresh()
        self.motor_auto.blockSignals(True)
        self.motor_auto.setChecked(self.backend.motorAutoEnabled)
        self.motor_auto.blockSignals(False)
        self.solenoid_enabled.blockSignals(True)
        self.solenoid_enabled.setChecked(self.backend.solenoidEnabled)
        self.solenoid_enabled.blockSignals(False)

        self.led_total_spin.blockSignals(True)
        self.led_total_spin.setValue(self.backend.ledTotalCount)
        self.led_total_spin.blockSignals(False)
        self.led_active_spin.blockSignals(True)
        self.led_active_spin.setMaximum(self.backend.ledTotalCount)
        self.led_active_spin.setValue(self.backend.ledActiveCount)
        self.led_active_spin.blockSignals(False)
        self.led_brightness_spin.blockSignals(True)
        self.led_brightness_spin.setValue(self.backend.ledBrightness)
        self.led_brightness_spin.blockSignals(False)
        self.led_status.setText(self.backend.ledDetailText)

        self.motor_armed.setText(
            "Next valid flap-open trigger ARMED"
            if self.backend.motorTriggerArmed
            else "Motor trigger not armed"
        )
        self.flap_status.setText(
            f"Flap reed: {self.backend.flapText} · {self.backend.solenoidText}"
        )
        self.event_log.setPlainText(self.backend.eventLog)
        self.event_log.verticalScrollBar().setValue(self.event_log.verticalScrollBar().maximum())
        self.barcode_history.setPlainText(self.barcode.historyText)


# =============================================================================
# MAIN
# =============================================================================


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Run machine GPIO and I2C without physical hardware",
    )
    parser.add_argument(
        "--fullscreen",
        action="store_true",
        help="Start in fullscreen kiosk mode",
    )
    parser.add_argument(
        "--led-spi-device",
        default=DOTSTAR_SPI_DEVICE_DEFAULT,
        help="Linux SPI device for DotStar LEDs, normally /dev/spidev0.0",
    )
    parser.add_argument(
        "--led-total-count",
        type=int,
        default=DOTSTAR_TOTAL_LEDS_DEFAULT,
        help="Physical DotStar/APA102 LED count",
    )
    parser.add_argument(
        "--led-active-count",
        type=int,
        default=DOTSTAR_ACTIVE_LEDS_DEFAULT,
        help="Default number of LEDs to light from the start of the strip",
    )
    parser.add_argument(
        "--led-brightness",
        type=int,
        default=DOTSTAR_BRIGHTNESS_DEFAULT,
        help="DotStar global brightness, 0..31",
    )
    parser.add_argument(
        "--no-camera-autostart",
        dest="no_camera_autostart",
        action="store_true",
        help="Open the dashboard without automatically starting either camera",
    )
    parser.add_argument(
        "--no-barcode-autostart",
        dest="no_camera_autostart",
        action="store_true",
        help="Legacy alias for --no-camera-autostart",
    )
    parser.add_argument(
        "--camera-backend",
        choices=("imx708", "argus", "v4l2", "auto"),
        default="imx708",
    )
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--sensor-width", type=int, default=4608)
    parser.add_argument("--sensor-height", type=int, default=2592)
    parser.add_argument("--sensor-fps", type=int, default=8)
    parser.add_argument("--v4l2-device", default="auto")
    parser.add_argument("--gstreamer-pipeline", default=None)
    parser.add_argument("--barcode-width", type=int, default=1280)
    parser.add_argument("--barcode-height", type=int, default=720)
    parser.add_argument("--barcode-fps", type=float, default=8.0)
    parser.add_argument("--preview-width", type=int, default=400)
    parser.add_argument("--preview-height", type=int, default=225)
    parser.add_argument("--preview-fps", type=float, default=8.0)
    parser.add_argument("--secondary-v4l2-device", default="auto")
    parser.add_argument("--secondary-sensor-width", type=int, default=4608)
    parser.add_argument("--secondary-sensor-height", type=int, default=2592)
    parser.add_argument("--secondary-sensor-fps", type=int, default=8)
    parser.add_argument("--secondary-preview-width", type=int, default=400)
    parser.add_argument("--secondary-preview-height", type=int, default=225)
    parser.add_argument("--secondary-preview-fps", type=float, default=5.0)
    parser.add_argument("--secondary-barcode-width", type=int, default=1280)
    parser.add_argument("--secondary-barcode-height", type=int, default=720)
    parser.add_argument("--secondary-barcode-fps", type=float, default=8.0)
    parser.add_argument("--barcode-formats", default="retail")
    parser.add_argument("--barcode-roi", default="0.0,0.0,1.0,1.0")
    parser.add_argument(
        "--barcode-binarizer", choices=("local", "global"), default="local"
    )
    parser.add_argument("--barcode-cooldown", type=float, default=0.75)
    parser.add_argument("--barcode-products", default=str(DEFAULT_BARCODE_PRODUCTS))
    parser.add_argument("--product-options", default=str(DEFAULT_PRODUCT_OPTIONS))
    parser.add_argument(
        "--transactions",
        default=str(APP_DIR / "bottle_transactions.csv"),
        help="Append-only accepted bottle transaction CSV",
    )
    parser.add_argument("--exposure-us", type=int, default=None)
    parser.add_argument("--camera-gain", type=float, default=4.0)
    parser.add_argument("--barcode-rotate", action="store_true")
    parser.add_argument("--barcode-downscale", action="store_true")
    parser.add_argument("--barcode-invert", action="store_true")
    parser.add_argument(
        "--allow-unknown-barcodes",
        action="store_true",
        help=(
            "Allow unknown barcodes to unlock the flap. Default is safe: only "
            "barcodes already present in barcode_products.csv are accepted."
        ),
    )
    parser.add_argument("--no-barcode-beep", dest="barcode_beep", action="store_false")
    parser.set_defaults(barcode_beep=True)
    args = parser.parse_args()

    # Qt Widgets only. This deliberately avoids QtQml/QtQuick, which crashed
    # inside QQmlApplicationEngine.load() on the target PySide6 6.8/aarch64 setup.
    app = QApplication(sys.argv)
    app.setApplicationName("BB01 Machine + Barcode Control")
    app.setOrganizationName("BB01")

    load_gpio(args.simulate)
    controller = MachineController(
        simulate=args.simulate,
        led_spi_device=args.led_spi_device,
        led_total_count=args.led_total_count,
        led_active_count=args.led_active_count,
        led_brightness=args.led_brightness,
    )
    controller.setup()

    backend = QtMachineBackend(controller)
    preview_provider = BarcodeImageProvider()
    secondary_provider = BarcodeImageProvider()
    barcode = BarcodeService(args, preview_provider)
    secondary = SecondaryCameraService(args, secondary_provider)
    transaction_log = BottleTransactionLog(Path(args.transactions).expanduser())
    cycle = BottleCycleCoordinator(
        controller,
        barcode,
        secondary,
        transaction_log,
        beep=args.barcode_beep,
        allow_unknown_barcodes=args.allow_unknown_barcodes,
    )

    window = BB01Dashboard(
        backend,
        barcode,
        secondary,
        cycle,
        preview_provider,
        secondary_provider,
        args,
    )
    if args.fullscreen:
        window.showFullScreen()
    else:
        window.show()

    if not args.no_camera_autostart:
        QTimer.singleShot(700, cycle.startSystem)

    def handle_signal(_signum, _frame):
        app.quit()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    app.aboutToQuit.connect(cycle.shutdown)
    app.aboutToQuit.connect(secondary.shutdown)
    app.aboutToQuit.connect(barcode.shutdown)
    app.aboutToQuit.connect(backend.shutdown)

    exit_code = app.exec()
    cycle.shutdown()
    secondary.shutdown()
    barcode.shutdown()
    backend.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
