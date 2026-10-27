
## v17 MCP23017 flap reed input

This build reads the flap reed from the Waveshare MCP23017 on I2C bus 1 at address 0x27, input PA0/GPA0. PA0=1 means flap OPEN and PA0=0 means flap CLOSED. The dashboard enables the MCP23017 internal pull-up, so the reed wiring is simply PA0 -> reed -> GND.

Run `./check_mcp23017_reed.sh` before starting the dashboard to confirm PA0 changes correctly.

# BB01 Qt Widgets Dual-Camera Dashboard v8

This complete build preserves the working BB01 machine controller and dual raw-V4L2 barcode scanner processes, but replaces the QML user interface with Qt Widgets. It does not import `PySide6.QtQml` or `PySide6.QtQuick`, avoiding the `QQmlApplicationEngine.load()` segmentation fault seen on the target Jetson with PySide6 6.8.0.2.

## Run

```bash
chmod +x *.sh *.py
./install.sh
./run_dashboard.sh
```

Start without automatically locking/starting the scanners:

```bash
./run_dashboard.sh --no-camera-autostart
```

## Bottle cycle

1. Idle: flap closed/locked and both scanners active.
2. First catalogue-valid barcode wins and is synchronously recorded.
3. Both scanner subprocesses stop.
4. Flap unlock pulse runs.
5. Reed CLOSED->OPEN transition starts the crusher for 15 seconds.
6. Reed returns CLOSED; flap lock pulse runs and scanners reactivate.
7. Motor stops at its own 15-second deadline.

## Camera mapping

The application identifies cameras by V4L2 card name, not by fixed node number.

- IMX708: barcode scanner and preview
- IMX219: barcode scanner and preview

## Reed wiring

- Physical pin 29: reed input
- Physical pin 30: ground
- 10 kOhm pull-up from physical pin 29 to physical pin 17 (3.3 V)
- Flap closed = LOW; flap open = HIGH


## v9 product-assignment fix

The assignment list is now refreshed only when the assignment state changes. The main 150 ms dashboard refresh no longer clears the selected brand, subbrand, or capacity before SELECT is pressed.


## v10 reed auto-calibration

Start the application with the flap fully closed. The controller learns the initial pin-29 level as CLOSED and treats the opposite level as OPEN. Startup logs show both learned levels. This supports either reed polarity while retaining the external 10 kΩ pull-up wiring.


## v11 Crydom SSR motor output

The motor output is physical BOARD pin 31. This uses a direct Crydom SSR input control output. Keep the door magnet safety switch in series with the Crydom input so the SSR cannot energise unless the door is closed/safe.

Control-side wiring:

```text
Jetson J12 pin 31  -> door magnet safety switch -> Crydom input +
Crydom input -    -> Jetson GND, such as pin 30/34/39
```

The software drives pin 31 HIGH for motor ON and LOW for motor OFF. Pins 31 and 32 remain the solenoid RPWM/LPWM pins. Pin 29 remains the flap reed sensor input.


## v12 packaging fix

This package intentionally excludes `.venv`. Run `./install.sh` on the Jetson so the Python virtual environment is created for aarch64. The installer removes any existing `.venv` first to avoid architecture mismatch errors.


## v13 pin 31 pinmux overlay

Run `sudo ./no pin-7 overlay required` once, then reboot. This installs a boot-time overlay that configures J12 physical pin 31 / PAC.06 as GPIO for the Crydom SSR motor output. After reboot, run `./check_motor_pin31_output.sh` and measure pin 31 to GND.


## v16 MD13S solenoid update

Pin 31 remains the motor request output for the DFR0457/Crydom AC motor chain.
The latching solenoid is now controlled by an MD13S: pin 32 to PWM, pin 33 to DIR, MD13S GND to Jetson GND.
Run `./check_md13s_pins.sh` before connecting the solenoid, then use `sudo -E env JETSON_MODEL_NAME=JETSON_ORIN_NANO python3 test_md13s_solenoid.py` for manual lock/unlock testing.
