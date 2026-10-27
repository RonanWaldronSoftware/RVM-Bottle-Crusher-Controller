# Crydom SSR + MS6 Door Magnet Wiring — Pin 31

This build uses Jetson J12 physical pin 31 as the motor request output. Legacy IBT-2 solenoid control is bypassed until the MD13S arrives.

## Control side

```text
Jetson physical pin 31 -> MS6 door magnet switch -> Crydom SSR input +
Crydom SSR input - -> Jetson GND, for example physical pin 34 or 39
```

Use the MS6 contact pair that is CLOSED when the door is closed/safe and OPEN when the door is open/unsafe.

## Test before connecting Crydom

Run:

```bash
./check_motor_pin31_output.sh
```

With the Crydom/MS6 disconnected from pin 31, measure pin 31 to Jetson GND. Expected values are about 3.3 V during HIGH and 0 V during LOW.

## Load side

Use the correct Crydom output type for the motor/load. DC motors require a DC-output SSR; AC loads require an AC-output SSR switching live.
