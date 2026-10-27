# BB01 Waveshare MCP23017 Reed Wiring — v17

This dashboard reads the flap reed from the Waveshare MCP23017 board instead of Jetson GPIO pin 29/16.

## Jetson to Waveshare MCP23017

Use the I2C bus that was verified by `i2cdetect`:

| Jetson physical pin | Waveshare MCP23017 pin |
|---:|---|
| 17, 3.3 V | VCC |
| 30, GND | GND |
| 27, SDA | SDA |
| 28, SCL | SCL |

The board was detected as:

```text
I2C bus: 1
Address: 0x27
```

## Reed to Waveshare MCP23017

```text
One reed wire   -> Waveshare PA0 / GPA0
Other reed wire -> Waveshare GND
```

No external resistor is used on the reed. The dashboard enables the MCP23017 internal pull-up on PA0.

## Expected reed logic

```text
PA0 = 1 -> flap OPEN
PA0 = 0 -> flap CLOSED
```

In the normal barcode workflow, a valid barcode scan arms the motor. The first debounced PA0 transition from 0 to 1 starts the 15-second crusher motor timer. PA0 returning to 0 means the flap is closed and scanners can resume/relock according to the existing dashboard logic.

## Standalone test

From the dashboard folder:

```bash
./check_mcp23017_reed.sh
```

Expected:

```text
flap open   -> PA0 = 1
flap closed -> PA0 = 0
```
