# BB01 DotStar / APA102 LED wiring

This code is for the Adafruit DotStar / APA102 style LED strip.

## Jetson GPIO header wiring

Use the Jetson SPI pins:

```text
Jetson physical pin 19 = SPI MOSI / DATA
Jetson physical pin 23 = SPI SCLK / CLOCK
Jetson physical pin 6  = GND
```

Wire:

```text
External 5 V LED PSU +  -> LED strip 5V
External 5 V LED PSU -  -> LED strip GND

Jetson pin 6 GND        -> LED strip GND / PSU -
Jetson pin 19 MOSI      -> LED strip DI / DATA IN
Jetson pin 23 SCLK      -> LED strip CI / CLOCK IN
```

## Strong recommendation

Because the Jetson signal is 3.3 V and the LED strip is powered at 5 V,
use a 3.3 V to 5 V logic buffer/level shifter:

```text
Jetson pin 19 MOSI -> 74AHCT input -> LED DI
Jetson pin 23 SCLK -> 74AHCT input -> LED CI
74AHCT VCC -> 5 V LED PSU +
74AHCT GND -> common GND
```

## Important

Do not power the LED strip from the Jetson 5 V pin.
Use a separate 5 V LED PSU. For a 72 LED strip, use at least a 5 V 5 A PSU.

Connect to the LED input end marked DI/CI, not DO/CO.
