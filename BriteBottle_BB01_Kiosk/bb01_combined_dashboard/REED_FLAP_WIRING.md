# BB01 flap reed-switch wiring

This configuration assumes a **bare two-wire, normally-open reed switch** that
closes when the flap is resting in its normal closed position. Mount the magnet
on the moving flap and the reed body on the fixed frame.

## Jetson J12 connections

```text
J12 physical pin 17 — 3.3 V
        |
       10 kΩ external pull-up resistor
        |
        +---------------- J12 physical pin 29 — reed input
        |
     reed switch
        |
J12 physical pin 30 — GND
```

Equivalent ground pins 34 or 39 may also be used. Use **3.3 V only** for the
pull-up; never apply 5 V to physical pin 29. With the flap closed, the magnet
closes the reed and pin 29 reads LOW. Opening the flap opens the contact and the
10 kΩ pull-up makes pin 29 read HIGH.

The software debounces the contact for 80 ms. It starts the motor only when all
of these are true:

1. The machine is in `SCANNING ENABLED`.
2. A barcode already present in `barcode_products.csv` is accepted.
3. Both scanners are stopped and the flap unlock pulse is issued.
4. The reed changes from CLOSED/LOW to OPEN/HIGH.

The motor then runs for 15 seconds. Flap closure can relock and re-enable the
scanners before that timer expires; the motor still switches off exactly at its
15-second deadline.

If using a three-wire magnetic sensor module instead of a bare reed, verify that
its output is 3.3-V logic before connecting it to the Jetson. Do not connect a
5-V output to pin 29.
