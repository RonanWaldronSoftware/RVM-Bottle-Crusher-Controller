# Hardware commissioning checklist

1. Disconnect mechanical load where possible.
2. Confirm Crydom SSR motor input control is on physical BOARD pin 31.
3. Confirm RPWM is on physical pin 31 and LPWM on physical pin 32.
4. Wire the flap reed per REED_FLAP_WIRING.md: pin 29 input, 10 kΩ pull-up to 3.3 V, switch to GND.
5. Confirm the DFR0627 is visible on I2C bus 7 with `sudo i2cdetect -y 7`.
6. Start with `./run_dashboard.sh --no-barcode-autostart`.
7. Verify both sensor readings before operating outputs.
8. Tap ALL OUTPUTS OFF and confirm motor and bridge outputs are inactive.
9. Test MOTOR ON, then MOTOR OFF, with the Crydom load side safe/unloaded where possible.
10. Test UNLOCK and LOCK separately; each pulse should end after 600 ms.
11. With no accepted scan, open the flap and confirm the motor stays OFF.
12. Scan a known product, verify unlock, then open the flap and confirm the motor starts its 15-second timer.
13. Close the flap; verify lock and both scanners rearm while the motor stops only at its timer deadline.
14. Scan an unknown product; verify the flap stays locked and no transaction row is added. Reboot and repeat safe-output checks.


## Dual barcode transaction cycle

- Confirm the flap reed is connected to physical pin 29 and reads CLOSED before testing.
- Start the dashboard and verify it pulses LOCK before enabling both scanners.
- Present one barcode where both cameras can see it; verify only one transaction row is added.
- Verify both camera processes stop before the flap unlocks.
- Push the bottle through; verify reed OPEN starts the motor and reed CLOSED starts relock.
- Verify the lock pulse completes before both scanners restart.
- Verify `bottle_transactions.csv` contains the camera, barcode and `pending_ir_sensor`.
- Test an unknown barcode; the flap must remain locked, motor unarmed and no transaction row added.


## Crydom SSR control-side wiring

- J12 physical pin 31 -> door magnet safety switch -> Crydom input +
- Crydom input - -> Jetson GND, such as J12 physical pin 30, 34, or 39
- Door closed/safe = switch closed
- Door open/unsafe = switch open

Physical pin 31 is intentionally used for the Crydom SSR in this build. Legacy solenoid outputs are bypassed until the MD13S driver is installed.
