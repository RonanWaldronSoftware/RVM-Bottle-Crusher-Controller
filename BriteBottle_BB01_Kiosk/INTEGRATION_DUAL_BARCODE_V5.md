# BB01 Dual Camera Barcode Interlock v5

- IMX708 and IMX219 both decode barcodes.
- The first Qt signal received while the cycle is armed wins atomically.
- A durable CSV transaction is written before unlocking.
- Both scanner subprocesses stop during bottle passage.
- The existing flap reed drives OPEN -> CLOSED recognition.
- The latching solenoid relocks before scanners are restarted.
- A future IR chute sensor has a prepared transaction field.

The automatic cycle requires the flap reed on physical pin 29 to be wired and
calibrated as documented by `FLAP_OPEN_LEVEL_VALUE` in `controller_kiosk.py`.
