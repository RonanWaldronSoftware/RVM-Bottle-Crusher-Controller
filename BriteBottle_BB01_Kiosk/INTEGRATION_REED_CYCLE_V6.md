# BB01 reed-gated cycle v6

- CAM1 IMX708 and CAM0 IMX219 both scan barcodes.
- First catalogue-valid result atomically disables both scanners.
- Transaction is flushed to CSV before unlock.
- Accepted transaction arms one physical-pin-29 CLOSED→OPEN event.
- That event starts the Panasonic AQ-A motor output for 15 seconds.
- CLOSED triggers the lock pulse and scanner reactivation.
- Idle flap openings do not run the motor.
- Unknown barcodes stay locked and are not transaction-recorded.
