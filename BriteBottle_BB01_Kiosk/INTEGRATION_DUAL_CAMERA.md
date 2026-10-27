# BB01 concurrent dual-camera integration

Active boot configuration:

- CAM0: Raspberry Pi Camera v2.1 / Sony IMX219
- CAM1: Raspberry Pi Camera Module 3 Wide / Sony IMX708
- Default boot entry: `BB01DualCam`

Dashboard services:

1. `fast_barcode_scanner_jetson.py` owns the IMX708 and decodes barcodes.
2. `imx219_preview_jetson.py` owns the IMX219 and publishes a secondary preview.
3. Each service is a separate process and can be restarted independently.

Run `./check_dual_cameras.sh` with the dashboard stopped before the first use.
