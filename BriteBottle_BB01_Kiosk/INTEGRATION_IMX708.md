# Integration with the installed BB01DualCam setup

This archive contains the kiosk code only. It assumes the Jetson already boots
with `DEFAULT BB01DualCam`, loads `nv_imx708`, and exposes IMX708 and IMX219 as
V4L2 devices.

For a new installation, use `bb01_combined_dashboard/install.sh`.

To update an existing kiosk while preserving learned barcode/product CSV files:

```bash
cd bb01_combined_dashboard
./upgrade_existing_kiosk.sh /absolute/path/to/the/existing/bb01_combined_dashboard
```

Then run `./check_imx708.sh` from the target directory.
