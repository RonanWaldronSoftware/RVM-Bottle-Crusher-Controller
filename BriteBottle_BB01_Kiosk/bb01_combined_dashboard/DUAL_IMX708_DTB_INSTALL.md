# BB01 dual IMX708 device tree install

This package is for the direct two-camera layout:

```text
Jetson CAM0 -> Pi Camera Module 3 Wide / IMX708
Jetson CAM1 -> Pi Camera Module 3 Wide / IMX708
No UC-475
No IMX219
```

Run:

```bash
sudo ./install_dual_imx708_dtb.sh
sudo reboot
./verify_dual_imx708_dtb.sh
./check_dual_imx708_direct.sh
./run_dashboard.sh --no-camera-autostart
```

The installer starts from the current working `BB01DualCam` DTB and creates a new boot entry:

```text
DEFAULT BB01DualIMX708
```

It backs up `/boot/extlinux/extlinux.conf` and the source DTB before changing anything.

Recovery:

```bash
sudo ./restore_bb01_dualcam_boot.sh
sudo reboot
```

If the Jetson fails to detect `/dev/video*` after reboot, check ribbon orientation and run:

```bash
./verify_dual_imx708_dtb.sh
```
