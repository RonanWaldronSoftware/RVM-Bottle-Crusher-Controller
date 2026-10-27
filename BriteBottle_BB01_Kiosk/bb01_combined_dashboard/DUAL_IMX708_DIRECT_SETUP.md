# BB01 v27 — two independent direct IMX708 cameras

This build is based on the uploaded v17 MCP23017 reed dashboard.

## Physical camera layout

Power the Jetson off before moving CSI ribbons.

```text
Jetson CAM0 CSI connector -> Pi Camera Module 3 Wide / IMX708
Jetson CAM1 CSI connector -> Pi Camera Module 3 Wide / IMX708
```

Do not use the UC-475 in this setup.

## Important DTB requirement

The dashboard can only use cameras that Linux has already detected as `/dev/video*`.

For this physical layout, the active device tree must declare:

```text
CAM0 = IMX708
CAM1 = IMX708
```

The older v17 BB01DualCam DTB declared:

```text
CAM0 = IMX219
CAM1 = IMX708
```

That old DTB is not enough for two IMX708 cameras. If `ls -l /dev/video*` shows no nodes or only one node, fix the camera DTB/ribbon detection first.

## Check before dashboard

Run:

```bash
./check_dual_imx708_direct.sh
```

You need a PASS before running the dashboard.

## Dashboard camera selection

The dashboard runs two scanner processes:

```text
CAM1 IMX708 = first auto-selected V4L2 camera, auto:0
CAM0 IMX708 = second auto-selected V4L2 camera, auto:1
```

If the device order is backwards, start manually with explicit devices, for example:

```bash
./run_dashboard.sh --v4l2-device /dev/video1 --secondary-v4l2-device /dev/video0 --no-camera-autostart
```

## Machine control unchanged

```text
Pin 31 = motor command
Pin 32 = MD13S solenoid PWM
Pin 33 = MD13S solenoid DIR
MCP23017 I2C bus 1 address 0x27 PA0 = flap reed input
PA0 = 1 -> flap open
PA0 = 0 -> flap closed
```
