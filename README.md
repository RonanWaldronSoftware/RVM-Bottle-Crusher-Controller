# RVM Bottle Collection Kiosk + Crusher Controller "BriteBottle"

BriteBottle is a local control application for an automated bottle-collection machine. It combines barcode recognition, flap locking, crusher-motor control, sensor monitoring, and operator controls in one desktop dashboard.

The aim is to accept recognized products in a controlled sequence, keep a record of accepted scans, and provide staff with a way to monitor and test the machine.

This repository contains the **v34 kiosk software**, designed around an NVIDIA Jetson Orin Nano and connected cameras, sensors, and machine-control hardware.

## Main functions

| Area | Function |
| --- | --- |
| Barcode scanning | Uses two IMX708 cameras to detect bottle barcodes from either view. |
| Product recognition | Checks scanned barcodes against a local product catalogue. |
| Unknown products | Keeps the flap locked for unrecognized barcodes by default and supports assigning products to the catalogue. |
| Bottle intake | Coordinates barcode acceptance, scanner pause, flap unlocking, and relocking. |
| Crusher control | Starts a timed motor cycle when the flap-open sensor reports the expected movement after an accepted scan. |
| Machine monitoring | Displays system status, camera views, flap state, and selected sensor and control information. |
| Operator controls | Provides start/stop, relock/rearm, motor, flap-lock, and all-outputs-off controls. |
| LED lighting | Controls a compatible DotStar/APA102 LED strip, including colour, brightness, and the number of active LEDs. |
| Records | Saves accepted barcode transactions to a local CSV file. |
| Testing | Includes simulation, component checks, and hardware commissioning scripts. |

## How a bottle is processed

1. The machine starts with its flap closed and locked. Both camera scanners can look for a barcode.
2. The first recognized barcode is accepted; the application prevents the two cameras from creating duplicate transactions for the same intake cycle.
3. The accepted scan is written to the local transaction file before the flap is unlocked.
4. Both scanners pause while the flap is unlocked for the bottle to enter.
5. Opening the flap is detected by a magnetic reed switch. After a valid scan, this movement starts the configured **15-second crusher-motor cycle**.
6. When the flap closes, the software relocks it and prepares the cameras for the next bottle. The motor follows its own timed stop.

An unrecognized barcode does not normally unlock the flap or arm the motor. The operator can resolve a product assignment or cancel that scan.

## Operator dashboard

The application has four main screens:

- **Overview:** Current intake-cycle status, machine state, and recent activity.
- **Cameras:** Live views and scanning status for both cameras.
- **Controls:** Manual motor and flap-lock functions, machine reset/rearm controls, and LED strip settings.
- **Logs:** Machine events and barcode activity for troubleshooting.

The software can run in a window or full-screen kiosk mode.

## Hardware and operating environment

The project targets an **NVIDIA Jetson Orin Nano** running Linux and uses Python with a Qt desktop interface. Its current hardware integration includes:

- Two IMX708 cameras for barcode scanning.
- A flap-position reed switch connected through an MCP23017 input device.
- A motor-control output through an external solid-state relay and safety-switch arrangement.
- An MD13S driver for the flap-lock solenoid.
- I²C-connected distance-sensor hardware.
- Optional DotStar/APA102 LED strip lighting.

These devices require the appropriate Jetson camera setup, drivers, wiring, power supplies, and operating-system permissions. See the included current camera, wiring, and hardware-check documents before connecting outputs.

**Safety:** This software controls moving machinery. Hardware interlocks, guards, emergency isolation, and electrical protection must be verified independently. Software buttons and timers must not be treated as substitutes for physical safety measures.

## Data files

| File | Purpose |
| --- | --- |
| `barcode_products.csv` | Known barcodes and their associated products. |
| `product_options.csv` | Product choices used during barcode assignment. |
| `bottle_transactions.csv` | Local record of accepted scans, including time, camera, barcode, and product information. |

The transaction file records acceptance of a scan. Its initial passage-confirmation value is **pending**, so a saved row should not automatically be interpreted as independently confirmed bottle collection or completed crushing.

## Project structure

The main application is `bb01_combined_dashboard/controller_kiosk.py`. The same directory contains the camera scanner, startup scripts, CSV catalogues, hardware checks, and installation instructions.

