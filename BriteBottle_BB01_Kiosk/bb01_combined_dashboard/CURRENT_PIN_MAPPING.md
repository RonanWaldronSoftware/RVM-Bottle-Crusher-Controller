# BB01 Current Jetson / MCP23017 Pin Mapping — v17

## Jetson J12 physical pins

| Physical pin | Function |
|---:|---|
| 17 | 3.3 V to Waveshare MCP23017 VCC |
| 27 | I2C SDA to Waveshare MCP23017 SDA |
| 28 | I2C SCL to Waveshare MCP23017 SCL |
| 30 | GND to Waveshare MCP23017 GND |
| 31 | Motor request output to DFR0457 / Crydom chain |
| 32 | MD13S PWM / solenoid pulse enable |
| 33 | MD13S DIR / lock-unlock direction |
| 34 / 39 | GND options |

## Waveshare MCP23017 reed input

Detected/tested settings:

```text
I2C bus: 1
I2C address: 0x27
Input pin: PA0 / GPA0
```

Reed wiring:

```text
One reed wire   -> Waveshare PA0 / GPA0
Other reed wire -> Waveshare GND
```

Dashboard logic:

```text
PA0 = 1 -> flap OPEN
PA0 = 0 -> flap CLOSED
```

No external resistor is needed on the reed. The dashboard enables the MCP23017 internal pull-up on PA0.

## MD13S control

MD13S DIR -> Jetson physical pin 33  
MD13S PWM -> Jetson physical pin 32  
MD13S NC -> not connected  
MD13S GND -> Jetson GND  

## MD13S power/output

MD13S + -> solenoid PSU +  
MD13S - -> solenoid PSU -  
Solenoid wire 1 -> MD13S MA  
Solenoid wire 2 -> MD13S MB  

If lock/unlock are reversed, swap MA/MB or invert the direction constants in `controller_kiosk.py`.

## v27 camera layout

```text
Jetson CAM0 CSI -> Pi Camera Module 3 Wide / IMX708
Jetson CAM1 CSI -> Pi Camera Module 3 Wide / IMX708
```

No UC-475 is used in v27.

The active Jetson DTB must declare both CSI ports as IMX708 before the dashboard can show video.


## DotStar / APA102 LED strip

```text
Jetson pin 19 MOSI -> LED DI / DATA IN
Jetson pin 23 SCLK -> LED CI / CLOCK IN
Jetson GND -> LED strip GND / external 5 V PSU negative
External 5 V PSU positive -> LED strip 5V
```

The dashboard controls total LED count, active lit LED count, brightness, colour presets, and LED OFF from the Controls tab.
