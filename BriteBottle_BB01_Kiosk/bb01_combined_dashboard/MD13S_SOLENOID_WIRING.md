# MD13S Solenoid Wiring

## Control header

| MD13S terminal | Jetson connection |
|---|---|
| DIR | J12 physical pin 33 |
| PWM | J12 physical pin 32 |
| NC | Not connected |
| GND | Jetson GND, for example pin 34 or 39 |

## Power and solenoid terminals

| MD13S terminal | Connect to |
|---|---|
| + | Solenoid power supply + |
| - | Solenoid power supply - |
| MA | Solenoid wire 1 |
| MB | Solenoid wire 2 |

The Jetson does not power the solenoid. It only sends 3.3 V logic to DIR and PWM.
The dashboard pulses PWM for about 600 ms, then turns PWM off.
