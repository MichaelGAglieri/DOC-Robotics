# Reference hardware

Five commanded axes plus one servo gripper. J6 is not fitted in the reference arm. This documents the project configuration; check the labels and pinout of your own hardware before applying it.

| Axis | RAMPS/driver | STEP | DIR | Enable |
| --- | --- | ---: | ---: | --- |
| J1 | X / A4988 | 54 | 55 | 38, active LOW |
| J2 | Y signals / external DollaTek DM542 | 60 | 61 | External ENA unmanaged |
| J3 | Z / A4988 | 46 | 48 | 62, active LOW |
| J4 | E0 / A4988 | 26 | 28 | 24, active LOW |
| J5 | E1 / A4988 | 36 | 34 | 30, active LOW |

Configured conversion: 200 full steps/revolution, 1/8 microstep, 38.4:1 reduction: 61,440 pulses per output revolution, approximately 170.667 pulses/degree. A4988 jumper configuration: MS1/MS2 fitted, MS3 open. The DM542 configuration used in the project is 1,600 input pulses per motor revolution; confirm the switch table for the exact driver variant.

For J2 the reference connection is PUL−/DIR− to signal GND, PUL+ to D60 and DIR+ to D61. External ENA is not connected. Software STOP stops pulses and can leave holding torque. The unused RAMPS Y driver enable is held inactive.

Gripper reference: MG995 and EBG-30, signal D11, Timer1 at 50 Hz, open 1944 µs and closed 1150 µs. These are the tested project settings, not universal endpoints for every servo or assembly. Use a suitable external servo supply and common signal ground; do not power the servo from a Mega I/O pin or connect it to the motor supply.

The stable firmware and upload workflow are maintained in the [Robot Console repository](https://github.com/MichaelGAglieri/Robot-Console). It starts without motor movement and requires an operator reference before normal control. Driver current is set on the actual driver, not by increasing the app’s speed control.

Original mechanical files are available from the designer: see [SOURCES.md](SOURCES.md). No third-party STL, STEP, sliced files or datasheet copies are bundled in this repository.
