# Kerbal Simpit Controller

Build guide and Arduino sketch for a custom Arduino Mega Kerbal Space Program control panel.

## Included

- `index.html` - GitHub Pages build guide
- `sketch_apr18a.ino` - Arduino Mega controller sketch
- `stl/` - place for 3D print STL files and print notes

## Hardware Summary

- Arduino Mega 2560
- 16x2 LCD with I2C backpack
- 4-digit common-cathode 7-segment display
- 1-digit common-cathode 7-segment display
- Analog joystick with push switch
- Two EC11 rotary encoders
- Buttons/toggles for stage, SAS, RCS, brakes, lights, gear, and action groups
- Status LEDs with current-limiting resistors

## 3D Print Warning

The current 3D model is a first draft. The original build required cutting and sawing parts of the printed model to make everything fit. Expect trimming, sanding, drilling, or other fit adjustments before final assembly.

## Main Pins

- LCD I2C: SDA 20, SCL 21
- Joystick: A0, A1, switch 46
- LCD encoder: 2, 3, switch 4
- Throttle encoder: 18, 19, switch 29
- Stage button: 10
- 4-digit display: digits 30-33, segments 34-41
- 1-digit display: segments 22-28

## Software

Install the Arduino libraries:

- KerbalSimpit
- LiquidCrystal_I2C
- Encoder
- SevSeg

Then upload `sketch_apr18a.ino` to an Arduino Mega 2560.
