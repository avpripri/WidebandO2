# WidebandO2

An Arduino sketch for controlling a Bosch LSU 4.9 Wideband Lambda sensor with minimal parts

The board is uses the ProMicro (ATMEGA32U)
The project uses Visual Studio Code + PlatformIO but can easily be built in Arduino IDE by renaming main.cpp to main.ino.

## Update Jan-2021

Changes:
- PlatformIO
- ProMicro from Teensy 2.0 (just what I have handy)
- Redrafted the circuit for EasyEDA with gerbers, BOM & PnP for JLPCB orders 

## Required Libraries

* RunningAverage - http://playground.arduino.cc/Main/RunningAverage
* PIDLibrary - http://playground.arduino.cc/Code/PIDLibrary

