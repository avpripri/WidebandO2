##WidebandO2

An Arduino sketch for controlling a Bosch LSU 4.9 Wideband Lambda sensor with minimal parts

The board is uses the ProMicro (ATMEGA32U)
The project uses Visual Studio Code + PlatformIO but can easily be built in Arduino IDE by loadin the .ino file there.

##Update Jan-2021

I was unable to get this older eagle file to load, and it was pretty simple, so I rebuilt it in EasyEDA.  All files added to fabrication folder and include gerber, bom + pnp files.  Upload to JLPCB, as of htis writing, 5 boards cost $4, The  

##Required Libraries

* RunningAverage - http://playground.arduino.cc/Main/RunningAverage
* PIDLibrary - http://playground.arduino.cc/Code/PIDLibrary

