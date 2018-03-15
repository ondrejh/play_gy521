# tank
msp430 based tank servo signal convertor

## description
It makes my tank gear bought from China move. Now i have quite minimalistic version with msp430 but planning to add RPi version equiped with camera.

## minimalistic prototype
![minimalistic guts](/doc/guts.jpg)
Fromat: ![minimalistic guts]

## software
Reading UART input from FlySky micro reciever, converting XY servo signals to XX tank stearing values and efectively changint it to PWMs.

## hardware
Double H bridge, MSP430G2553 (from MSP430G2 launchpad), linear stabilisator TO92 3.3V, receiver FS-A8S.
I promise I'll draw the schematic, once...
