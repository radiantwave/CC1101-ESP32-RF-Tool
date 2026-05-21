A cheap fob cloner wasn't able to clone, things escalated, but the goal was reached.  
Have fun
 
```
/*
 * CC1101 Universal 433 MHz RF Tool  |  ESP32-WROOM-32
 *
 * Wiring (8-pin CC1101 module):
 *   GND-GND  VCC-3V3 (1.8-3.6V only!)  CSN-GPIO5  SCK-GPIO18  MOSI-GPIO23  MISO-GPIO19
 *   GDO0-GPIO2 (TX data)               GDO2-GPIO4 (RX data)
 *
 * Libraries: "SmartRC-CC1101-Driver-Lib" (LSatan), "rc-switch" (sui77)
 * Serial @ 115200, line ending = Newline. Type ? for commands.
 *
 * Modes: 'decode' lets rc-switch identify known protocols; 'raw' captures the
 * bare pulse timing of any OOK burst so it can be replayed like a copy fob.
 * All settings persist in NVS flash.
 */
````

