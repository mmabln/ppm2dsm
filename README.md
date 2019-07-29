# ppm2dsm
PPM to DSM signal converter

This version has been tested with STM32F030 F4P6

HW connections to STM32F030 are as this:
- PA0   LED output (active low, connect RED LED in series ~1kOhm to Vdd)
- PA1   Bind button input, active low, connect to pushbutton to ground
- PA2   Serial output
- PA9   PPM input

DSM2 serial frame format is this:
- header:
  - byte 1: 0x80 == BIND mode, 0x00 = NORMAL mode
  - byte 2: 0x00
 
- channel data:
  - byte 1:
    - bit 0-1:  Bits 8-9 of channel data
    - bit 2-4:  Channel number
  - byte 2:
    - bit 0-7:  Bits 0-7 of channel data
