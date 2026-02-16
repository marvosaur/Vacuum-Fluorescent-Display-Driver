# Vacuum-Fluorescent-Display-Driver
the cubeIDE files you'd need for driving a VFD. u8g2 only for startup animation-if you choose to omit it, then u8g2 is unnecessary. main.c, main.h and the .ioc are from STM32CubeIDE if you choose to mess with the onboard MCU.

Final version of firmware with stable brightness control using PWM on GRID_BLANK

## Notes
Looking at the display driver board facing you, the right side pins are as follows
From top to bottom
- Vin (5-12V)
- GND
- RX
- TX (unused)

The left side pins are routed out SPI that are unused. They serve as a mechanical connection for the display.

Header info and UART baud rate can be found in the .ino

Read the full writeup here!
https://www.marvintian.com/projects/vfd/ 
