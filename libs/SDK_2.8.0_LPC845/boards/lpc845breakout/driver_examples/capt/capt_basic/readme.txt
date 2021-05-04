Overview
========
The CAPT Basic Example shows the simplest way(YH port pin measurement) to use CAPT driver and help user with a quick start.

In this example, user should indicate  the set of X pins to use by writing to the XPINSEL field in the
control register. The CAPT work in YH port pin measurement menthod. In Measure Voltage state, the module
samples the YH port pin (which is in input mode). Until the voltage on the pad increases above the 
VIH (HIGH-level input voltage) of the pad, the module will sample ‘0’, above VIH it will sample ‘1’ (that is,
the YH port pin has “triggered”). Then we would handle the CAPT ISR for triggered events or no-triggered events.


Toolchain supported
===================
- IAR embedded Workbench  8.50.1
- Keil MDK  5.30
- GCC ARM Embedded  9.2.1
- MCUXpresso  11.2.0

Hardware requirements
=====================
- Micro USB cable
- LPC845 Breakout board
- Personal Computer

Board settings
==============
Remove R23 on the LPC845 Breakout board.

Prepare the demo
1.  Connect a micro USB cable between the PC host and the CMSIS DAP USB port on the board.
2.  Open a serial terminal with the following settings:
    - 9600 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

Prepare the Demo
================

Running the demo
================
The log below shows the output of the CAPT basic driver example in the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
CAPT Basic example.
Calibration has finished.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
When touch the electrode plate, the LD1 GREEN would turn on.
Customization options
=====================

