# AVR HIDRemote

HIDRemote is implementation of USB IR Remote dongle implemented with
AVR ATtiny45/85 microcontroller based on V-USB.

- V-USB is a software-only implementation of a low-speed USB device for AVR microcontrollers. See http://www.obdev.at/products/vusb/index.html

# Building and Flashing HIDRemote

## Requirements
- avr-gcc
- avrdude
- usbasp

## Building
    $ cd avr-hidremote/firmware
    $ make 

## Flashing
    $ make flash
