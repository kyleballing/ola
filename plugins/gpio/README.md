General Purpose I/O Plugin
==========================

This plugin controls the General Purpose Digital I/O (GPIO) pins on devices
like a Raspberry Pi. It creates a single device, with a single output port.
The offset (start address) of the GPIO pins is configurable.


## Config file: `ola-gpio.conf`

`gpio_pins = [int]`  
A comma separated list of GPIO pins to control. Each pin is mapped to a DMX512 slot. Default is an empty string. Examples: 17 or 17,18,19

`gpio_slot_offset = <int>`  
The DMX512 slot for the first pin. Slots are indexed from 1. Default is 1.

`gpio_pwm_frequency = <int>`
The PWM frequency to use for GPIO light dimming. Default is 1000 Hz.

## TODO
* Add arguments to adjust the duty cycle range for each pin
* Add arguments to adjust the frequency for each pin
* Add argument to modify the sample rate of the GPIO daemon
* Allow pins to use PWM, On/Off, or be inputs
* Can we use GPIO for SPI to do RGB LED arrays? Better to set this up externally?