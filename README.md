# EL_driver
Driver for EL320.240.36 using an RP2040 or STM32. The STM32 code is just a protocol example, while the rp2040 one provides full on display streaming over USB.

This repository contains code required to drive and stream your display to an EL display with a 4 bit parallel HS VS interface. Including 50% pixel brightness through PWM dimming.

See it in action here:

[![YouTube video](https://img.youtube.com/vi/tvotsRGWCcw/0.jpg)](https://www.youtube.com/watch?v=tvotsRGWCcw)

## Notes:
* Current version only supports EL320.240.36
* The rust Streamer is depricated, use the displayStreamerC
