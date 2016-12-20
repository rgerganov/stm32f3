Firmware for the `stm32f3discovery` board which exposes its sensors through a 
virtual serial port. It just combines several examples for `libopencm3` into
one program. Different sensors are read by sending a different command to the
searial port: `m` for magnetometer, `a` for accelerometer, etc.

To build and flash the firmware:

    $ git submodule init
    $ git submodule update
    $ cd firmware/libopencm3
    $ make
    $ cd ..
    $ make
    $ make flash

There is a sample application for a compass in the `host` folder.

You can see it in action here: https://www.youtube.com/watch?v=qPO57GnuQYc

