# WSPR Beacon


## IMPORTANT ##

The firmware in this repository was not tested with the PCB design in the
respository.
It was tested on a prototype board with a 3.3V version of an Arduino Nano and a
Si5351 breakout board.

## Getting started

The firmware is built using CMake and requires an installation of `avr-gcc`.

```
cd firmware
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
make flash
```

The beacon is controlled with the `wsprctrl` tool in the `software` directory.
It is written in Python 3 and requires `pyusb`. It is used to set the location,
callsign, frequency and power of the beacon and starts the transmission.
The beacon keeps these settings in memory, so they only have to be set after
a reset. Saving settings to EEPROM is planned.

```
./wsprctrl --correction 26.5
./wsprctrl --drive 8
./wsprctrl --lat 48.210033 --lon 16.363449
./wsprctrl --frequency 7040150
./wsprctrl --call OE5TKM
./wsprctrl --wspr
```

It's also possible to combine multiple options.

## Prototype

* Atmega328 with V-USB for control
* Si5351 with 3 clock outputs (one output per band)
* 40/30/20 meters
* Powered and controlled via USB

### Filter Design

Filter values are taken from [here](https://dk4sx.darc.de/txtp.htm).


## Libraries

* V-USB: http://vusb.wikidot.com/
* Si5351: https://github.com/etherkit/si5351-avr-tiny-minimal/
* I2C: https://github.com/g4lvanix/I2C-master-lib
* UART: http://homepage.hispeed.ch/peterfleury/index.html
