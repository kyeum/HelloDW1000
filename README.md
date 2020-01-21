# HelloDW1000
DW1000 guide with pyboard

![Maintenance](https://img.shields.io/maintenance/no/2019.svg)
![Required know-how](https://img.shields.io/badge/Required%20know--how-professional-red.svg)
![Additional hardware required](https://img.shields.io/badge/Additional%20hardware-required-orange.svg)
![c++11](https://img.shields.io/badge/C%2B%2B-11-brightgreen.svg)
[![releases](https://img.shields.io/github/release/thotro/arduino-dw1000.svg?colorB=00aa00)](https://github.com/thotro/arduino-dw1000/releases)
![min arduino ide](https://img.shields.io/badge/ArduinoIDE-%3E%3D1.6.10-lightgrey.svg)


A library that offers basic functionality to use Decawave's DW1000 chips/modules with Arduino
(see https://www.decawave.com/products/dwm1000-module).

Project state
-------------

**Development:**

This library is currently (2019) **not actively maintained** by teh owner *thotro*.

Anyway you can create pull requests if you found a bug or developed a new feature. They maybe help others.

**TODOLists:**
* Design architecture and library for other user's with DWM1000 & pyboard with the help of other's arduino libraries
* First step will be 

**General notice:**
* Datasheet and application notices are available at https://www.decawave.com/ (require free registration).

Installation
------------

**Requires c++11 support**, Arduino IDE >= 1.6.6 support c++11.

 1. Get a ZIP file of the master branch or the latest release and save somewhere on your machine.
 2. Open your Arduino IDE and goto _Sketch_ / _Include Library_ / _Add .ZIP Library..._
 3. Select the downloaded ZIP file of the DW1000 library
 4. You should now see the library in the list and have access to the examples in the dedicated section of the IDE

Note that in version 1.6.6 of your Arduino IDE you can get the library via the Arduino library manager.

Contents
--------

 * [Project structure](../../wiki/Project-structure)
 * [Features and design intentions](../../wiki/Features)
 * [Testbed and Adapter board](../../wiki/Testbed-and-Adapter-board)
 * [Projects, Media, Links](../../wiki/Projects)
 * [Benchmarks](../../wiki/Benchmarks)
 * API docs
   * [HTML](https://cdn.rawgit.com/thotro/arduino-dw1000/master/extras/doc/html/index.html)
   * [PDF](https://cdn.rawgit.com/thotro/arduino-dw1000/master/extras/doc/DW1000_Arduino_API_doc.pdf)
