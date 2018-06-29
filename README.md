# OpenModelicaEmbedded
OpenModelica for Embedded Applications

## Installation

### Arduino
* Download and Install the latest version of arduino ide from the [Arduino](https://www.arduino.cc/en/Main/Software?) website.

### Energia
* Download the latest release of [Energia](http://energia.nu/download/).
* Follow the installation instructions provided by [Energia](http://energia.nu/download/).
* In Energia IDE, Select `Tools` and the `Board` and check if `Tiva C` board is present.
* If not, then click on `Board Manager`, type Tiva C in search bar and then click on `Install` to install board.
* While using Energia on Windows Operating System, make sure you install necessary device drivers by following the instructions given [here](http://energia.nu/guide/guide_windows/).
---
## How to make changes to source code?

## Linux Operating System
* Open the source codes by browsing to this location : `OpenModelicaEmbedded > Source`.
* After making changes to these files open `Terminal`.
* Browse to `OpenModelicaEmbedded > Source` folder using `cd` command.
* Run command `make`.

## Windows Operating System
* Open the source codes by browsing to this location : `OpenModelicaEmbedded > Source`.
* After making changes to these files open `Command Prompt (cmd)`.
* Browse to `OpenModelicaEmbedded > Source` folder using `cd` command.
* To compile the CPP files run the command: `g++ -c file1.cpp file2.cpp ...`.
* To create a DLL from generated object files, run the command: `g++ -shared -o modelPlugFirmata.dll file1.o file2.o ...`.
* Then copy the generated DLL file and paste it in folders: `OpenModelicaEmbedded` and `Resources > Library > win64`.

## Working with Arduino UNO [Atmega328p]

### Setting up firmware for Arduino board
* In `Tools` Menu, select appropriate Board (Arduino/Genuino UNO) and Port as the available serial port to which Arduino is connected.
* Open **pidmata3** sketch: `File > Open > OpenModelicaEmbedded > Firmware > Arduino > pidmata3 > pidmata3.ino`.
* Upload the sketch to the board.
### Simulating the Modelica model
* Now open OMEdit window.
* Open **package.mo** file OpenModelicaEmbedded folder.
* In OpenModelicaEmbedded package, open **ArduinoExamples** package which consists of examples for arduino board.
* Check and simulate the example models and verify the results.

---
## Working with Tiva C [TM4C123G]

### Simulate a model with Tiva C
* In Energia, open the firmware for Tiva C provided in folder through path : `File > Open > OpenModelicaEmbedded > Firmware > Tiva C > StandardFirmata > StandardFirmata.ino` or add zip file of this StandardFirmata as an external library in Energia from the same folder.
* Select appropriate Board (Tiva C) and Port (USB port where Tiva C is connected) in `Tools` menu.
* Then, upload the firmware on board.
* Now open OMEdit and Open the `package.mo` file from OpenModelicaEmbedded package
* Open an example provided in the OpenModelicaEmbedded package which includes a Tiva C board.
* Check and Simulate the model and verify the results in Plotting window.
