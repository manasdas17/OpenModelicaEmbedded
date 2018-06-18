# OpenModelicaEmbedded
OpenModelica for Embedded Applications

### Installation
## Energia
	* Download the latest release of [Energia](http://energia.nu/download/).
	* Follow the installation instructions provided by [Energia](http://energia.nu/download/).
	* In Energia IDE, Select `Tools` and the `Board` and check if `Tiva C` board is present.
	* If not, then click on `Board Manager`, type Tiva C in search bar and then click on `Install` to install board.


## Working with Arduino UNO [Atmega328p]

## Working with Tiva C [TM4C123G]

### Simulate a model with Tiva C
	* In Energia, open the firmware for Tiva C provided in folder named `Firmware`.
	* Select appropriate Board(Tiva C) and Port(USB port where Tiva C is connected) in `Tools` menu.
	* Then, upload the firmware on board.
	* Now open OMEdit and Open the `package.mo` file from OpenModelicaEmbedded package
	* Open an example provided in the OpenModelicaEmbedded package which includes a Tiva C board.
	* Check and Simulate the model and verify the results in Plotting window.
