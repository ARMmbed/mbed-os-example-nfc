![](./resources/official_armmbed_example_badge.png)
# Mbed OS NFC examples 

This repo contains NFC example applications based on mbed OS and built with [mbed-cli](https://github.com/ARMmbed/mbed-cli).

Each example directory prefixed with `NFC_` contains an Mbed os project.

The [NFC documentation](https://os.mbed.com/docs/latest/apis/nfc-technology.html) describes the NFC APIs on mbed OS.

## Using the examples

### Targets for NFC

To build these examples, you need to have a computer with software installed as described [here](https://os.mbed.com/docs/latest/tools/index.html).

In order to use NFC in mbed OS you will need a baord with an NFC controller or a shield. These examples have been
tested on:
- `DISCO_L475VG_IOT01A` with its built in `M24SR`
- `NUCLEO_F401RE` board with a `NXP PN512` shield

But any board with a NFC shield should work. See documentation for the driver for your shield/board for details.

### Building and flashing examples

__To build an example:__

* Clone the repository containing the collection of examples:

	```
	$ git clone https://github.com/ARMmbed/mbed-os-example-nfc.git
	```

	**Tip:** If you don't have git installed, you can [download a zip file](https://github.com/ARMmbed/mbed-os-example-nfc/archive/master.zip) of the repository.

* Using a command-line tool, navigate to any of the example directories, like NFC EEPROM:

	```
	$ cd mbed-os-example-nfc
	$ cd NFC_EEPROM
	```

* Update the source tree:

	```
	mbed deploy
	```

* Run the build:

	```
	mbed compile -t <ARM | GCC_ARM> -m <YOUR_TARGET>
    ```

__To run the application on your board:__

* Connect your mbed board to your computer over USB. It appears as removable storage.

* When you run the `mbed compile` command above, mbed cli creates a .bin or a .hex file (depending on your target) in
```BUILD/<target-name>/<toolchain>``` under the example's directory. Drag and drop the file to the removable storage.

Alternatively you may launch compilation with `-f` flag to have mbed tools attempt to flash your board.
The tools will flash the binary to all targets that match the board specified by '-m' parameter. 

### Running the examples

When example application is running information about activity is printed over the serial connection.
The default serial baudrate has been set to 115200 for these examples.

Please have a client open and connected to the board. You may use:

- [Tera Term](https://ttssh2.osdn.jp/index.html.en) for windows

- screen or minicom for Linux (example usage: `screen /dev/serial/<your board> 115200`)

- mbed tools have terminal command `mbed term -b 115200`

Verification of the sample application can be seen on any a smartphone with an NFC reader.
After running you will be able to read the tag with an NFC tag reader application.

#### Known issues 

The repository is not meant to be imported directly inside the Mbed online compiler. To import one example inside the online compiler, go to https://os.mbed.com/teams/mbed-os-examples/ and import from here the NFC example of your choice.

### License and contributions

The software is provided under Apache-2.0 license. Contributions to this project are accepted under the same license. Please see [contributing.md](CONTRIBUTING.md) for more info.

This project contains code from other projects. The original license text is included in those source files. They must comply with our license guide.

