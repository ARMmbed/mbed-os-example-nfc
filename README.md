![](./resources/official_armmbed_example_badge.png)
# Mbed OS NFC examples 

This repo contains NFC example applications based on mbed OS and built with [Mbed CLI 1](https://github.com/ARMmbed/mbed-cli) or [Mbed CLI 2](https://github.com/ARMmbed/mbed-tools).

Each example directory prefixed with `NFC_` contains an Mbed os project.

The [NFC documentation](https://os.mbed.com/docs/latest/apis/nfc-technology.html) describes the NFC APIs on mbed OS.

## Using the examples

### Targets for NFC

Supported targets vary for each example. Check `README.md` in the example's directory for details.

## Mbed OS build tools

### Mbed CLI 2
Starting with version 6.5, Mbed OS uses Mbed CLI 2. It uses Ninja as a build system, and CMake to generate the build environment and manage the build process in a compiler-independent manner. If you are working with Mbed OS version prior to 6.5 then use [Mbed CLI 1](#mbed-cli-1).
[Install Mbed CLI 2](https://os.mbed.com/docs/mbed-os/latest/build-tools/install-or-upgrade.html).

### Mbed CLI 1
[Install Mbed CLI 1](https://os.mbed.com/docs/mbed-os/latest/quick-start/offline-with-mbed-cli.html).

## Building the examples

1. Clone the repository containing the collection of examples:

	```bash
	$ git clone https://github.com/ARMmbed/mbed-os-example-nfc.git
	```

1. Using a command-line tool, navigate to any of the example directories, like `NFC_EEPROM`:

	```bash
	$ cd mbed-os-example-nfc
	$ cd NFC_EEPROM
	```

1. Update the source tree:

	* Mbed CLI 2
	```bash
	$ mbed-tools deploy
	```

	* Mbed CLI 1

	```bash
	$ mbed deploy
	```

1. Connect a USB cable between the USB port on the board and the host computer.

1. Run the following command: this will build the example project, program the microcontroller flash memory, and then
open a serial terminal to the device.

    * Mbed CLI 2

    ```bash
    $ mbed-tools compile -m <TARGET> -t <TOOLCHAIN> --flash --sterm --baudrate 9600
    ```

    * Mbed CLI 1

    ```bash
    $ mbed compile -m <TARGET> -t <TOOLCHAIN> --flash --sterm --baudrate 9600
    ```


Your PC may take a few minutes to compile your code.

The binary will be located in the following directory:
* **Mbed CLI 2** - `./cmake_build/<TARGET>/<PROFILE>/<TOOLCHAIN>/`</br>
* **Mbed CLI 1** - `./BUILD/<TARGET>/<TOOLCHAIN>/`

You can manually copy the binary to the target, which gets mounted on the host computer through USB, rather than using the `--flash` option.

You can also open a serial terminal separately, as explained below, rather than using the `--sterm` and `--baudrate` options.

## Running the examples

When the example application is running, information about activity is printed over the serial connection.
The default serial baudrate has been set to 9600 for these examples.

If not using the `--sterm` and `--baudrate` options when flashing, have a client 
open and connected to the board. You may use:

- Mbed CLI 2 
	```bash
	$ mbed-tools sterm -b 9600
	```

- Mbed CLI 1
	```bash
	$ mbed sterm -b 9600
	```

- [Tera Term](https://ttssh2.osdn.jp/index.html.en) for Windows

- screen or minicom for Linux
    ```bash
    $ screen /dev/serial/<your board> 9600
    ```


Verification of the sample application can be seen on any a smartphone with an NFC reader.
After running you will be able to read the tag with an NFC tag reader application.

#### Known issues 

The repository is not meant to be imported directly inside the Mbed online compiler. To import one example inside the online compiler, go to https://os.mbed.com/teams/mbed-os-examples/ and import from here the NFC example of your choice.

### License and contributions

The software is provided under Apache-2.0 license. Contributions to this project are accepted under the same license. Please see [contributing.md](CONTRIBUTING.md) for more info.

This project contains code from other projects. The original license text is included in those source files. They must comply with our license guide.

