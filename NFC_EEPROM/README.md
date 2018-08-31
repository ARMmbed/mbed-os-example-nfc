# NFC EEPROM usage example

Demonstration of possible usage of the NFC EEPROM class. 

The application will write an URL to the EEPROM of the NFC tag on your board. This will be able to be read by any NFC device capable of reading NFC tags.

# Running the application

## Requirements

Verification of the sample application can be seen on any a smartphone with an NFC reader. After running you will be able to read the tag with an NFC tag reader application.

Information about activity is also printed over the serial connection - please have a client open. You may use:

- [Tera Term](https://ttssh2.osdn.jp/index.html.en)

You will also need to supply the driver for the EEPROM. This example is known to work on DISCO_L475VG_IOT01A which uses M24SR. To get the driver you may clone the repository:

	```
	$ git clone https://github.com/ARMmbed/mbed-nfc-m24sr.git
	```
    
and place the folder in the NFC_EEPROM folder. The build process will pick up this library and build it as part of the mbed-os build.

## Building instructions

Clone the repository containing the collection of examples:

	```
	$ git clone https://github.com/ARMmbed/mbed-os-example-nfc.git
	```

Using a command-line tool, navigate to the exmaple:

	```
	$ cd mbed-os-example-ble
	$ cd NFC_EEPROM
	```

Update the source tree:

	```
	mbed deploy
	```

Run the build:

	```
    mbed compile -t <ARM | GCC_ARM> -m <YOUR_TARGET>
    ```

