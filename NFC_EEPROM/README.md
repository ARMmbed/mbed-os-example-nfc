# NFC EEPROM usage example

Demonstration of possible usage of the NFC EEPROM class. 

The application will write an URL to the EEPROM of the NFC tag on your board. This will be able to be read by any NFC device capable of reading NFC tags.

The example needs to supply a driver to the eprom. This example uses the M24SR driver. You may wish to replace the existing driver with your own driver if you're using a different board:

```c++
mbed::nfc::vendor::ST::M24srDriver eeprom_driver;
```

Replace this with a driver you wish to use - it will be passed in to the example to construct the `NFCEEPROM`.

The example may also use a PN512 controller. This driver is included and doesn't require any external dependency.

# Running the application

## Requirements

Verification of the sample application can be seen on any a smartphone with an NFC reader. After running you will be able to read the tag with an NFC tag reader application.

Information about activity is also printed over the serial connection - please have a client open. You may use:

- [Tera Term](https://ttssh2.osdn.jp/index.html.en)

You will also need to supply the driver for the EEPROM. This example is known to work on DISCO_L475VG_IOT01A which uses M24SR. The driver is downloaded during deployment step (`mbed deploy`). This is based on the contents of the `eeprom_driver.lib` file. The build process will pick up this library and build it as part of the mbed-os build. If you want to use a different driver, please update the contents of `eeprom_driver.lib` to point at the github repository containing the driver.

## Building instructions

Clone the repository containing the collection of examples:

```
git clone https://github.com/ARMmbed/mbed-os-example-nfc.git
```

Using a command-line tool, navigate to the exmaple:

```
cd mbed-os-example-nfc
cd NFC_EEPROM
```

Update the source tree:

```
mbed deploy
```

Edit `mbed_app.json` which contains the configuration. Depending on what board you have select which driver to use by setting the value to true or false. If you have a DISCO_L475VG_IOT01A set the M24SR value to true:
```
"M24SR": {
    "help": "Build example including the M24SR EEPROM driver",
    "macro_name": "EXAMPLE_M24SR",
    "value": true
}
```

Unless your board also has a PN512 controller set the other value to false.

Run the build:

```
mbed compile -t <ARM | GCC_ARM> -m <YOUR_TARGET>
```

