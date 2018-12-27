# NFC EEPROM usage example

Demonstration of possible usage of the NFC EEPROM class. 

The application will write an URL to the EEPROM of the NFC tag on your board. This will be able to be read by any NFC device capable of reading NFC tags.

The example needs to supply a driver to the eeprom. This example uses the M24SR driver on the DISCO\_L475VG\_IOT01A and target and the PN512 driver on the NUCLEO\_F401RE target. You may wish to add your own driver or update the configuration if you're using a different board.

# Running the application

## Requirements

Verification of the sample application can be seen on any a smartphone with an NFC reader. After running you will be able to read the tag with an NFC tag reader application.

Information about activity is also printed over the serial connection - please have a client open. You may use:

- [Tera Term](https://ttssh2.osdn.jp/index.html.en)

You will also need to supply the driver for the EEPROM. This example is known to work on DISCO_L475VG_IOT01A which uses M24SR. The driver is downloaded during deployment step (`mbed deploy`). This is based on the contents of the `eeprom_driver.lib` file. The build process will pick up this library and build it as part of the mbed-os build. If you want to use a different driver, please update the contents of `eeprom_driver.lib` to point at the github repository containing the driver.

## Building instructions

Clone the repository containing the collection of examples:

```
git clone https://github.com/vince-zeng/mbed-os-example-nfc.git
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

If your board is not a DISCO\_L475VG\_IOT01A or a NUCLEO\_F401RE you should indicate in the configuration which driver should be selected by the build system. Edit `mbed_app.json` which contains the configuration. In the `target_overrides` section add your target configuration. The drivers currently supported are `PN512` and `M24SR`.

```json
    "<TARGET_NAME>": {
        "target.extra_labels_add": ["<DRIVER_NAME>"]
    }
```


Run the build:

```
mbed compile -t <ARM | GCC_ARM> -m <YOUR_TARGET>
```

For NXP K64F, run the following command:
```
(5)	mbed compile –toolchain GCC_ARM –target K64F
```

# Known issues

* This example doesn't work with IAR 7.80.