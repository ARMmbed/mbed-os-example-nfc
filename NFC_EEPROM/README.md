# NFC EEPROM usage example

Demonstration of possible usage of the NFC EEPROM class. 

The application will write an URL to the EEPROM of the NFC tag on your board. This will be able to be read by any NFC device capable of reading NFC tags.

The example needs to supply a driver for the EEPROM. The `M24SR` and `ST25DV` EEPROM drivers are provided for the `DISCO_L475VG_IOT01A` and `NUCLEO_F401RE` targets respectively. The `PN512` EEPROM driver is also provided for the `NUCLEO_F401RE` target. You may wish to add your own driver or update the configuration if you're using a different target.

# Running the application

## Requirements

Verification of the sample application can be seen on any smartphone with a NFC reader. After running you will be able to read the tag with a NFC tag reader application running on the smartphone.

Information about activity is also printed on the serial interface. See the [official documentation website](https://os.mbed.com/docs/mbed-os/v5.15/tutorials/serial-comm.html#using-terminal-applications) for more informartion about using serial terminals.

As mentioned above, you will also need to supply the driver for the EEPROM. This example is known to work on the `DISCO_L475VG_IOT01A` target which uses the [`M24SR`](./source/target/TARGET_M24SR) driver. It also works on the `NUCLEO_F401RE` target which uses the [`ST25DV`](./source/target/TARGET_ST25DV) driver. Alternatively, the `NUCLEO_F401RE` target can use the[`PN512`](./source/target/TARGET_PN512) driver. The three drivers are downloaded during deployment with `mbed deploy`. If you want to use a different driver, create a new directory similar to the ones found in the [./source/target/](./source/target/) and update the content of the library file (`eeprom_driver.lib`) found in it to point to the Git repository containing the driver.

## Building instructions

Clone this repository:

```
git clone https://github.com/ARMmbed/mbed-os-example-nfc.git
```

Navigate to the example:

```
cd mbed-os-example-nfc/NFC_EEPROM
```

Update the source tree:

```
mbed deploy
```

If your are not using a `DISCO_L475VG_IOT01A` or a `NUCLEO_F401RE` you should indicate in the application configuration file ([`mbed_app.json`](./mbed_app.json)) which driver should be selected by the build system as follows:

```json
    "<TARGET_NAME>": {
        "target.extra_labels_add": ["<DRIVER_NAME>"]
    }
```


Build the application:

```
mbed compile -t <TOOLCHAIN> -m <TARGET>
```