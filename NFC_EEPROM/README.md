# NFC EEPROM usage example

Demonstration of possible usage of the `NFCEEPROM` class. 

The application will write an URL to the EEPROM of the NFC tag on your board.
This will be able to be read by any NFC device capable of reading NFC tags.

The example needs to supply a driver for the EEPROM. The `M24SR` and `ST25DV` EEPROM drivers are provided for the
`DISCO_L475VG_IOT01A` and `NUCLEO_F401RE` targets respectively. The `PN512` EEPROM driver is also provided for the
`NUCLEO_F401RE` target. You may wish to add your own driver or update the configuration if you're using a different target.

# Running the application

## Requirements

This example is known to work on the `DISCO_L475VG_IOT01A` target which uses the [`M24SR`](./source/target/TARGET_M24SR)
driver. It also works on the `NUCLEO_F401RE` target which uses the [`ST25DV`](./source/target/TARGET_ST25DV) driver.
Alternatively, the `NUCLEO_F401RE` target can use the[`PN512`](./source/target/TARGET_PN512) driver. The three drivers
are downloaded during deployment with `mbed deploy`.

If you want to use a different driver, create a new directory similar to the ones found in the
[./source/target/](./source/target/) and update the content of the library file (`eeprom_driver.lib`) found in it to
point to the Git repository containing the driver.

### Building and running

Building and further running instructions for all samples are in the
[main readme](https://github.com/ARMmbed/mbed-os-example-nfc/blob/master/README.md).
