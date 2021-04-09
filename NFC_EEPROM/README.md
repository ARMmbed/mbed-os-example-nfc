# NFC EEPROM usage example

Demonstration of possible usage of the `NFCEEPROM` class. 

The application will write an URL to the EEPROM of the NFC tag on your board.
This will be able to be read by any NFC device capable of reading NFC tags.

# Running the application

## Requirements

This example requires a driver to be supplied for the EEPROM:
[`M24SR`](./EEPROMDriver/target/TARGET_M24SR),
[`NT3H2111`](./EEPROMDriver/target/TARGET_NT3H2111),
[`PN512`](./EEPROMDriver/target/TARGET_PN512), and
[`ST25DV`](./EEPROMDriver/target/TARGET_ST25DV)
drivers are included. The example is known to work with the following targets:

- `DISCO_L475VG_IOT01A`, using the [`M24SR`](./EEPROMDriver/target/TARGET_M24SR)
driver.

- `K64F`, using the [`NT3H2111`](./EEPROMDriver/target/TARGET_NT3H2111) driver.

- `NRF52_DK`, using the [`NT3H2111`](./EEPROMDriver/target/TARGET_NT3H2111) driver.

- `NRF52840_DK`, using the [`NT3H2111`](./EEPROMDriver/target/TARGET_NT3H2111) driver.

- `NUCLEO_F401RE`, using either the [`PN512`](./EEPROMDriver/target/TARGET_PN512) or [`ST25DV`](./EEPROMDriver/target/TARGET_ST25DV) driver.

### Using other drivers

If you want to use a different driver, create a new directory similar to the ones found in
[./EEPROMDriver/target/](./EEPROMDriver/target/) and update the content of the library file `eeprom_driver.lib`, pointing to
the Git repository containing the driver if needed.

If using Mbed CLI 2, you will also need to update `CMakeLists.txt` in [./EEPROMDriver/target/](./EEPROMDriver/target/) to include the new driver.

### Building and running

Building and further running instructions for all samples are in the
[main readme](https://github.com/ARMmbed/mbed-os-example-nfc/blob/master/README.md).
