# NFC SmartPoster example

:warning: **Please note running this example requires a PN512 NFC controller**

Demonstration of possible usage of the the `NFCController` class.

The application creates a smart poster record and sends it when a connected peer requests it.
The smart poster record generated contains:
- A URI: https://www.mbed.com
- A title: "mbed website"
- An action: `EXECUTE` which asks the peer to open the URI.

## Running the application

### Requirements

This example has been tested on boards connected to a PN512 shield. A configuration is supplied for `NUCLEO_F401RE` and `NUCLEO_L433RC_P`, but most boards will work.
To use other boards you must supply an override in `mbed_app.json` for your board:

```
        "pn512_mosi": "NC",
        "pn512_miso": "NC",
        "pn512_sclk": "NC",
        "pn512_ssel": "NC",
        "pn512_irq": "NC",
        "pn512_reset": "NC"
```

### Wiring diagram for Explore NFC with PN512

If using the Raspberry Pi Explore NFC (PN512) board, use this pinout mapping diagram to connect the shield.

                Explore NFC
      (pin1 on shield shown with a <|)
                +--------+
                |[ 2][ 1]| 3V3
                |[ 4][ 3]| SSEL
                |[ 6][ 5]|             
                |[ 8][ 7]|
                |[10][ 9]|
                |[12][11]|
                |[14][13]|
           IRQ  |[16][15]|
                |[18][17]|
                |[20][19]| MOSI
                |[22][21]| MISO
                |[24][23]| SCLCK
          RESET |[26][25]| GRND
                +--------+

In this case a ST NucleoF401RE pinout is shown.

              Nucleo F401RE                 
             (Arduino header)
         +-------+     +-------+                  
         | [NC]  |     | [B8]  |                  
         | [IOREF|     | [B9]  |                  
         | [RST] |     | [AVDD]|                  
    1<---+ [3V3] |     | [GND] |                  
         | [5V]  |     | [A5]  +--->23                  
         | [GND] |     | [A6]  +--->21                  
    25<--+ [GND] |     | [A7]  +--->19                  
         | [VIN] |     | [B6]  +--->3                  
         |       |     | [C7]  |                  
    26<--+ [A0]  |     | [A9]  |                  
    16<--+ [A1]  |     | [A9]  |                  
         | ...   |     |       |                  
         |       |     | [A8]  |                  
         +-------+     | ...   |                  
                       |       |
                       |       |
                       +-------+

Patch using jumper wires to the indicated pins on the Shield.            

Schematic (https://www.element14.com/community/docs/DOC-76384/l/explore-nfc-board-schematic)

### Building and running

Building and further running instructions for all samples are in the
[main readme](https://github.com/ARMmbed/mbed-os-example-nfc/blob/master/README.md).
