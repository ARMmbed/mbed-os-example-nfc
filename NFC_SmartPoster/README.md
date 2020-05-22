# NFC SmartPoster example

** :warning: Please not running this example requires a PN512 NFC controller and does not support NFC EEPROMs **

Demonstration of possible usage of the the `NFCController` class.

The application creates a smart poster record and sends it when a connected peer requests it.
The smart poster record generated contains:
- A URI: https://www.mbed.com
- A title: "mbed website"
- An action: `EXECUTE` which asks the peer to open the URI.

# Running the application

## Requirements

Verification of the sample application can be seen on any a smartphone with an NFC reader. After running you will be able to read the tag with an NFC tag reader application.

This example is known to work on boards connected to a PN512 shield.

**Wiring diagram for NFC Explorer with PN512**

If using the Raspbery Pi explorer (PN512) board, use this pinout mapping diagram to connect the shield to the reference target. In this case a ST NucleoF401RE pinout is shown.

              Nucleo F401RE                Explore NFC                 
             (Arduino header)        (pin1 on shield shown with a <|)
         +-------+     +-------+             +--------+                  
         | [NC]  |     | [B8]  |             |[ 2][ 1]|                  
         | [IOREF|     | [B9]  |             |[ 4][ 3]|                  
         | [RST] |     | [AVDD]|             |[ 6][ 5]|                  
    1<---+ [3V3] |     | [GND] |             |[ 8][ 7]|                  
         | [5V]  |     | [A5]  +--->23       |[10][ 9]|                  
         | [GND] |     | [A6]  +--->21       |[12][11]|                  
    25<--+ [GND] |     | [A7]  +--->19       |[14][13]|                  
         | [VIN] |     | [B6]  +--->3        |[16][15]|                  
         |       |     | [C7]  |             |[18][17]|                  
    26<--+ [A0]  |     | [A9]  |             |[20][19]|                  
    16<--+ [A1]  |     | [A9]  |             |[22][21]|                  
         | ...   |     |       |             |[24][23]|                  
         |       |     | [A8]  |             |[26][25]|                  
         +-------+     | ...   |             +--------+                  
                       |       |                               
                       |       |                               
                       +-------+                               
                                             
    Patch using jumper wires to the             
    indicated pins on the Shield.            


Schematic (https://www.element14.com/community/docs/DOC-76384/l/explore-nfc-board-schematic)


## Building instructions

Clone the repository containing the collection of examples:

```
git clone https://github.com/ARMmbed/mbed-os-example-nfc.git
```

Using a command-line tool, navigate to the exmaple:

```
cd mbed-os-example-nfc
cd NFC_SmartPoster
```

Update the source tree:

```
mbed deploy
```

Run the build:

```
mbed compile -t <ARM | GCC_ARM> -m <YOUR_TARGET>
```

