# NFC SmartPoster example

Demonstration of possible usage of the the `NFCController` class.

The application creates a smart poster record and sends it when a connected peer requests it.
The smart poster record generated contains:
- A URI: https://www.mbed.com
- A title: "mbed website"
- An action: `EXECUTE` which asks the peer to open the URI.

# Running the application

## Requirements

- NXP PN512 shield
- Smartphone with an NFC reader

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

