# ZWUtil
ZWUtil is a Linux utility for controllers/gateways that communicates with a Z-Wave device via SerialAPI.

Present functionality is to query the Z-Wave device for version info and print the information to the console. Other functionality may be added in the future.

# Setup
   1. Make sure the target device is flashed with a SerialAPI firmware image. Note that the UZB dongles (UZB, UZB-7, etc.) should be compatible.
   2. Clone the repo on your target or in your build environment.
   3. Use the makefile to build the project. If you are using a Raspberry Pi, you should just be able to go to the folder and type "make". You can also "make debug" which generates more console output for debugging purposes.
   4. Run the program, targeting the port for the SerialAPI device. If directly connected to the Raspberry Pi hardware unit, the port will be "/dev/ttyAMA0". If you are connected via a USB device (UZB, Wireless Starter Kit, etc.) it will probably be "/dev/ttyACM0". Example:
```
./exe/ZWUtil /dev/ttyACM0
```

# Credits
This code was adapted from Dr. ZWave's "BasicOnOff" project.

# Contacts
- Kris Young - kris.young@silabs.com
