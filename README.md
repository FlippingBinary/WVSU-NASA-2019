# WVSU-NASA-2019
This repository contains code for a Raspberry Pi Compute Module 3 and an external Arduino MKR Zero microcontroller. The Raspberry Pi will process images captured from two cameras which will be exposed to the vacuum of space, communicate with the microcontroller, and transmit data over an external telemetry line. The microcontroller code will monitor and manage geiger circuits, silicone particle detectors, optical sensors, a BMP280 pressure/temperature sensor, and an IMU.

The Raspberry Pi SD card is set up with an extra partition which is mounted at `/rocksatx`.
This extra partition is formatted with FAT32 to allow Windows computers to interact with the files.
The `rc.local` file is a copy of what is located at `/etc/rc.local` on the SD card and launches the code at `/rocksatx/launch.sh`.
This arrangement makes it much easier to use a Windows computer to rewrite the c code and have `launch.sh` compile the c program prior to running it.

The `launch.sh` script creates a unique directory from which all future scripts are run. This preserves data from each launch attempt since the payload will be tested multiple times prior to launch and may accidentally be powered on after the launch.

The `readtelemetry.c` code is intended to run on a Windows computer and can read directly from a serial COM port or a binary file containing the telemetry data. It will output to stdout if a second argument does not specify a filename to save it to.

The `rocksatx.c` code is what passes along telemetry from the Arduino MKR Zero to the telemetry pin which is ultimately transmitted to Earth.
That is outside the scope of this project.

The `sbsvideo.sh` script just repeatedly attempts to create a SBS video file from the two cameras using the Raspberry Pi's raspivid program.
