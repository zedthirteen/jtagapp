# jtag
Raspberry Pi JTAG adapter

use "make clean jtag" to build

"wiringPi" folder and its files needs to be in parent folder for builds

home folder also needs a folder named jtagdisk - e.g. ~/jtagdisk
The folder jtagdisk needs to contain a single file called 
"not_mounted".
This is used to determine if a USB thumb drive has been 
mounted. If the file is found by the application then the USB
drive has not been mounted to this folder.
There should NOT be a file called "not_mounted" on the USB 
thumb drive.
There is no hot detection of USB drive mounting so the device needs
to be rebooted once a USB drive is inserted.

The file ~/jtag/jtag.ini has user configurable settings for:

[settings]
   customer_name       text value
   clock_timing        value appropriate to Raspberry Pi model in use
[display]
   type                currently only supports parallel but may support I2C in the future
   rows                number of rows on LCD display
   columns             number of columns on LCD display

See ~/jtag/jtag.in for additional information

