# SD Stoplight #

#### Setup ####
This is a program written in `C` and compiled via `AVR-GCC` for the Microchip `ATTiny804`  
The `ATTiny804` was flashed using `avrdude` and an Arduino UNO as a UPDI programmer  
The Arduino programmer was setup using JTAG2UPDI created by ElTangas.  

Link to repo: https://github.com/ElTangas/jtag2updi  

There are no non-standard libraries used, everything is coded using standard C and AVR-GCC libraries. 

- - - -

### Introduction ###
This program is used to identify SD Cards CMD42 lock status. It is part of a suite of tools used for  
MMC/SD Card serial communication. Sending MMC specific commands and reading responses returned from  
the card or device itself. All transactions are done via the SPI (Serial Peripheral Interface).   

When powered on the code will run in a loop polling for a button press. Once the button is pressed a LED startup sequence
is run. Then the process begins. The SPI protocol is enabled and power is sent to the SD card reader. A initialization 
command is run and if the card cannot be detected or the card fails initialization the program will end and the `Yellow`  
LED will be turned on. If the initialization passes, the code will run further commands to prep the card for a read of the 
password status register. If the card is unlocked the `Green` LED will be turned on. If the card is locked the `Red` LED
will be turned on. Fairly simple program, the tool is used for verification of lock status.


Basic Use:
Plug in SD Card to card reader  
Press button to begin process 
LED Conditions:  
Red LED - Card is locked
Yellow LED - No Detect/

Created by Reilley Ford
