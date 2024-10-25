# Embedded MODBUS Master & Slave Drivers
## Description

This driver is for the implementation of MODBUS aimed at any processor.  It is written in C and hardware specific functions are stubbed out and rely on the user providing a port file for the implementation of this.  The driver allows opening multiple ports as either slave or master.  It implements 485 RTU, ASCII, and TCP modes.

These drivers were written to be as generic, portable, and expandable as possible.  Basically the files in the source folder you should be able to just drop into your project. You will then need to implement a modbus_port file to define the stubbed out hardware functions for transmitting and receiving, and a modbus_callbacks file to handle the retrieving of requested registers.  You also will likely need to edit the conf_modbus.h file for your needs, you can disable and enable different features here to save on code space as well. Master mode works off callbacks as well, when you issue a master command you can provide a callback to be called when/if the slave responds.

As far as running the drivers, take a look at the examples.  Basically you just include modbus_module.h in your main app and then call modbus_module_init and pass in your desired settings to start up an instance of the interface.  You can open up multiple ports by calling the init function multiple times with different port numbers (just be sure you have written modbus_ports implementations to handle the port value being passed in).  The maximum number of allowable ports is defined in the conf file and can be changed to whatever is needed (more ports = more memory).  Once inited you need to routinely call modbus_module_service in your main loop to handle the processing of packets such.  There is also a modbus_module_needs_serviced function if you prefer to check if the module needs serviced or not.

As for the PORT file, you are basically setting up your part's UART and a timer or for TCP an FSM to manage the socket connection.  MODBUS RTU is a time based protocol so it is recommended that you make this an interrupt based setup but if not then you should ensure you service them often in order to indicate accurately to the driver's when a byte is received and when a timeout occurs.  Your port init function will take in a TX, RX, and Timeout callback pointer and you should call these whenever one of the 3 circumstances occur.  You will want to disable RX, TX, and the timer in the init routine because the drivers will call the enable functions that you implement when it is time to turn them on.  See the examples folder to get and idea of how to implement these : ).

## Revision Log

Rev 1.0 - Jarrod Cook (5/2014):
	- The first version of drivers adapted from freemodbus library.  Still need to implement TCP and ASCII drivers.

Rev 1.1 - Jarrod Cook (1/2015):
	- Jacob found and issue where I only had the START and NUM Address callbacks returning bytes instead of words, fixed this.
	- Added versioning into the library files themselves, just to make it easier to see what version a project has
	
Rev 1.2 - Jarrod Cook (4/2017):
	- Fixed an issue with discrete looking at the wrong start address function.
	
Rev 1.3 - Jarrod Cook (2/2020):
	- Finished ASCII Functionality.

Rev 1.4 - Jarrod Cook (4/2022)
	- Added in TCP functionality with example
	- Added master example to example code
	
## Todos
- [ ] Implement the ability to set both they BYTE (endianness) and WORD (word swap) order [ABCD,BADC,CDAB,DCBA].
- [ ] Add the ability for a timeout in master mode (would need ports stub out for timer so maybe leave this to the user?)
- [ ] Simplify holding write callbacks by passing the bytes remaining to write holding and remove the start and verify
- [ ] Improve the example code to actually be a working full example for a given CPU?
