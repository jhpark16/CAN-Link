## Weather data network implemented with CAN bus
A weather station needs to communicate with various sensors at some distances and CAN bus is ideal for the small scale network. Compared with I2C bus, CAN bus is superior because the differential driver allows long distance communication at a relatively fast speed and the hardware takes care of communication priorioties and conflicts. Furthermore, any devices in CAN bus initiate the communication. Thus, the main station does not need to poll devices.
The CAN bus is constructed with STM32F103C8T6 and PCA82C250 CAN bus driver. The STM32F103C8T6 is an cheap ARM based microcontroller with hardware CAN bus and a blue STM32F103C8T6 board can be purchased for AU$2.30 at AliExpress. This CAN bus was terminated with a 60 ohm resistor. The bus is currently operating at 1MHz. 


### CAN bus communication protocol analysed with a logic analyser.



