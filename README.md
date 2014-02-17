STM32 port of Repapers example code - https://github.com/repaper/gratis

This code uses the ST FW library, this needs to be put in a directory called "st-lib"

Hook up the boards like this:

PA8 - PWM 9 (Brown)
PB12 - SPI2_NSS - EPD_CS 19 (Brown)
PB13 - SPI2_SCK - SPI_CLK 7 (Yellow)
PB15 - SPI2_MOSI - SPI_MOSI 15 (Blue) 
PC2 - 8 BUSY (Orange)
PA1 - 10 RESET (Black) 
PA2 - 11 PANEL_ON (Red)
PA3 - 12 DISCHARGE (White)
PA15 - 13 BORDER_CONTROL (Grey) 
