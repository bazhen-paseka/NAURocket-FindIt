#ifndef __LCD_CONFIG_LOCAL_H
#define __LCD_CONFIG_LOCAL_H

// Please uncomment one of the lines to select your STM chip
//#define STM32F4xx
#define STM32F4xx

//#define NEW_RST
//#define NEW_BIT1

#define ILI9340_COLOR_INVERT

//#define USE_FONT8
#define USE_FONT12
//#define USE_FONT16
//#define USE_FONT20
//#define USE_FONT24

	// Please uncomment one of the lines to select your LCD chip
//#define ILI9325			// works!!
//#define ILI9328			// works!!
#define ILI9340			// works!!
//#define ILI9341			// works!!
//#define ILI9341_00		// works!!
//#define R61505			// works!!
//#define R61505V			// works!! (ATTN! Used to be named R61505 in versions below V1.4)
//#define R61520			// works!! // this is a splitfix screen
//#define S6D0154			// works!!
//#define UNKNOWN1602		// works!!
//#define HX8347D			// works!!
//#define HX8347G			// not tested yet
//#define HX8357D			// not tested yet
//#define SSD1297				// works!!


// Please uncomment one of the lines to select your LCD connection mode
#define USE_8080_8BIT
//#define USE_FSMC

// Please uncomment to draw BMP from SD Card
//#define USE_FATFS

// Please uncomment to clear screen when wrapping
//#define WIPE_SCREEN
// Please uncomment to clear line when wrapping
//#define WIPE_LINES

#endif	//	__LCD_CONFIG_LOCAL_H
