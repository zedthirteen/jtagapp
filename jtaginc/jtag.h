#ifndef JTAG_H
#define JTAG_H

#include "./jtag_types.h"

static unsigned TRST_State = 0;
static unsigned WR_State = 0;
unsigned long int FLASH_START, FLASH_TOP;

#define JTAG_TCK	0
#define JTAG_TMS	2
#define JTAG_TDI	3	// name is from ETM side
#define JTAG_TDO	4	// INPUT = name is from ETM side
//#define JTAG_TDO	21	// Alternative if needed to support digitalWriteByte
#define JTAG_TRST	5
#define JTAG_WR		6	// I think this is STROBE in intel app note?

// bit mask defines for digitalWriteByte
//
// DigitalWriteByte will configure the first 8
// wiringPi GPIO pins in a single hit
// (it's actually 2 instructions)
//
#define TCK		0x01	// Bit 0 is pin 0 = TCK output
//			0x02	// Bit 1 is pin 1 = is not being used
#define TMS		0x04	// Bit 2 is pin 2 = TMS output
#define TDI		0x08	// Bit 3 is pin 3 = TDI output
#define TDO		0x10	// Bit 4 is pin 4 = TDO input
#define TRST		0x20	// Bit 5 is pin 5 = TRST output
#define J_WR		0x40	// Bit 6 is pin 6 = WR output
//			0x80	// Bit 7 is pin 7 = is not being used

#define TDITMS		0x0C
#define TCKTMS		0x05
#define notTCKTMS	0xFA	// not of TCK, TMS
#define notTRST		0xDF	// all bit except bit 5

#define MAX_MENU_ITEMS  30
int menuItemIndex;


enum device
{
   ETM_TGXtra = 0,
   SCV,
   ETM_TGX
};

enum action
{
   CHECK_SIGNATURE = 0,
   DUMP_MEMORY,
   LOAD_MEMORY,
   REBOOT,
   SHUTDOWN,
   DISPLAY_IP,
   CHECK_DATE_TIME,
   EXIT_TO_CMD // only if DJF_DEBUG defined
};

enum timing_mode
{
   NOP_LOOP = 0, // volatile "nop" for loop
   CCR_LOOP      // volatile kernel cycle counter for loop
};

#endif // JTAG_H include sentry

