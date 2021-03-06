/*
 * jtag.c:
 * Text-based LCD driver test code
 * This is designed to drive the DJF LCD Plate (like adafruit RGB but no RGB   and 4 x 20)
 * with the additional 5 buttons for the Raspberry Pi
 *
 * Copyright (c) 2012-2013 Gordon Henderson./David Field
 ***********************************************************************
 * This file is part of wiringPi:
 * https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

#include <termios.h>

#include <sys/mount.h>
#include <errno.h>

#include <dirent.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>

#include <wiringPi.h>
#include <mcp23017.h>
#include <lcd.h>

#include <wiringShift.h>

#include <stdbool.h>

#include <jtag.h>
#include <jtag_bsr_pins.h>
#include <jtag_funcs.h>
#include <jtag_types.h>
#include <pi_models.h>

#include <ini.h>

#ifndef TRUE
#define TRUE (1==1)
#define FALSE (1==2)
#endif

// Note: select 1 or the other based on PCB in use
// DJF To Do - might be possible to test for presence of hat EEPROM on board rev 2?
//
//#define BOARD_REV_1
#define BOARD_REV_2

#define m1 0x0782B003

// DOS style revolving paddle
//
// DJF Note: display is Asian character set and has Yen symbol instead of backslash
// use a bitmap for the backslash character in programmable area 0
//
// This should work for both European and Asian displays
//
unsigned char backslashChar[8] = {0, 0x10, 0x08, 0x04, 0x02, 0x01, 0, 0};
//char paddleChar[8] = { '|', '/', '-', '\\', '|', '/', '-', '\\' };
char paddleChar[8] = { '|', '/', '-', 0, '|', '/', '-', 0 };

int paddleCharIndex = 0;

// DJF DEBUG TO BE REMOVED
//#define DJF_ANALYSER // don't wait on display delays

// DJF Note: 300  cycles appeared to work fine on Pi ZeroW
// 500 no cycles is still failing on Pi 3B+
// 1000 appears to be ok for ETM but failing on SCV - now okay obn SCV - was problem with WR line
//
//int cycles_to_wait = 1000; // will be updated from config ini file if available
// Note: cycles_to_wait is nw derived from cpu max freq but can be overriden if value is
//       present in ini file
//
extern int cycles_to_wait;

extern bool verbose_debug;

// parameters for display size - will be read from config ini file
//
//char * display_type = "**** Unkwnown display type ****"; // will be parallel but may support i2c in future
int display_rows = 4;
int display_cols = 20;

// special functions - will be read from jtag.ini
int diagnostic_menu = 0;
int debug_level = 0;
int command_line_opt = 0;
int ignore_errors_opt = 0;

int cpu_max_freq = 0;
extern int wait_mode;                  // whether to wait for a nop loo count or elapsed cycle counter register vlaue
int pulse_width_ns = 1000;             // the desired JTAG uls width of a mark or space clock pulse

const word ID_String_Length = 32;

unsigned long int A;                   // stores address data
unsigned long int i;                   // stores index value
unsigned long int data_start_address;  // Holds start address of program
extern unsigned long int flash_magic_1;
extern unsigned long int flash_magic_2;
unsigned long int Device_ID;           // 32 bit value for 386EX version identifier
unsigned long int M5_Device_ID;        // 32 bit value for M5 identifier
int Flash_Device_ID;                   // 32 bit value for flash ID

char filepath[256];

void initTermios(int echo);
void resetTermios(void);
char getch_(int echo);
char getch(void);
char getche(void);

void Get_M5_Device_ID(void);
void Get_JTAG_Device_ID(void);
void Get_Board_Revision(int);

void show_char(int);

struct signatureBlockType
{
   unsigned short int theSignature[2]; // 4 bytes read from signature area
   bool signatureOk;
};

struct addressRangeType
{
   unsigned long int startAddress;
   unsigned long int endAddress;
};

struct signatureBlockType testForSignature(unsigned long int, int);
void dump_to_file(unsigned long int, unsigned long int, int, char *);
void show_dump_stats(time_t, time_t);
void retest_signature(int);
unsigned long int selectMaximumMemory(int);
struct addressRangeType selectAddressRange(unsigned long int);
unsigned long int selectHexAddress(char*, unsigned long int);
char incrementHexDigit(char hexDigit);
char decrementHexDigit(char hexDigit);
bool confirmAddressRange(struct addressRangeType stAddressRange);
void showWelcomeScreen(void);
void configureGPIO(void);
void topLevelMenu(void);
void JTAGMenu(void);
void diagnosticsMenu(int thisDeviceType, int totalMemory);
void settingsMenu(void);
void showIPDetails(void);
void showHostname(void);
void checkDateTime(void);
void checkClockPulseWidth(int cycles_to_wait);
void configurationMenu(void);
void checkSignature(int);
void dumpMemory(int thisDeviceType, int totalMemory);
void loadMemory(void);
void notYetImplemented(void);
void exportSettings();
void importSettings();
int File_Copy(char FileSource[], char FileDestination[]);
void askRebootNow(void);
void promptForEnter(void);
int askHowToProgressError(void);
char getNextPaddleChar(void);

char menuItems[MAX_MENU_ITEMS][20]; // array to hold up to 30 <MAX_MENU_ITEMS> rows of 20 characters for menu items

int select_menu_item(int numberOfItems, char(*menuItems)[20], char* menuTitle);
void display_menu_items(int numberOfItems, int firstItem, char(*menuItems)[20]);

bool selectBoolean(char* title);

int selectDeviceType(void);

bool selectFilename(char *);

void reboot_system(void);
void shutdown_system(void);

bool exitToCommandLine(void);

void testOutputs(void);

static struct termios old, current;

const unsigned short int expectedSignature[] = { 0xAA55, 0x5AA5 };

word RX;                         // stores register data
word new_word;                   // hold data to be written to flash
word high_part;                  // temporary holder for upper part of word
char PinState[BSR_Length];       // holds pin data to move in and out
char input_file[80];             // holds name of input file
int c;                           // character being worked with
char ch;                         // temporary character from keyboard
FILE *in;                        // points to input file location
int board_rev;                   // TGXtra board revision code
int board_rev_port[] = { 0x430,  // {TGXtra ETM, SCV, TGX ETM} this port remains fixed for all hardware versions
                         0x374,
                         0x334 };
int verify;                      // TRUE if flash verify requested
int check_file;                  // TRUE if check file requested
int SMM_Please;                  // TRUE if SMM read requested

//#define FLASH_START_1M 0x3F00800
// 2021-03-31 DJF - flash start appears to be wrong
//const int FLASH_START_1M[] = {0x3F00800,// {TGXtra ETM, SCV, TGX ETM} FLASH_START is loaded one or the other of these
//                              0x3F00000,
//                              0x3F00800};
const int FLASH_START_1M[] = { 0x3F00000, // {TGXtra ETM, SCV, TGX ETM} FLASH_START is loaded one or the other of these
                               0x3F00000,
                               0x3F00000 };
const int FLASH_START_BB = 0x3F00800;		// DJF Note: these should probably be new device types?
const int FLASH_START_PP = 0x3F00800;

const int SMM_START       = 0x3F800;
const int SMM_TOP         = 0x3FFFF;
const int SMM_FLASH_START = 0x38000;
const int SMM_FLASH_TOP   = 0x38000 + 2047;

// values.
// FLASH_START_BB is only used for the original
// "breadboard" non-graphics  PCB

// local function prototypes
void bad_id(void);
void finished();


/******* JTAG1149 Commands for Intel 386EX ******************************/
//
// Each 386EX instruction is 4 bits long. Each M5 isntruction is 6 bits.
// The commands are stored here as strings and converted to binary
// as they are shifted out. Note: They are in bit reversed order.
//
// For example the INTEST instruction for the 386EX is 1001, so adding
// the BYPASS instruction for the M5 (which is 111111) makes the
// entire instruction sequence 11111101001 binary. The commands are stored
// here as strings and converted to binary as they are shifted out.
// NB They are in bit reversed order.
/*
char *BYPASS = "1111";     // Use BYPASS register in data path
char *EXTEST = "0000";     // External test mode (i.e. test the board)
char *SAMPLE = "1000";     // Sample/Preload instruction
char *IDCODE = "0100";     // Read ID CODE from chip
char *INTEST = "1001";     // On-chip system test (i.e. test the device)
char *HIGHZ = "0001";      // Place the device in hi-z mode
*/
char *BYPASS = "1111111111";     // Use BYPASS register in data path
char *EXTEST = "1111110000";     // External test mode (i.e. test the board)
char *SAMPLE = "1111111000";     // Sample/Preload instruction
char *IDCODE = "1111110100";     // Read ID CODE from chip
char *INTEST = "1111111001";     // On-chip system test (i.e. test the device)
char *HIGHZ = "1111110001";      // Place the device in hi-z mode

/********* JTAG Commands for MACH5 192/120 ***************************/

char *M5IDCODE = "1000001111";    // Device JTAG I.D. (Manuf,device,rev.)
char *M5EXTEST = "0000001111";    // External (system) test
char *M5BYPASS = "1111111111";    // Bypass both devices
char *M5SAMPLE = "0100001111";    // Sample/preload
char *M5HIGHZ  = "1000101111";    // Float pins

//The last 4 "1"s are the 386EX instruction (BYPASS)

// Defines for the Adafruit Pi LCD interface board

#define AF_BASE      100

#define AF_E         (AF_BASE + 13)
#define AF_RW        (AF_BASE + 14)
#define AF_RS        (AF_BASE + 15)

#define AF_DB4       (AF_BASE + 12)
#define AF_DB5       (AF_BASE + 11)
#define AF_DB6       (AF_BASE + 10)
#define AF_DB7       (AF_BASE +  9)

#ifdef BOARD_REV_1
#define AF_SELECT    (AF_BASE +  0)
#define AF_RIGHT     (AF_BASE +  1)
#define AF_DOWN      (AF_BASE +  2)
#define AF_UP        (AF_BASE +  3)
#define AF_LEFT      (AF_BASE +  4)
#define AF_BTN3      (AF_BASE +  5)
#define AF_BTN2      (AF_BASE +  6)
#define AF_BTN1      (AF_BASE +  7)
#endif

#ifdef BOARD_REV_2
// DJF Note: buttons placed on PCB upside down in relation to screen!
//           buttons 1, 2 & 3 removed as found to be unnecessary
//
#define AF_SELECT    (AF_BASE +  0)
#define AF_LEFT      (AF_BASE +  1)
#define AF_UP        (AF_BASE +  2)
#define AF_DOWN      (AF_BASE +  3)
#define AF_RIGHT     (AF_BASE +  4)
//#define AF_BTN3         (AF_BASE +  5)
//#define AF_BTN2         (AF_BASE +  6)
//#define AF_BTN1         (AF_BASE +  7)
#endif

// Global lcd handle:

static int lcdHandle;


static void LCDSetup(void)
{
   int i;

   // Input buttons

   // DJF extend to 8 buttond
   //for (i = 0 ; i <= 4 ; ++i)
   for (i = 0; i <= 7; ++i)
   {
      pinMode(AF_BASE + i, INPUT);
      pullUpDnControl(AF_BASE + i, PUD_UP);           // Enable pull-ups, switches close to 0v
   }

   // Control signals

   pinMode(AF_RW, OUTPUT); digitalWrite(AF_RW, LOW);  // Not used with wiringPi - always in write mode

   // The other control pins are initialised with lcdInit ()

   // DJF Note: configure for 4 x 20 display (was 2 x 16)
   //lcdHandle = lcdInit (2, 16, 4, AF_RS, AF_E, AF_DB4,AF_DB5,AF_DB6,AF_DB7, 0,0,0,0) ;
   lcdHandle = lcdInit(display_rows, display_cols, 4, AF_RS, AF_E, AF_DB4, AF_DB5, AF_DB6, AF_DB7, 0, 0, 0, 0);

   if (lcdHandle < 0)
   {
      fprintf(stderr, "lcdInit failed\n");
      exit(EXIT_FAILURE);
   }

   // populate programmabel character index 0 with backslash bitmap
   //
   lcdCharDef(lcdHandle, 0,backslashChar);
}

/*
 * waitForEnter:
 * On the Adafruit display, wait for the select button
 *********************************************************************************
 */

static void waitForEnter(void)
{
   printf("Press SELECT to continue: "); fflush(stdout);

   while (digitalRead(AF_SELECT) == HIGH) // Wait for push
   {
      delay(1);
   }

   while (digitalRead(AF_SELECT) == LOW)  // Wait for release
   {
      delay(1);
   }
   printf("OK\n");
}

/* Initialise new termianl I/O settings */
void initTermios(int echo)
{
   tcgetattr(0, &old);           // grab the old termianl I/O settings
   current = old;                // make new settings the same as old settings
   current.c_lflag &= ~ICANON;   // disable buffered I/O
   if (echo)
   {
      current.c_lflag |= ECHO;   // set echo mode
   }
   else
   {
      current.c_lflag |= ~ECHO;  // set no echo mode
   }
   tcsetattr(0, TCSANOW, &current); // use these new termianl I/O settings now
}

/* Restore old terminal I/O settings */
void resetTermios(void)
{
   tcsetattr(0, TCSANOW, &old);
}

/* Read a character - echo defines echo mode */
char getch_(int echo)
{
   char ch;
   initTermios(echo);
   ch = getchar();
   resetTermios();
   return ch;
}

/* Read 1 character without echo */
char getch(void)
{
   return getch_(0);
}

/* Read 1 character with echo */
char getche(void)
{
   return getch_(1);
}

char getNextPaddleChar(void)
{
   // old school DOS style progress/activity paddle wheel rotation
   if (paddleCharIndex < 7)
   {
      paddleCharIndex++;
   }
   else
   {
      // start again
      paddleCharIndex = 0;
   }
   return paddleChar[paddleCharIndex];
}

void Get_M5_Device_ID(void)
{
   const char   *p="01010101010101010101010101010101";     //dummy string
   //const char *m="00000111100000101011000000000011";     //M5-192/120 code
   //               0000                                     version number
   //                   0111100000101011                     part identification
   //                                   00000000001          company code
   //                                              1         IDCODE present flag
   //unsigned long int m1 = 0x0782B003;                    // The same in hex
   // This is after it has been flipped.

   char ID[ID_String_Length + 1];

   strcpy(ID, p);                                          //Fill with dummy string
   Send_Instruction_IN(strlen(M5IDCODE), M5IDCODE);        //Do not overwrite instr
                                                           //because it resides in the
                                                           //fixed string area.

   //printf("%s\n",ID);
   Send_Data(strlen(ID), ID);
   //printf("%s\n",ID);
   Flip_ID_String(strlen(ID), ID);                //make MSB first in array
   M5_Device_ID = Parse_ID(ID);
   printf("\nThe M5-192/120 Chip Identifier reads %08lX",M5_Device_ID);
   if (M5_Device_ID != m1)
   {
      printf(" but it should be 0x0782B003.");
      printf("\nCarry on anyway? ");
      ch = getche();
      if ((ch == 'y') || (ch == 'Y'))
      {
         printf("\nContinuing......  \n\n");
         return;
      }
      bad_id();
      finished();                        // Give up.
   }
   else printf(", which is correct.\n");
}

/**** Fucntion to get ID string from the Intel 386EX chip ****/

void Get_JTAG_Device_ID(void)
{

   const char *p = "01010101010101010101010101010101";   // dummy string
   char ID[ID_String_Length + 1];

   strcpy(ID, p);                                        // Fill with dummy string
   Send_Instruction_IN(strlen(IDCODE), IDCODE);          // Do not overwrite Instr!
   Send_Data(strlen(ID), ID);
   Flip_ID_String(strlen(ID), ID);                       // make MSB first in array
   Device_ID = Parse_ID(ID);

   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);

   printf("\nThe JTAG CPU Chip Identifier is 0x%08lX, ", Device_ID);
   lcdPrintf(lcdHandle, "CPUID is 0x%08lX", Device_ID);
   lcdPosition(lcdHandle, 0, 1);
   if (Device_ID == 0x00270013)
   {
      printf("('B step' part)\n");
      lcdPuts(lcdHandle, "B step part");
   }
   else if (Device_ID == 0x20270013)
   {
      printf("('C step' part)\n");
      lcdPuts(lcdHandle, "C step part");
   }
   else if (Device_ID == 0x28270013)
   {
      printf("('C step' part, 33MHz)\n");
      lcdPuts(lcdHandle, "C step part, 33MHz");
   }
   else
   {
      lcdPuts(lcdHandle, "* UNKNOWN PART *");
      printf("\nThis ID is wrong so the data dump probably won't work.\n");
      //      printf("Carry on anyway? ");
      printf("Press Select to exit");
      if(ignore_errors_opt == 0)
      {
         lcdPosition(lcdHandle, 0, display_rows - 1);
         lcdPuts(lcdHandle, "Press Select to Exit");
         waitForEnter();

         lcdClear(lcdHandle);
         if (command_line_opt != 0)
         {
            if (exitToCommandLine())
            {
               lcdClear(lcdHandle);
               lcdPosition(lcdHandle, 0, 0);
               lcdPuts(lcdHandle, "Application");
               lcdPosition(lcdHandle, 0, 1);
               lcdPuts(lcdHandle, "   Terminated");
               exit(0);
            }
         }
         bad_id();
         finished();
      }
      else
      {
         // just carry on
      }
   }
}

bool exitToCommandLine(void)
{
   int selectedItem;
   bool bResult = false; // default to shutdown

   sprintf(menuItems[0], "%-19s", "Exit");
   sprintf(menuItems[1], "%-19s", "Shutdown");
   selectedItem = select_menu_item(2, menuItems, "Exit or Shutdown?:");

   switch (selectedItem)
   {
   case 0:
   {
      bResult = true;
      break;
   }
   case 1:
   default:
   {
      bResult = false;
      break;
   }
   }
   return bResult;
}

void bad_id(void)
{
   printf("\nThe most likely causes of the problem are:\n");
   printf(" * TgXtra not powered;\n");
   printf(" * JTAG cable error (faulty or wrong cable);\n");
   //printf(" * Incompatible PC or parallel port;\n");
   printf(" * WAY386 PSU faulty (check +5v);\n");
   //printf(" * JTAG cable in the wrong PC port, (should be in LPT1);\n");
   printf(" * Intel or Vantis (AMD) changed the ID code;\n!");
   printf(" * 386EX CPU or M5-192/120 faulty;\n");
   printf(" * Connectivity fault between JTAG connector and CPU or M5-192/120.\n\n");
}

void Get_Board_Revision(int dev_type)
{
   // routine reads the WAY386 board revision code from I/O port

   Send_Instruction_IN(strlen(SAMPLE), SAMPLE); // SAMPLE/preload to initialise BSR
   Send_Instruction_IN(strlen(EXTEST), EXTEST); // Configure for external test

   //printf("Reading board rev port at 0x%08X for device type %d\r\n", board_rev_port[dev_type], dev_type);
   board_rev = IO_Read(PinState, board_rev_port[dev_type], dev_type);

   // DJF Note: I suspect the two bytes are in the wrong endian so examine MSB

   //printf("Raw board_rev: %08X\n", board_rev);
   //board_rev = (board_rev & 0x0F);
   board_rev = (board_rev & 0x0F00) / 0x100;

   //printf("masked  board_rev: %02X\n", board_rev);

   printf("Board revision code is %X ", board_rev);
   lcdPosition(lcdHandle, 0, 2);
   lcdPrintf(lcdHandle, "Board Rev: %X", board_rev);
   lcdPosition(lcdHandle, 0, display_rows - 1);
   switch (board_rev)
   {
   case 0:
      printf("(Pre-production)\n");
      FLASH_START = FLASH_START_1M[dev_type];
      lcdPuts(lcdHandle, "Pre-Production");
      break;

   case 1:
      printf("(Issue2)\n");
      FLASH_START = FLASH_START_1M[dev_type];
      lcdPuts(lcdHandle, "Issue 2");
      break;

   default:
      printf("(Unknown board version.)\n");
      printf("There is something wrong.\n");
      lcdPuts(lcdHandle, "Something is wrong");
      FLASH_START = FLASH_START_1M[dev_type];   // as good as any!
      break;
   }
   FLASH_TOP = FLASH_START + 0x100000 - 1;
}

// DJF TESTING - function to test each of the outputs
// Warning - do not run this function when connected
// to the target device. It is intended that a scope
// or logic analyser will be monitoring the outputs
//
void testOutputs(void)
{

   digitalWrite(JTAG_TCK, HIGH);
   delay(500);
   digitalWrite(JTAG_TCK, LOW);
   delay(500);
   digitalWrite(JTAG_TCK, HIGH);
   delay(500);
   digitalWrite(JTAG_TCK, LOW);
   digitalWrite(JTAG_TMS, HIGH);
   delay(500);
   digitalWrite(JTAG_TMS, LOW);
   digitalWrite(JTAG_TDI, HIGH);
   delay(500);
   digitalWrite(JTAG_TDI, LOW);
   digitalWrite(JTAG_TRST, HIGH);
   delay(500);
   digitalWrite(JTAG_TRST, LOW);
   digitalWrite(JTAG_WR, HIGH);
   delay(500);
   digitalWrite(JTAG_WR, LOW);
}

/*
 * The works
 *********************************************************************************
 */

int main(int argc, char *argv[])
{

   char sysfilename[256];
   int thumb;
   int debug = 0;

   //int cpu_max_freq;
   int result = 0;

   char pi_model[128];
   FILE *fp;

   // DJF TESTING
   //uint32_t t0 = ccnt_read();
   //uint32_t t1 = ccnt_read();
   ///printf("\n%u\n", t1-t0);
   //volatile uint64_t n = 100000000;
   //while(n > 0) n--;
   //t1 = ccnt_read();
   //printf("\n%u\n", t1-t0);

// DJF TESTING
//test_wait_cycles();
//exit(0);

   // variables for ini file processing
   //
   const char * custname = "Customer Name Is Not Known";
   ini_t * jtag_config = ini_load("/home/pi/jtag/jtag.ini");

   // DJF Note: CPU appears to run at max freq when dump is in progess
   //
   // The JTAG puls width needs to be roughly 1us to work reliably.
   // The following file access reads the maximum CPU frequency in
   // KHz (I think? A 1.4GHz Pi 3B= returns 1,400,000).
   // Divide this value by 1000 to get the number of cycles to wait for 1us
   //
   fp = fopen("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq", "r");
   if (fp != NULL)
   {
      result = fscanf(fp, "%d", &cpu_max_freq);
      fclose(fp);
      if(result == 0)
      {
         // failed to get value. Set a safe default
         // pi model A/A+ s only 700MHz
         cpu_max_freq = 700000;
      }
   }
   cycles_to_wait = cpu_max_freq / 1000; // this gives us 1us wait time

//printf("\ncycles_to_wait = %d (1)\r\n", cycles_to_wait);

   if (jtag_config != NULL)
   {
      ini_sget(jtag_config, "settings", "customer_name", NULL, &custname);

      if (!ini_sget(jtag_config, "settings", "pulse_width_ns", "%d", &pulse_width_ns))
      {
         pulse_width_ns = 1000; // 1us
      }

//printf("\npulse_width_ns = %d\r\n", pulse_width_ns);

      cycles_to_wait = (cpu_max_freq * pulse_width_ns) / 1000000;

//printf("\ncycles_to_wait = %d (2)\r\n", cycles_to_wait);

      // this will override CPU based setting from cpu_max_freq
      if (!ini_sget(jtag_config, "settings", "clock_timing", "%d", &cycles_to_wait))
      {
         cycles_to_wait = (cpu_max_freq * pulse_width_ns) / 1000000;
      }

      if (!ini_sget(jtag_config, "settings", "wait_mode", "%d", &wait_mode))
      {
         wait_mode = CCR_LOOP;
      }

      //ini_sget(jtag_config, "display", "type", NULL, &display_type);
      ini_sget(jtag_config, "display", "rows", "%d", &display_rows);
      ini_sget(jtag_config, "display", "columns", "%d", &display_cols);

      if (display_cols > 20)
      {
         // restrict to 20 columns
         // displays are normally 2x16, 4x16 or 4x20
         display_cols = 20;
      }

      // handle special settings for Flowbird and development use only
      if (!ini_sget(jtag_config, "special_settings", "diagnostics", "%d", &diagnostic_menu))
      {
         diagnostic_menu = 0;
      }
      if (!ini_sget(jtag_config, "special_settings", "debug_level", "%d", &debug_level))
      {
         debug_level = 0;
      }
      if (!ini_sget(jtag_config, "special_settings", "command_line", "%d", &command_line_opt))
      {
         command_line_opt = 0;
      }
      if (!ini_sget(jtag_config, "special_settings", "ignore_errors", "%d", &ignore_errors_opt))
      {
         ignore_errors_opt = 0;
      }
      if (!ini_sget(jtag_config, "special_settings", "verbose_debug", "%d", &verbose_debug))
      {
         verbose_debug = 0;
      }

      // this must not be called until after everything is finished with 
      // as it invalidates all string pointers returned by the library
      //ini_free(jtag_config);

      if (debug_level > 0)
      {
         printf("\nini file found, customer name is %s and cycles_to_wait is %d\n", custname, cycles_to_wait);
         printf("diags = %d, debug = %d, command line opt = %d\n", diagnostic_menu, debug_level, command_line_opt);
         printf("CPU Max Frequncy = %d\r\n", cpu_max_freq);
      }
   }
   else
   {
      // use defaults
      //cycles_to_wait = 1000;
      display_rows = 4;
      display_cols = 20;
      diagnostic_menu = 0;
      debug_level = 0;
      command_line_opt = 0;
      ignore_errors_opt = 0;
   }

// DJF TESTING
printf("\ncycles_to_wait = %d (3)\r\n", cycles_to_wait);

   fp = fopen("/proc/device-tree/model", "r");
   if (fp != NULL)
   {
      if (fgets(pi_model, 128, fp) != NULL)
      {
         // DJF DEBUG TO BE REMOVED
         printf("Raspberry Pi Model is: \"%s\"\n", pi_model);
      }
      else
      {
         // DJF DEBUG TO BE REMOVED
         printf("Raspberry Pi Model is: \"UNKNOWN\"\n");
      }
      fclose(fp);
   }

   printf("Flowbird JTAG Adaptor for TGX Range\n");
   printf("===================================\n");


   // set-up for wiringPi PIN naming
   wiringPiSetup();

   // configure the I2C port expander for display and keyboard
   mcp23017Setup(AF_BASE, 0x20);

   LCDSetup();


   //system("sudo mount -t vfat -o uid=pi,gid=pi /dev/sda1 /home/pi/jtagdisk");

   thumb = system("ls /dev/sda");
   // Thumbrive = 0 if thumbdrive detected or 512 if not available
   // if it IS available, it should already mounted to ~/jtagdisk

   //printf("DEBUG:USB thumb drive is %d\n", thumb);

   //
   // DJF Notes on the following mount technique
   //
   // the raspberry pi OS appears to need to know the UUID of a USB drive
   // in order for it to be mounted automatically during system startup.
   // As a user will just use any available USB drive the UUID will not
   //  be known.
   //
   // At least, I have not been able to find a way to automatically mount any
   // supplied USB drive on the moutn point.
   //
   // This application hopes to have a USB drive mounted in /home/pi/jtagdisk.
   //
   // A file has been created in the unmounted /home/pi/jtagdisk folder.
   //
   // If the file not_mounted can be detected in the folder then the USB drive
   // has not been mounted. If the USB drive is mounted then the not_mounted
   // file will not be present.
   //



   if (thumb == 0)
   {
      // check that it is actually mounted!
      // there will be a file called "not_mounted" if it isn't!

      if (access("/home/pi/jtagdisk/not_mounted", F_OK) == 0)
      {
         // The "not_mounted" file was found in the dirrectory
         printf("USB Flash disk is not mounted\n");
         debug = 1;

         // try to mount it now
         //
         system("sudo mount -t vfat -o uid=pi,gid=pi /dev/sda1 /home/pi/jtagdisk");

         // is it mounted now?
         //
         if (access("/home/pi/jtagdisk/not_mounted", F_OK) == 0)
         {
            printf("USB Flash disk could not be mounted\n");
            //return 0; // exit at this point - will need to force reboot or something
            sprintf(filepath, "/boot"); // write to the OS SD card as a last resort
            debug = 2;
         }
         else
         {
            printf("USB Flash disk is now mounted\n");
            // save files to USB drive
            sprintf(filepath, "/home/pi/jtagdisk");
            debug = 3;
         }
      }
      else
      {
         // the "not_mounted" file was not found because the USB disk is mounted in its directory
         printf("USB Flash disk was already mounted\n");
         // save files to USB drive
         sprintf(filepath, "/home/pi/jtagdisk");
      }
   }
   else
   {
      printf("USB device not found\n");
      debug = 5;
      // save files on boot partition of SD card :-(
      sprintf(filepath, "/boot");
   }
   //printf("filepath is: %s\n", filepath);

   lcdClear(lcdHandle);

   //// DJF DEBUG TO BE REMOVED
   //   lcdPosition (lcdHandle, 0, 0); lcdPrintf(lcdHandle, filepath );
   //   lcdPosition (lcdHandle, 0, 1); lcdPrintf(lcdHandle, "DEBUG: %d", debug);
   //   waitForEnter();
   if ((debug == 5) || (debug == 2))
   {
      // A USB thumb drive was not found so ask for one rather than risk
      // trashing the OS SD card image.
      //
      lcdPosition(lcdHandle, 0, 0); lcdPuts(lcdHandle, "Please insert a USB ");
      lcdPosition(lcdHandle, 0, 1); lcdPuts(lcdHandle, "drive in a USB port ");
      lcdPosition(lcdHandle, 0, 2); lcdPuts(lcdHandle, "and press SELECT    ");
      lcdPosition(lcdHandle, 0, 3); lcdPuts(lcdHandle, "to reboot device    ");
      waitForEnter();
      reboot_system();
      return 0;
   }

   // 2021-05-19 DJF - menu changes
   showWelcomeScreen();

   piHiPri(20); // get beter delayMicroseconds() resolution

   // configure he JTAG GPIO pins
   configureGPIO();

   //printf("Power TGX device now and press select on JTAG device\n");

   // 2021-05-18 DJF - new menu structure
   /*
   waitForEnter(); // Wait until TGX or SCV is powered
   */

   //  DJF TESTING - run output tests
   /*
   lcdClear(lcdHandle);
   testOutputs();
   return 0; // exit now
   */
   // clear the display
   lcdClear(lcdHandle);

   topLevelMenu();

   // if reached here - exit to command line
   printf("\nReturning control to 386EX CPU.\n");
   printf("Calling Restore_Idle()\n");
   Restore_Idle();      // Let go of the processor...
   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);
   lcdPuts(lcdHandle, "Finished...");
   delay(2000);
   lcdClear(lcdHandle);

   exit(0);
}

void retest_signature(int deviceType)
{
   struct signatureBlockType theSignatureBlock;

   // try to read 0x40004 again to check it can still be read after JTAG activity
   // if it can't be read then JTAG dump is probably no good
   theSignatureBlock = testForSignature(0x40004, deviceType);
   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);
   lcdPrintf(lcdHandle, "Signature: %04X%04X", theSignatureBlock.theSignature[0], theSignatureBlock.theSignature[1]);
   lcdPosition(lcdHandle, 0, 1);
   if (!theSignatureBlock.signatureOk)
   {
      lcdPuts(lcdHandle, "...is incorrect!");
      printf("WARNING: Signature is incorrect\n");
   }
   else
   {
      lcdPuts(lcdHandle, "...is still correct");
   }
   printf("Signature bytes read 0x%04X%04X\n", theSignatureBlock.theSignature[0], theSignatureBlock.theSignature[1]);
   lcdPosition(lcdHandle, 0, display_rows - 1);
   lcdPuts(lcdHandle, "...Press Select...");
   waitForEnter();
}

void promptForEnter()
{
   // puts prompt to pres enter/select on last row
   lcdPosition(lcdHandle, 0, display_rows - 1);
   lcdPuts(lcdHandle, "...Press Select...");
   waitForEnter();
}

bool selectBoolean(char* title)
{
   int selectedItem;

   sprintf(menuItems[0], "%-19s", "No");
   sprintf(menuItems[1], "%-19s", "Yes");
   selectedItem = select_menu_item(2, menuItems, title);

   if (selectedItem == 0)
   {
      return FALSE;
   }
   else
   {
      return TRUE;
   }
}

int selectDeviceType(void)
{
   int deviceType;
   int selectedItem;

   sprintf(menuItems[0], "%-19s", "ETM TGXtra");
   sprintf(menuItems[1], "%-19s", "SCV");
   sprintf(menuItems[2], "%-19s", "ETM TGX");
   selectedItem = select_menu_item(3, menuItems, "Select device type:");

   switch (selectedItem)
   {
   case 0:
   default:
   {
      deviceType = ETM_TGXtra;
      //printf("DEBUG: selected device type is ETM(%d)\n", ETM);
      break;
   }
   case 1:
   {
      deviceType = SCV;
      //printf("DEBUG: selected device type is SCV(%d)\n", SCV);
      break;
   }
   case 2:
   {
      deviceType = ETM_TGX;
      //printf("DEBUG: selected device type is ETM_TGX(%d)\n", ETM_TGX);
      break;
   }
   }
   return deviceType;
}

unsigned long int selectMaximumMemory(int thisDeviceType)
{
   unsigned long int maximum = 0xFFFFF;
   int selectedItem;

   switch (thisDeviceType)
   {
   case SCV:
      sprintf(menuItems[0], "%-19s", "1MB");
      sprintf(menuItems[1], "%-19s", "2MB");
      sprintf(menuItems[2], "%-19s", "3MB");
      selectedItem = select_menu_item(3, menuItems, "Select memory size:");
      switch (selectedItem)
      {
      default:
      case 0:
         maximum = 0x0FFFFF; // 1MB
         break;
      case 1:
         maximum = 0x1FFFFF; // 2MB
         break;
      case 2:
         maximum = 0x2FFFFF; // 3MB
         break;
      }
      break;
   case ETM_TGX:
      sprintf(menuItems[0], "%-19s", "1MB");
      sprintf(menuItems[1], "%-19s", "2MB");
      sprintf(menuItems[2], "%-19s", "3MB");
      sprintf(menuItems[3], "%-19s", "4MB");
      sprintf(menuItems[4], "%-19s", "5MB");
      selectedItem = select_menu_item(5, menuItems, "Select memory size:");
      switch (selectedItem)
      {
      default:
      case 0:
         maximum = 0x0FFFFF; // 1MB
         break;
      case 1:
         maximum = 0x1FFFFF; // 2MB
         break;
      case 2:
         maximum = 0x2FFFFF; // 3MB
         break;
      case 3:
         maximum = 0x3FFFFF; // 4MB
         break;
      case 4:
         maximum = 0x4FFFFF; // 5MB
         break;
      }
      break;
   default: // greatest choice
   case ETM_TGXtra:
      sprintf(menuItems[0], "%-19s", "1MB");
      sprintf(menuItems[1], "%-19s", "2MB");
      sprintf(menuItems[2], "%-19s", "3MB");
      sprintf(menuItems[3], "%-19s", "4MB");
      sprintf(menuItems[4], "%-19s", "5MB");
      sprintf(menuItems[5], "%-19s", "8MB");
      selectedItem = select_menu_item(6, menuItems, "Select memory size:");
      switch (selectedItem)
      {
      default:
      case 0:
         maximum = 0x0FFFFF; // 1MB
         break;
      case 1:
         maximum = 0x1FFFFF; // 2MB
         break;
      case 2:
         maximum = 0x2FFFFF; // 3MB
         break;
      case 3:
         maximum = 0x3FFFFF; // 4MB
         break;
      case 4:
         maximum = 0x4FFFFF; // 5MB
         break;
      case 5:
         maximum = 0x7FFFFF; // 8MB
         break;
      }
      break;
   }
   return maximum;
}

void show_dump_stats(time_t start_time, time_t end_time)
{
   time_t duration;
   struct tm *time_info;
   char * c_time_string;
   int durationHours, durationMinutes, durationSeconds, durationRemainder;

   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);
   time_info = localtime(&start_time);

   lcdPuts(lcdHandle, "Dump Finished");
   lcdPosition(lcdHandle, 0, 1);
   lcdPrintf(lcdHandle, "%02d:%02d:%02d to", time_info->tm_hour, time_info->tm_min, time_info->tm_sec);

   c_time_string = ctime(&end_time);
   printf("dump finished at %s\n", c_time_string);

   time_info = localtime(&end_time);
   lcdPosition(lcdHandle, 12, 1);
   lcdPrintf(lcdHandle, "%02d:%02d:%02d", time_info->tm_hour, time_info->tm_min, time_info->tm_sec);

   //duration = end_time - start_time;
   duration = difftime(end_time, start_time);

   durationHours = duration / 3600;
   durationRemainder = duration % 3600;
   durationMinutes = durationRemainder / 60;
   durationSeconds = durationRemainder % 60;

   sprintf(c_time_string, "%02d:%02d:%02d", durationHours, durationMinutes, durationSeconds);
   printf("total time to dump: %s\n", c_time_string);

   lcdPosition(lcdHandle, 0, 2);
   lcdPrintf(lcdHandle, "Duration: %s", c_time_string);

   lcdPosition(lcdHandle, 0, display_rows - 1);
   lcdPuts(lcdHandle, "...Press Select...");

   waitForEnter();
}

struct signatureBlockType testForSignature(unsigned long int StartAddress, int dev_type)
{
   unsigned long int index, index2;
   unsigned short int bytesRead;
   int count = 0;
   int bytesToRead = 0;
   int bytesRemaining = 0;
   struct signatureBlockType theSignatureBlock;
   unsigned long int EndAddress;

   theSignatureBlock.signatureOk = TRUE;

   EndAddress = StartAddress + 4;

   count = 0;

   bytesRemaining = (EndAddress - StartAddress);
   for (index = StartAddress; index <= EndAddress; index += 0x10)
   {
      printf("0x%08lX  ", index);
      if (bytesRemaining > 0x10)
      {
         bytesToRead = 0x10;
      }
      else
      {
         bytesToRead = bytesRemaining;
      }

      for (index2 = 1; index2 < bytesToRead; index2 += 2)
      {
         bytesRead = Memory_Read(PinState, index + index2, dev_type);
         //printf(" %04X", Memory_Read(PinState, index + index2, dev_type));
         printf(" %04X", bytesRead);
         theSignatureBlock.theSignature[count] = bytesRead;
         if (theSignatureBlock.theSignature[count] != expectedSignature[count])
         {
            theSignatureBlock.signatureOk = FALSE;
         }
         count++;
      }
      printf("   ");
      bytesRemaining -= 2;
      for (index2 = 1; index2 < bytesToRead; index2 += 2)
      {
         show_char(Memory_Read(PinState, index + index2, dev_type));
      }
      printf("\n");
   }
   return theSignatureBlock;
}

void show_char(int hex_val)
{
   unsigned char upper, lower;

   upper = hex_val >> 8;
   lower = hex_val & 0xFF;

   if (lower >= 20)
   {
      printf("%c", lower);
   }
   else
   {
      printf(".");
   }
   if (upper >= 20)
   {
      printf("%c", upper);
   }
   else
   {
      printf(".");
   }
}

void finished()
{
   int index;

   printf("\nReturning control to 386EX CPU.\n");
   printf("Calling Restore_Idle()\n");
   Restore_Idle();      // Let go of the processor...

   lcdPosition(lcdHandle, 0, 0);
   lcdPuts(lcdHandle, "     Finished       ");
   lcdPosition(lcdHandle, 0, 1);
   lcdPuts(lcdHandle, "   Shutting Down...");

   // clear remainder of display
   for (index = 2; index < 4; index++)
   {
      lcdPosition(lcdHandle, 0, index);
      lcdPuts(lcdHandle, "                    ");
   }

   // shutdown the system
   system("sudo shutdown -h now");

   exit(0);    // ...exit.
}

void dump_to_file(unsigned long int start, unsigned long int end, int dev_type, char *filename)
{
   unsigned long int index;
   unsigned long address;
   unsigned short int data;
   FILE *dataFile;
   unsigned char dataByte[2];
   unsigned long int progress_block_size;
   unsigned long int bytes_read;
   unsigned long int bytes_to_read;
   int progress_blocks;
   char progress[20 + 1];
   char progress_char[1 + 1] = { 0xFF, 0x00 }; // block character from Hitachi data sheet
   char bytes_read_string[50 + 1];

   if (end <= start) // check for silly input
   {
      end = start + 1;
   }

   bytes_to_read = end - start;
   if (bytes_to_read > display_cols)
   {
      // there is enough length to show 20 blocks of progress
      progress_block_size = bytes_to_read / display_cols;
   }
   else
   {
      progress_block_size = 1; // this isn't right but it will do!
   }

   if (debug_level > 0)
   {
      printf("DEBUG: dumping %ld to %ld to file \'%s\'\n", start, end, filename);
   }

   dataFile = fopen(filename, "w+b");

   if (debug_level > 0)
   {
      if (dataFile == NULL)
      {
         // failed to open file
         printf("DEBUG: fopen failed error: %s\n", strerror(errno));
      }
      else
      {
         printf("DEBUG: file opened\n");
      }
   }

   bytes_read = 0;
   progress_blocks = 0;

   for (index = 0; index < 4; index++)
   {
      lcdPosition(lcdHandle, 0, index);
      lcdPuts(lcdHandle, "                    "); // clear line on display
   }

   lcdPosition(lcdHandle, 0, 0);
   lcdPuts(lcdHandle, "dumping mem to file");

   for (address = start; address < end; address += 2)
   {
      data = Memory_Read(PinState, address, dev_type);
      bytes_read += 2;

      if ((bytes_read & 0x3FF) == 0) // start address may not have been 0!
      {
         sprintf(bytes_read_string, "%ldKB read", bytes_read / 1024);
         printf("%ldKB read\r", bytes_read / 1024);
         fflush(stdout);

         lcdPosition(lcdHandle, 0, 1);
         lcdPuts(lcdHandle, bytes_read_string);
      }
      dataByte[0] = data & 0xFF;             // LSB
      dataByte[1] = (data & 0xFF00) / 0x100; // MSB

      if ((fwrite(&dataByte[1], sizeof(unsigned char), 1, dataFile) != 1) ||
          (fwrite(&dataByte[0], sizeof(unsigned char), 1, dataFile) != 1))
      {
         printf("Error writing to file\n");
      }

      if ((bytes_read % progress_block_size) == 0)
      {
         strcpy(progress, "");
         progress_blocks = bytes_read / progress_block_size;
         for (index = 0; index < progress_blocks; index++)
         {
            strncat(progress, progress_char, display_cols); // show 20 progress blocks total (screen width)
         }
         lcdPosition(lcdHandle, 0, display_rows - 1);
         lcdPuts(lcdHandle, progress);
      }
      // see if the user wants to cancel - just now an again
/*
      if((bytes_read & 0xF) == 0)
      {
         if (digitalRead(AF_SELECT) == LOW)
         {
            printf("\nCancel button pressed\r\n");
            lcdClear(lcdHandle);
            if(selectBoolean("Cancel dump?"))
            {
               address = end;
            }
            else
            {
               // user did not intend to cancel
               // reset the display and continue
               for (index = 0; index < 4; index++)
               {
                  lcdPosition(lcdHandle, 0, index);
                  lcdPuts(lcdHandle, "                    "); // clear line on display
               }

               lcdPosition(lcdHandle, 0, 0);
               lcdPuts(lcdHandle, "dumping mem to file");
               sprintf(bytes_read_string, "%ldKB read", bytes_read / 1024);
               lcdPosition(lcdHandle, 0, 1);
               lcdPuts(lcdHandle, bytes_read_string);
               lcdPosition(lcdHandle, 0, display_rows - 1);
               lcdPuts(lcdHandle, progress);
            }
         }
      }*/
   }
   fclose(dataFile);
   printf("\nBinary file %s has been created (or overwritten!)\n", filename);
}

struct addressRangeType selectAddressRange(unsigned long int max_address)
{
   int selectedItem;
   struct addressRangeType stAddressRange;

   bool addressRangeConfirmed = FALSE;

   stAddressRange.startAddress = 0;
   stAddressRange.endAddress = max_address;

   while (!addressRangeConfirmed)
   {
      sprintf(menuItems[0], "%-19s", "Full Memory Dump");
      sprintf(menuItems[1], "%-19s", "Custom Addr. Range");
      sprintf(menuItems[2], "%-19s", "[return]");
      selectedItem = select_menu_item(3, menuItems, "Select Addr. Range:");

      switch (selectedItem)
      {
      case 0:
         stAddressRange.startAddress = 0;
         stAddressRange.endAddress = max_address;
         addressRangeConfirmed = confirmAddressRange(stAddressRange);
         break;
      case 1:
         stAddressRange.startAddress = selectHexAddress("Start offset(HEX):", 0);
         stAddressRange.endAddress = selectHexAddress("End offset(HEX):", max_address);
         addressRangeConfirmed = confirmAddressRange(stAddressRange);
         break;

      case 2:
         // set start address and end address the same to cancel dump
         stAddressRange.startAddress = 0;
         stAddressRange.endAddress = 0;
         addressRangeConfirmed = true; // exit while loop

      }
   } // while not confirmed address range

   return stAddressRange;
}

bool confirmAddressRange(struct addressRangeType stAddressRange)
{
   int selectedItem;

   char menu_title[21];
   lcdClear(lcdHandle);
   sprintf(menuItems[0], "%-19s", "Yes");
   sprintf(menuItems[1], "%-19s", "No");
   sprintf(menu_title, "%08lX to %08lX", stAddressRange.startAddress, stAddressRange.endAddress);
   selectedItem = select_menu_item(2, menuItems, menu_title);

   switch (selectedItem)
   {
   case 0:
      return TRUE;
      break;
   default:
   case 1:
      return FALSE;
      break;
   }
}

unsigned long int selectHexAddress(char* title, unsigned long int defaultAddress)
{
   unsigned long int selectedAddress;
   bool waitForRelease = FALSE;
   int cursorPosition = 0;
   char hexString[9]; // 8 hex digits plus a string null terminator

   selectedAddress = defaultAddress; // set initial display address to that supplied

   sprintf(hexString, "%08lX", selectedAddress);

   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);
   lcdPrintf(lcdHandle, "%s", title);
   lcdPosition(lcdHandle, 0, 1);
   lcdPuts(lcdHandle, hexString);
   lcdPosition(lcdHandle, 0, 2);
   lcdPuts(lcdHandle, "^"); // this is cursor set to first character

   // check for button presses until select button
   //
   for (;;) // for ever!
   {
      if (waitForRelease)
      {
         // if any key is still pressed then wait for it to be released
         if ((digitalRead(AF_UP) == LOW) || (digitalRead(AF_DOWN) == LOW) ||
            (digitalRead(AF_LEFT) == LOW) || (digitalRead(AF_RIGHT) == LOW) ||
            (digitalRead(AF_SELECT) == LOW))
         {
            continue;
         }
         else
         {
            waitForRelease = FALSE;
         }
      }

      if (digitalRead(AF_LEFT) == LOW)
      {
         if (cursorPosition > 0)
         {
            lcdPosition(lcdHandle, cursorPosition--, 2); // move and then decrement
            lcdPuts(lcdHandle, " ");
            lcdPosition(lcdHandle, cursorPosition, 2);
            lcdPuts(lcdHandle, "^");
         }
         // else leave cursor at 0
         waitForRelease = TRUE;
      }

      if (digitalRead(AF_RIGHT) == LOW)
      {
         if (cursorPosition < 7)
         {
            lcdPosition(lcdHandle, cursorPosition++, 2); // move and then decrement
            lcdPuts(lcdHandle, " ");
            lcdPosition(lcdHandle, cursorPosition, 2);
            lcdPuts(lcdHandle, "^");
         }
         // else leave cursor at 7
         waitForRelease = TRUE;
      }

      if (digitalRead(AF_UP) == LOW)
      {
         hexString[cursorPosition] = incrementHexDigit(hexString[cursorPosition]);
         lcdPosition(lcdHandle, 0, 1);
         lcdPuts(lcdHandle, hexString);
         waitForRelease = TRUE;
      }

      if (digitalRead(AF_DOWN) == LOW)
      {
         hexString[cursorPosition] = decrementHexDigit(hexString[cursorPosition]);
         lcdPosition(lcdHandle, 0, 1);
         lcdPuts(lcdHandle, hexString);
         waitForRelease = TRUE;
      }

      if (digitalRead(AF_SELECT) == LOW)
      {
         waitForRelease = TRUE;
         break; // this will end the forever loop
      }

   } // end for ever loop

   selectedAddress = (unsigned long int)strtol(hexString, NULL, 16);

   return selectedAddress;
}

bool selectFilename(char * filename)
{
   // file path is passed in in file name

   DIR *dir;
   struct dirent *ent;
   int menuItemCount = 0;
   int selected_file_index = 0;

   if ((dir = opendir(filename)) != NULL)
   {
      // print all the files and directories
      while (((ent = readdir(dir)) != NULL) && (menuItemCount < MAX_MENU_ITEMS))
      {
         if (ent->d_type == DT_REG)
         {
            // if this has the .bin or .hex extension add it to the list
            const char *ext = strrchr(ent->d_name, '.');
            if ((!ext) || (ext == ent->d_name))
            {
               // not interested in this file - bit of Welsh logic here - coding the else!
            }
            else
            {
               if ((strcmp(ext, ".bin") == 0) || (strcmp(ext, ".hex") == 0) ||
                  (strcmp(ext, ".BIN") == 0) || (strcmp(ext, ".HEX") == 0))
               {
                  // add this file to the lsit of menu items
                  sprintf(menuItems[menuItemCount++], "%s", ent->d_name);
               }
            }
         }
      } // end of while loop

      if (menuItemCount > 0)
      {
         // select the file from the list of menu items
         selected_file_index = select_menu_item(menuItemCount, menuItems, "Select File To Load");
         strcat(filename, "/");
         strcat(filename, menuItems[selected_file_index]);
         return TRUE;
      }
      else
      {
         // did not select a file
         return FALSE;
      }
   }
   else
   {
      // could not open directory
      printf("DJF DEBUG: Could not open directory \"%s\"\n", filename);
      lcdClear(lcdHandle);
      lcdPosition(lcdHandle, 0, 0);
      lcdPuts(lcdHandle, "\"Image Files\" folder");
      lcdPosition(lcdHandle, 0, 1);
      lcdPuts(lcdHandle, "was not found!");
      lcdPosition(lcdHandle, 0, 3);
      lcdPosition(lcdHandle, 0, display_rows - 1);
      lcdPuts(lcdHandle, " ...Press SELECT...");
      waitForEnter();
      return FALSE;
   }
}

char incrementHexDigit(char hexDigit)
{
   char retHexDigit = hexDigit;
   // this can probably be done with a bit of logic but my brain isn't working
   switch (hexDigit)
   {
   case '0':
      retHexDigit = '1';
      break;
   case '1':
      retHexDigit = '2';
      break;
   case '2':
      retHexDigit = '3';
      break;
   case '3':
      retHexDigit = '4';
      break;
   case '4':
      retHexDigit = '5';
      break;
   case '5':
      retHexDigit = '6';
      break;
   case '6':
      retHexDigit = '7';
      break;
   case '7':
      retHexDigit = '8';
      break;
   case '8':
      retHexDigit = '9';
      break;
   case '9':
      retHexDigit = 'A';
      break;
   case 'A':
      retHexDigit = 'B';
      break;
   case 'B':
      retHexDigit = 'C';
      break;
   case 'C':
      retHexDigit = 'D';
      break;
   case 'D':
      retHexDigit = 'E';
      break;
   case 'E':
   case 'F': // dont increment!
      retHexDigit = 'F';
      break;
   }
   return retHexDigit;
}

char decrementHexDigit(char hexDigit)
{
   char retHexDigit = hexDigit;
   // this can probably be done with a bit of logic but my brain isn't working
   switch (hexDigit)
   {
   case '0':
      retHexDigit = '0'; // do not decrement
      break;
   case '1':
      retHexDigit = '0';
      break;
   case '2':
      retHexDigit = '1';
      break;
   case '3':
      retHexDigit = '2';
      break;
   case '4':
      retHexDigit = '3';
      break;
   case '5':
      retHexDigit = '4';
      break;
   case '6':
      retHexDigit = '5';
      break;
   case '7':
      retHexDigit = '6';
      break;
   case '8':
      retHexDigit = '7';
      break;
   case '9':
      retHexDigit = '8';
      break;
   case 'A':
      retHexDigit = '9';
      break;
   case 'B':
      retHexDigit = 'A';
      break;
   case 'C':
      retHexDigit = 'B';
      break;
   case 'D':
      retHexDigit = 'C';
      break;
   case 'E':
      retHexDigit = 'D';
      break;
   case 'F':
      retHexDigit = 'E';
      break;
   }
   return retHexDigit;
}

int select_menu_item(int numberOfItems, char(*menuItems)[20], char* menuTitle)
{
   int itemsToShow;           // how many items on the display (1..3)
   int selectedRow;           // which of the three rows currently has the marker

   int firstItemIndex;        // which menu item to display on the first row
   int itemIndex;             // the currently selected menu item (0..numberItems -1)
   bool selected = FALSE;
   bool waitForRelease = FALSE;

   // display the title of this menu selection
   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);
   lcdPuts(lcdHandle, menuTitle);

   // Limit the number of items on screen to smaller of number of items or 3
   if (numberOfItems > (display_rows - 1))
   {
      itemsToShow = display_rows - 1;
   }
   else
   {
      itemsToShow = numberOfItems;
   }

   itemIndex = 0;       // index into menu item array (range 0..9)
   firstItemIndex = 0;  // start with first menu item in array
   selectedRow = 1;     // which of the 3 menu rows currently has the cursor (range 1..3 - row 0 has the menu title)

   // display upto the first three of the menu items
   display_menu_items(itemsToShow, firstItemIndex, menuItems);

   // put marker next to first item
   lcdPosition(lcdHandle, 0, selectedRow);
   lcdPutchar(lcdHandle, '*');

   selected = FALSE;
   while (!selected)
   {
      // wait for up, down or select key press
      if (waitForRelease)
      {
         if ((digitalRead(AF_UP) == LOW) ||
            (digitalRead(AF_DOWN) == LOW) ||
            (digitalRead(AF_SELECT) == LOW))
         {
            continue;
         }
         else
         {
            waitForRelease = FALSE;
         }
      }

      // process whichever key press
      if (digitalRead(AF_UP) == LOW)
      {
         if (itemIndex > 0)
         {
            itemIndex--;
            if (selectedRow > 1)
            {
               // clear the current item marker as it is changing
               lcdPosition(lcdHandle, 0, selectedRow);
               lcdPutchar(lcdHandle, ' ');

               selectedRow--;
               lcdPosition(lcdHandle, 0, selectedRow);
               lcdPutchar(lcdHandle, '*');
            }
            else
            {
               // already at the top displayed menu item so need to scroll up the list
               firstItemIndex--;
               itemsToShow = numberOfItems - firstItemIndex;
               if (itemsToShow > (display_rows - 1))
               {
                  itemsToShow = display_rows - 1;
               }
               display_menu_items(itemsToShow, firstItemIndex, menuItems);
            }
         }
         else
         {
            // wrap around to end
            // clear the current marker as it is changing
            lcdPosition(lcdHandle, 0, selectedRow);
            lcdPutchar(lcdHandle, ' ');

            itemIndex = numberOfItems - 1;
            if (numberOfItems > (display_rows - 1))
            {
               selectedRow = display_rows - 1;
               itemsToShow = 3;
               firstItemIndex = numberOfItems - 3;
            }
            else
            {
               selectedRow = numberOfItems;
               itemsToShow = numberOfItems;
               firstItemIndex = 0;
            }
            display_menu_items(itemsToShow, firstItemIndex, menuItems);
            lcdPosition(lcdHandle, 0, selectedRow);
            lcdPutchar(lcdHandle, '*');
         }

         waitForRelease = TRUE;
      }

      if (digitalRead(AF_DOWN) == LOW)
      {
         // scroll down the list

         if (itemIndex < (numberOfItems - 1))
         {
            itemIndex++;
            if (selectedRow < (display_rows - 1))
            {
               // clear the current item marker as it is changing
               lcdPosition(lcdHandle, 0, selectedRow);
               lcdPutchar(lcdHandle, ' ');

               selectedRow++;
               lcdPosition(lcdHandle, 0, selectedRow);
               lcdPutchar(lcdHandle, '*');
            }
            else
            {
               // already at the bottom displayed menu item so need to scroll down the list
               firstItemIndex++;
               itemsToShow = numberOfItems - firstItemIndex;
               {
                  if (itemsToShow > (display_rows - 1))
                  {
                     itemsToShow = display_rows - 1;
                  }
               }
               display_menu_items(itemsToShow, firstItemIndex, menuItems);
            }
         }
         else
         {
            // wrap back to start
            // clear the current marker as it is changing
            lcdPosition(lcdHandle, 0, selectedRow);
            lcdPutchar(lcdHandle, ' ');

            itemIndex = 0;
            firstItemIndex = 0;
            selectedRow = 1;
            if (numberOfItems > (display_rows - 1))
            {
               itemsToShow = display_rows - 1;
            }
            else
            {
               itemsToShow = numberOfItems;
            }
            display_menu_items(itemsToShow, firstItemIndex, menuItems);
            lcdPosition(lcdHandle, 0, selectedRow);
            lcdPutchar(lcdHandle, '*');
         }
         waitForRelease = TRUE;
      }

      if (digitalRead(AF_SELECT) == LOW)
      {
         selected = TRUE;
         waitForRelease = TRUE;
      }
   } // end while

   return itemIndex;
}

void display_menu_items(int numberOfItems, int firstItem, char(*menuItems)[20])
{
   int itemsToShow;
   int index;

   if (numberOfItems > (display_rows - 1))
   {
      itemsToShow = display_rows - 1;
   }
   else
   {
      itemsToShow = numberOfItems;
   }

   for (index = 0; index < itemsToShow; index++)
   {
      lcdPosition(lcdHandle, 1, index + 1);
      lcdPuts(lcdHandle, menuItems[index + firstItem]);
   }
}

void reboot_system(void)
{
   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 1);
   lcdPuts(lcdHandle, "Rebooting...");
   printf("\nReturning control to 386EX CPU.\n");
   printf("Calling Retore_Idle\n");
   Restore_Idle();
   delay(1000); // wait for screens to refresh
   system("sudo reboot now");
}

void shutdown_system(void)
{
   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);
   lcdPuts(lcdHandle, "Shutting down system");
   lcdPosition(lcdHandle, 0, 1);
   lcdPuts(lcdHandle, "Please wait for LED ");
   lcdPosition(lcdHandle, 0, 2);
   lcdPuts(lcdHandle, "to stop flashing");
   lcdPosition(lcdHandle, 0, 3);
   lcdPuts(lcdHandle, "before turning off");
   printf("\nReturning control to 386EX CPU.\n");
   printf("Calling Retore_Idle\n");
   printf("\nPlease wait for green LED to stop flashing before powering off\n");
   Restore_Idle();
   system("sudo shutdown -h now");
}

void showWelcomeScreen()
{
   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0); lcdPuts(lcdHandle, "      Flowbird      ");
   lcdPosition(lcdHandle, 0, 1); lcdPuts(lcdHandle, "    JTAG Adaptor    ");

   // show for a few seconds
   delay(3000);
}

void configureGPIO()
{
   pinMode(JTAG_TCK, OUTPUT);
   pinMode(JTAG_TMS, OUTPUT);
   pinMode(JTAG_TDI, OUTPUT);
   pinMode(JTAG_TDO, INPUT);
   pinMode(JTAG_TRST, OUTPUT);
   pinMode(JTAG_WR, OUTPUT);

   // Clear the TRST# signal before we start
   digitalWrite(JTAG_TRST, HIGH);
   TRST_State = TRST;

   digitalWrite(JTAG_WR, HIGH);
   WR_State = J_WR;

   // DJF not sure about this WR line
   //digitalWrite(JTAG_WR, HIGH);

   // DJF Note: Set TRST low (active state) and then wait for key press
   // This appears to properly halt the machine
   digitalWrite(JTAG_TRST, LOW);
   TRST_State = 0;
}

void topLevelMenu()
{
   int numberOfItems = 0;
   bool running = true;

   while (running)
   {
      numberOfItems = 0;
      sprintf(menuItems[numberOfItems++], "%-19s", "JTAG Functions");
      sprintf(menuItems[numberOfItems++], "%-19s", "Settings");
      sprintf(menuItems[numberOfItems++], "%-19s", "Reboot");
      sprintf(menuItems[numberOfItems++], "%-19s", "Shutdown");

      if (command_line_opt != 0)
      {
         sprintf(menuItems[numberOfItems++], "%-19s", "Exit to CMD line");
      }

      switch (select_menu_item(numberOfItems, menuItems, "Main Menu"))
      {
      case 0:
         JTAGMenu();
         break;
      case 1:
         settingsMenu();
         break;
      case 2:
         reboot_system();
         break;
      case 3:
         shutdown_system();
         break;
      case 4:
         running = false;
         break;
      default:
         // just go around again - shouldn't happen
         break;
      } // end of switch
   } // end while running loop
}

void settingsMenu()
{
   int numberOfItems = 0;
   bool running = true;

   while (running)
   {
      numberOfItems = 0;
      sprintf(menuItems[numberOfItems++], "%-19s", "Display IP Address");
      sprintf(menuItems[numberOfItems++], "%-19s", "Display Hostname");
      sprintf(menuItems[numberOfItems++], "%-19s", "Check Date & Time");
      sprintf(menuItems[numberOfItems++], "%-19s", "Check Clock Period");
      sprintf(menuItems[numberOfItems++], "%-19s", "Configuration");
      sprintf(menuItems[numberOfItems++], "%-19s", "[return]");
      switch (select_menu_item(numberOfItems, menuItems, "Settings"))
      {
      case 0:
         // show the current IP settings
         showIPDetails();
         break;
      case 1:
         // show the current IP settings
         showHostname();
         break;
      case 2:
         // show the current date and time
         checkDateTime();
         break;
      case 3:
         // show the current JTAG Pulse Width
         checkClockPulseWidth(cycles_to_wait);
         break;
      case 4:
         // shpw the current date and time
         configurationMenu();
         break;
      case 5:
         // return to previous menu
         running = false;
         break;
      default:
         break;
      } // end of switch
   } // end of while running loop
}

void checkClockPulseWidth(int cycles_to_wait)
{
   // 1us is max_cpu_freq / 1000
   int cycles_us = cpu_max_freq / 1000; // how many clock cycles per microsecond
   int microSeconds =  cycles_to_wait / cycles_us;
   int nanoSeconds = cycles_to_wait % cycles_us;

   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);
   lcdPrintf(lcdHandle, "JTAG Pulse Width:");
   lcdPosition(lcdHandle, 2, 1);
   lcdPrintf(lcdHandle, "%d.%dus", microSeconds, nanoSeconds);
   lcdPosition(lcdHandle, 2, 2);
   switch(wait_mode)
   {
      case NOP_LOOP:
      {
         lcdPrintf(lcdHandle, "Mode: \"nop\" loop");
         break;
      }
      case CCR_LOOP:
      default:
      {
         lcdPrintf(lcdHandle, "Mode: CCR loop");
         break;
      }
   }
   lcdPosition(lcdHandle, 0, display_rows - 1);
   lcdPuts(lcdHandle, " ...Press SELECT...");
   waitForEnter();
}


void checkDateTime()
{
   time_t time_now;
   char time_string[20 + 1];
   char date_string[20 + 1];
   struct tm *ptm;

   lcdClear(lcdHandle);
   time_now = time(NULL);
   ptm = localtime(&time_now);
   sprintf(date_string, "Date: %04d-%02d-%02d",
      (ptm->tm_year) + 1900,
      (ptm->tm_mon) + 1,
      ptm->tm_mday);
   sprintf(time_string, "Time: %02d:%02d:%02d",
      ptm->tm_hour,
      ptm->tm_min,
      ptm->tm_sec);
   lcdPosition(lcdHandle, 0, 0);
   lcdPuts(lcdHandle, date_string);
   lcdPosition(lcdHandle, 0, 1);
   lcdPuts(lcdHandle, time_string);
   lcdPosition(lcdHandle, 0, display_rows - 1);
   lcdPuts(lcdHandle, " ...Press SELECT...");
   waitForEnter();
}

void showIPDetails()
{
   int fd;
   struct ifreq ifr;

   // Get Ethernet IP address
   fd = socket(AF_INET, SOCK_DGRAM, 0);
   ifr.ifr_addr.sa_family = AF_INET;
   strncpy(ifr.ifr_name, "eth0", IFNAMSIZ - 1);
   ioctl(fd, SIOCGIFADDR, &ifr);
   close(fd);
   printf("\neth0 IP address is: %s\n", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);
   lcdPuts(lcdHandle, "EIP=eth0: WIP=wlan0:");
   lcdPosition(lcdHandle, 0, 1);
   lcdPrintf(lcdHandle, "EIP: %s", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
   // Get Wirless IP address
   fd = socket(AF_INET, SOCK_DGRAM, 0);
   ifr.ifr_addr.sa_family = AF_INET;
   strncpy(ifr.ifr_name, "wlan0", IFNAMSIZ - 1);
   ioctl(fd, SIOCGIFADDR, &ifr);
   close(fd);
   printf("wlan0 IP address is: %s\n", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
   lcdPosition(lcdHandle, 0, 2);
   lcdPrintf(lcdHandle, "WIP: %s", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
   lcdPosition(lcdHandle, 0, display_rows - 1);
   lcdPuts(lcdHandle, " ...Press SELECT...");
   waitForEnter();
}

void showHostname()
{
   char hostname[display_cols];

   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);
   lcdPuts(lcdHandle, "Hostname:");
   lcdPosition(lcdHandle, 0, 1);
   if (gethostname(hostname, display_cols) == 0)
   {
      lcdPrintf(lcdHandle, "%s", hostname);
      lcdPosition(lcdHandle, 0, 2);
      lcdPrintf(lcdHandle, "%s.local", hostname);
   }
   else
   {
      lcdPuts(lcdHandle, "[unknown!]");
   }
   lcdPosition(lcdHandle, 0, display_rows - 1);
   lcdPuts(lcdHandle, " ...Press SELECT...");
   waitForEnter();
}

void configurationMenu()
{
   int numberOfItems = 0;
   bool running = true;

   while (running)
   {
      numberOfItems = 0;
      sprintf(menuItems[numberOfItems++], "%-19s", "Export settings");
      sprintf(menuItems[numberOfItems++], "%-19s", "Import settings");
      sprintf(menuItems[numberOfItems++], "%-19s", "[return]");
      switch (select_menu_item(numberOfItems, menuItems, "Settings"))
      {
      case 0:
         // copy the live jtag.ini file to US drive
         exportSettings();
         break;
      case 1:
         // update live jtag.in with file on USB drive
         importSettings();
         break;
      case 2:
      default:
         // just return to previous menu
         running = false;
         break;
      } // end of switch
   } // end of while running loop
}

void exportSettings()
{
   File_Copy("/home/pi/jtag/jtag.ini", "/home/pi/jtagdisk/jtag.ini");
}

void importSettings()
{
   if (File_Copy("/home/pi/jtagdisk/jtag.ini", "/home/pi/jtag/jtag.ini") == 0)
   {
      // ask user whether to reboot to use new settings
      askRebootNow();
   }
}

int File_Copy(char FileSource[], char FileDestination[])
{
   /*
    *  Function return value meanings
    * -1 cannot open source file
    * -2 cannot open destination file
    * 0 Success
    */
   int   c;
   FILE *stream_R;
   FILE *stream_W;

   stream_R = fopen(FileSource, "r");
   if (stream_R == NULL)
   {
      return -1;
   }

   stream_W = fopen(FileDestination, "w");   //create and write to file
   if (stream_W == NULL)
   {
      fclose(stream_R);
      return -2;
   }
   while ((c = fgetc(stream_R)) != EOF)
   {
      fputc(c, stream_W);
   }
   fclose(stream_R);
   fclose(stream_W);

   return 0;
}

void askRebootNow()
{
   int numberOfItems = 0;
   bool running = true;

   while (running)
   {
      numberOfItems = 0;
      sprintf(menuItems[numberOfItems++], "%-19s", "now");
      sprintf(menuItems[numberOfItems++], "%-19s", "later");
      switch (select_menu_item(numberOfItems, menuItems, "Reboot Required:"))
      {
      case 0:
         // reboot now
         reboot_system();
         break;
      case 1:
      default:
         // just return to previous menu
         running = false;
         break;
      } // end of switch
   } // end of while running loop
}

void JTAGMenu()
{
   int thisDeviceType;
   int numberOfItems = 0;
   unsigned long int totalMemory = 0x0FFFFF;
   bool running = true;

   // connect to the device
   printf("Power TGX device now and press select on JTAG device\n");

   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0); lcdPuts(lcdHandle, "Power TGX or SCV and");
   lcdPosition(lcdHandle, 0, 1); lcdPuts(lcdHandle, "    press SELECT    ");

   waitForEnter();

   configureGPIO();

   // DJF Note: Now that the machine is halted clear the TRST# signal
   digitalWrite(JTAG_TRST, HIGH);
   TRST_State = TRST;

   digitalWrite(JTAG_WR, HIGH);
   WR_State = J_WR;

   Fill_JTAG(PinState);       // initialisation string

   Reset_JTAG();              // toggle the TRST# signal low
   Restore_Idle();            // Reset JTAG state machine

   //Get_JTAG_Device_ID();    // Show ID of 386EX and abort if wrong
   Clear_Strobes(PinState);

   /*
    * As get board revision is what actually halts the CPU probably just need to
    * call Send_Instruction_IN for SAMPLE and EXTEST
    *
    Get_Board_Revision(thisDeviceType);
    */
   Send_Instruction_IN(strlen(SAMPLE), SAMPLE); // SAMPLE/preload to initialise BSR
   Send_Instruction_IN(strlen(EXTEST), EXTEST); // Configure for external test

   // DJF To Do  - what if incorrect?

   thisDeviceType = selectDeviceType();

   if (debug_level > 0)
   {
      printf("\ndevice type has been set to %d\n", thisDeviceType);
   }
   FLASH_START = FLASH_START_1M[thisDeviceType];
   FLASH_TOP = FLASH_START + 0x100000 - 1;

   totalMemory = selectMaximumMemory(thisDeviceType);

   if (debug_level > 0)
   {
      printf("total memory has been set to 0x%08lX\n", totalMemory);

      printf("FLASH_START is now set to 0x%08lX\n", FLASH_START);
   }

   lcdClear(lcdHandle);

   if (thisDeviceType == ETM_TGX)
   {
      Get_M5_Device_ID();
   }
   Get_JTAG_Device_ID();   // Show ID of 386EX and abort if wrong

   Get_Board_Revision(thisDeviceType);

   // make sure the #WR is clear
   digitalWrite(JTAG_WR, HIGH);
   WR_State = J_WR;

#ifndef DJF_ANALYSER
   delay(5000); // show info for 5 seconds
#endif

   lcdClear(lcdHandle);

   while (running)
   {
      numberOfItems = 0;
      sprintf(menuItems[numberOfItems++], "%-19s", "Test for signature");
      sprintf(menuItems[numberOfItems++], "%-19s", "Dump to file");
      sprintf(menuItems[numberOfItems++], "%-19s", "Load memory");
      if (diagnostic_menu != 0)
      {
         sprintf(menuItems[numberOfItems++], "%-19s", "Diagnostics");
      }
      sprintf(menuItems[numberOfItems++], "%-19s", "[return]");
      switch (select_menu_item(numberOfItems, menuItems, "Select Action:"))
      {
      case 0:
         checkSignature(thisDeviceType);
         break;
      case 1:
         dumpMemory(thisDeviceType, totalMemory);
         break;
      case 2:
         loadMemory();
         break;
      case 3:
         if (diagnostic_menu != 0)
         {
            diagnosticsMenu(thisDeviceType, totalMemory);
         }
         else
         {
            // release the device and return to previous menu
            printf("\nReturning control to 386EX CPU.\n");
            printf("Calling Restore_Idle()\n");
            //Restore_Idle();       // Let go of the processor...
            lcdClear(lcdHandle);
            lcdPosition(lcdHandle, 0, 0);
            lcdPuts(lcdHandle, "Calling Restore");
            lcdPosition(lcdHandle, 0, 1);
            lcdPuts(lcdHandle, "Idle...");
            Restore_Idle();
            delay(2000);            // show message briefly
            lcdClear(lcdHandle);

            running = false;
         }
         break;
      case 4:
         // release the device and return to previous menu
         printf("\nReturning control to 386EX CPU.\n");
         printf("Calling Restore_Idle()\n");
         //Restore_Idle();          // Let go of the processor...
         lcdClear(lcdHandle);
         lcdPosition(lcdHandle, 0, 0);
         lcdPuts(lcdHandle, "Calling Restore");
         lcdPosition(lcdHandle, 0, 1);
         lcdPuts(lcdHandle, "Idle...");
         Restore_Idle();
         delay(2000);               // show message briefly
         lcdClear(lcdHandle);

         running = false;
         break;
      default:
         break;
      } // end of switch
   } // end of while running loop
}

void notYetImplemented()
{
   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);
   lcdPuts(lcdHandle, "Not Yet");
   lcdPosition(lcdHandle, 0, 1);
   lcdPuts(lcdHandle, "Implemented...");
   lcdPosition(lcdHandle, 0, 3);
   lcdPuts(lcdHandle, " ...Press Select...");

   waitForEnter();
}

void diagnosticsMenu(int thisDeviceType, int totalMemory)
{
   unsigned int check_data = 0;
   unsigned int testValues[4] = { 0x0000, 0xFFFF, 0xAAAA, 0x5555 };
   unsigned int testAddress, scopeTestAddress = 0;
   unsigned int startAddress, endAddress = 0;
   int index = 0;
   bool bIgnore = FALSE;
   bool bManageError = TRUE;
   bool bContinue = TRUE;
   bool bQuitDiagnostics = FALSE;

   //notYetImplemented();

   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);

   Fill_JTAG(PinState);       // Initialisation string
   //Reset_JTAG();
   //Restore_Idle();          // Reset JTAG state machine

   Reset_JTAG();
   Get_JTAG_Device_ID();      // Show ID of 386EX
   //delay(1000);

   //Reset_JTAG();
   Get_Board_Revision(thisDeviceType); // Read TGX/tra revision code
   // which may affect the FLASH address etc.
   delay(1000);

   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);
   lcdPuts(lcdHandle, "Test RAM addr 0");
   delay(1000);

   testAddress = 0;

   /**************************************************************************/
   printf("\n\nTesting RAM address %X..................\n\n", testAddress);

   for (index = 0; index < 4; index++)
   {
      bManageError = TRUE;
      bIgnore = FALSE;
      printf("Writing 0x%04X to address 0x%08X.....", testValues[index], testAddress);

      lcdClear(lcdHandle);
      lcdPosition(lcdHandle, 0, 0);
      lcdPrintf(lcdHandle, "Wr: 0x%04X to Addr %X", testValues[index], testAddress);
      /*
       * 2021-08-03 DJF Note: this was passing address and data in the wrong order
       */
      RAM_Write(PinState, testAddress, testValues[index]);

      printf("reading.......");
      //check_data = RAM_Read(PinState, 0);
      check_data = Memory_Read(PinState, testAddress, thisDeviceType);
      lcdPosition(lcdHandle, 0, 1);
      lcdPrintf(lcdHandle, "Rd: 0x%04X fr Addr %X", check_data, testAddress);

      if ((check_data != (unsigned int)testValues[index]) && (bIgnore == FALSE))
      {
         printf("\n***ERROR!*** ");
         printf("address 0x%08X: Wrote 0x%04X, read 0x%04X.\n\n", testAddress, testValues[index], check_data);
         printf("if the previous tests were OK, there is probably a short or open on the \naddress bus.\n");
         printf("\nSelect 'scope loop, ignore error, exit test or exist diagnostics\n");
         lcdPosition(lcdHandle, 0, 2);
         lcdPuts(lcdHandle, " ***ERROR!*** ");
         promptForEnter();

         //waitForEnter();
         while (bManageError && !bIgnore)
         {
            switch (askHowToProgressError())
            {
            case 0:
               // run the scope loop
               bIgnore = FALSE;
               bContinue = TRUE;

               lcdClear(lcdHandle);
               lcdPosition(lcdHandle, 0, 0);
               lcdPuts(lcdHandle, "'scope loop active");
               lcdPosition(lcdHandle, 0, 1);
               lcdPrintf(lcdHandle, "Wr: 0x%04X to Addr %X", testValues[index], testAddress);
               lcdPosition(lcdHandle, 0, display_rows - 1);
               lcdPuts(lcdHandle, "Press Select to end");
               paddleCharIndex = 0;

               while (bContinue)
               {
                  RAM_Write(PinState, testAddress, testValues[index]);
                  check_data = Memory_Read(PinState, testAddress, thisDeviceType);

                  lcdPosition(lcdHandle, 0, display_rows - 2);
                  lcdPutchar(lcdHandle, getNextPaddleChar());
                  delay(50); // just to stop the paddle character blur on display

                  // See if user is ready to stop this loop
                  if (digitalRead(AF_SELECT) == LOW)  // check for push
                  {
                     bContinue = FALSE;
                  }
               } // end of while continue 'scope loop
               break;
            case 1:
               // ignore this error and other errors in this test phase
               bIgnore = TRUE;
               break;
            case 2:
               // quit this test
               //index = 4;
               bManageError = FALSE;
               break;
            case 3:
            default:
               // Quit diagnostic tests
               bManageError = FALSE;
               bQuitDiagnostics = TRUE;
               index = 4; // jump outof this cycle
               break;
            } // end of switch
         } // end while bContinue
      }
      else
      {
         printf("Okay\n");
         lcdPosition(lcdHandle, 0, 2);
         lcdPuts(lcdHandle, " read okay ");
         delay(500);
      }
   }

   delay(500);

   if (bQuitDiagnostics)
   {
      return;
   }

   /**************************************************************************/

   printf("\nTesting first RAM block...................\n");

   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);
   lcdPuts(lcdHandle, "Test RAM addr 0..FF");
   delay(1000);

   startAddress = 0;
   endAddress = 0xFF;

   for (index = 0; index < 4; index++)
   {
      bManageError = TRUE;
      bIgnore = FALSE;
      lcdClear(lcdHandle);

      printf("Writing 0x%04X to address 0x%08X to 0x%08X\n", testValues[index], startAddress, endAddress);

      for (testAddress = startAddress; testAddress < endAddress; testAddress += 2)
      {
         // write to all addresses in block
         lcdPosition(lcdHandle, 0, 0);
         lcdPrintf(lcdHandle, "Wr:0x%04X Addr %X", testValues[index], testAddress);

         RAM_Write(PinState, testAddress, testValues[index]);
      }

      for (testAddress = startAddress; testAddress < endAddress; testAddress += 2)
      {
         // read back from all addresses
         check_data = Memory_Read(PinState, testAddress, thisDeviceType);
         lcdPosition(lcdHandle, 0, 1);
         lcdPrintf(lcdHandle, "Rd:0x%04X Addr %X", check_data, testAddress);
         if ((check_data != (unsigned int)testValues[index]) && (bIgnore = FALSE))
         {
            printf("\n***ERROR at  ");
            printf("address 0x%08X: Wrote 0x%04X, read 0x%04X.\n\n", testAddress, testValues[index], check_data);
            printf("If the previous tests were OK, there is probably a short or open on the \naddress bus.\n");
            printf("\nSelect 'scope loop, ignore error, exit test or exit diagnostics\n");
            lcdPosition(lcdHandle, 0, 2);
            lcdPuts(lcdHandle, " ***ERROR!*** ");
            promptForEnter();
            while (bManageError && !bIgnore)
            {
               switch (askHowToProgressError())
               {
                  case 0:
                     // run the 'scope loop
                     bIgnore = FALSE;
                     bContinue = TRUE;

                     lcdClear(lcdHandle);
                     lcdPosition(lcdHandle, 0, 0);
                     lcdPuts(lcdHandle, "'scope loop active");
                     lcdPosition(lcdHandle, 0, 1);
                     lcdPrintf(lcdHandle, "Wr: 0x%04X to Addr %x", testValues[index], testAddress);
                     lcdPosition(lcdHandle, 0, display_rows - 1);
                     lcdPuts(lcdHandle, "Press Select to end");
                     paddleCharIndex = 0;
                     lcdPosition(lcdHandle, 0, display_rows - 2);
                     printf("\n\n-----Starting 'scope loop. Addresses 0x%08X to 0x%08X are set to 0x%04X, then ",
                            startAddress, endAddress, testValues[index]);
                     printf("-----read back. Check that all the data lines are 0x%04X and the lower 8\n", testValues[index]);
                     printf("-----address lines are activ.\n\nPress Select to leave this loop\n");
                     while (bContinue)
                     {
                        for(scopeTestAddress = startAddress; scopeTestAddress < endAddress; scopeTestAddress++)
                        {
                           RAM_Write(PinState, scopeTestAddress, testValues[index]);
                           check_data = Memory_Read(PinState, scopeTestAddress, thisDeviceType);
                        }
                        lcdPosition(lcdHandle, 0, display_rows - 2);
                        lcdPutchar(lcdHandle, getNextPaddleChar());
                        delay(50); // just to stop the paddle character blur on the display

                        // see if user is ready to stop this loop
                        if(digitalRead(AF_SELECT) == LOW) // check for push
                        {
                           bContinue = FALSE;
                        }
                     } // end while continue 'scope loop
                     break;
                  case 1:
                     // ignore this error and other errors in this test phase
                     bIgnore = TRUE;
                     break;
                  case 2:
                     // quit this test
                     bManageError = FALSE;
                     break;
                  case 3:
                  default:
                     // Quite diagnostic tests
                     bManageError = FALSE;
                     bQuitDiagnostics = TRUE;
                     index = 4; // jump out of this cycle
                     break;

               } // end of switch
            } // end of while manage error
            // stop testing now
            testAddress = 0xFF;
            index = 4;
         }
         else
         {
            printf("Okay\r");
            lcdPosition(lcdHandle, 0, 2);
            lcdPuts(lcdHandle, " read okay ");
            delay(500);
         }
      }
   }
   delay(500);

   if (bQuitDiagnostics)
   {
      return;
   }

   /**************************************************************************/

   // Extended RAM tests

   printf("\nRun extended RAM tests? ");
   lcdClear(lcdHandle);
   if(selectBoolean("Run ext.d RAM tests?"))
   {
      notYetImplemented();
//DJF To do

      delay(500);

      if (bQuitDiagnostics)
      {
         return;
      }
   }

   promptForEnter();
}

int askHowToProgressError()
{
   int numberOfItems = 0;
   sprintf(menuItems[numberOfItems++], "%-19s", "Run 'scope loop");
   sprintf(menuItems[numberOfItems++], "%-19s", "Ignore Error");
   sprintf(menuItems[numberOfItems++], "%-19s", "Exit this test");
   sprintf(menuItems[numberOfItems++], "%-19s", "Exit all tests");

   return select_menu_item(numberOfItems, menuItems, "ERROR:Continue?...");
}

void checkSignature(int thisDeviceType)
{
   struct signatureBlockType theSignatureBlock;

   theSignatureBlock = testForSignature(0x40004, thisDeviceType);
   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);
   lcdPrintf(lcdHandle, "Signature: %04X%04X", theSignatureBlock.theSignature[0], theSignatureBlock.theSignature[1]);
   lcdPosition(lcdHandle, 0, 1);
   if (!theSignatureBlock.signatureOk)
   {
      lcdPuts(lcdHandle, "...is incorrect!");
      printf("WARNING: Signature is incorrect\n");
   }
   else
   {
      lcdPuts(lcdHandle, "...is correct :-)");
   }
   printf("Signature bytes read 0x%04X%04X\n", theSignatureBlock.theSignature[0], theSignatureBlock.theSignature[1]);
   // display signature for a couple of seconds (unless logic using logic analyser to test)

#ifndef DJF_ANALYSER
   delay(2000);
#endif
}

void dumpMemory(int thisDeviceType, int totalMemory)
{
   time_t start_time, end_time;
   char * c_time_string;
   struct tm *ptm;

   char filename[512];        // will be <filepath>/<YYYYMMDD_HHMMSS_>D<evice Type>_jtagdump.bin
   //   char filepath[256]; needs to be global
   struct addressRangeType stAddressRange;

   stAddressRange = selectAddressRange(totalMemory);

   // return if start and end adddess are the same (cancel dump)
   //
   if(stAddressRange.startAddress != stAddressRange.endAddress)
   {
      start_time = time(NULL);
      c_time_string = ctime(&start_time);

      if (debug_level > 0)
      {
         printf("dump timing at %d cycles to wait\n", cycles_to_wait);
         printf("dump starting at %s\n", c_time_string);
      }

      ptm = localtime(&start_time);

      switch (thisDeviceType)
      {
      case ETM_TGXtra:
      case ETM_TGX:

         if (debug_level > 0)
         {
            printf("ETM Range selected is 0x%08lX to 0x%08lX\n", stAddressRange.startAddress, stAddressRange.endAddress);
         }

         sprintf(filename, "%s/%04d%02d%02d_%02d%02d%02d_ETM_jtagdump.bin",
            filepath,
            ptm->tm_year + 1900,
            ptm->tm_mon + 1,
            ptm->tm_mday,
            ptm->tm_hour,
            ptm->tm_min,
            ptm->tm_sec);

         break;
      case SCV:
      default:
         if (debug_level > 0)
         {
            printf("SCV Range selected is 0x%08lX to 0x%08lX\n", stAddressRange.startAddress, stAddressRange.endAddress);
         }
         sprintf(filename, "%s/%04d%02d%02d_%02d%02d%02d_SCV_jtagdump.bin",
            filepath,
            ptm->tm_year + 1900,
            ptm->tm_mon + 1,
            ptm->tm_mday,
            ptm->tm_hour,
            ptm->tm_min,
            ptm->tm_sec);

         break;
      } // switch deviceType

      dump_to_file(stAddressRange.startAddress, stAddressRange.endAddress, thisDeviceType, filename); // user selected range

      end_time = time(NULL);

      show_dump_stats(start_time, end_time);

      retest_signature(thisDeviceType);
   }
}

void loadMemory()
{
   char filename[512];
   bool bErase = FALSE;
   bool bVerify = FALSE;
   bool bCancelLoad = FALSE;
   unsigned long int bytesProgrammed = 0; // need to know for verify read back

   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 1);
   sprintf(filename, "/home/pi/jtagdisk/Image Files");
   if (selectFilename(filename))
   {
      if(debug_level > 0)
      {
         printf("\nDJF DEBUG: Filename is \"%s\"\n", filename);
      }
      // TGXtra Booter offset 0x03FFA000
      // SCV Booter Offset 0x03FF8000
      // Custom Offset
      sprintf(menuItems[0], "%-19s", "TGXtra 0x03FFA000");
      sprintf(menuItems[1], "%-19s", "SCV    0x03FF8000");
      sprintf(menuItems[2], "%-19s", "Custom Offset");
      sprintf(menuItems[3], "%-19s", "[return]");
      switch (select_menu_item(4, menuItems, "Start Address Type?"))
      {
      case 0: // TGXtra
         data_start_address = 0x03FFA000;
         break;
      case 1: // SCV
         data_start_address = 0x03FF8000;
         break;
      case 2:
      default: // Custom
         data_start_address = (unsigned long int)selectHexAddress("Start Offset(HEX):", FLASH_START);
         break;
      case 3:
         bCancelLoad = TRUE;
         break;
      }

      if(!bCancelLoad)
      {
         bErase = selectBoolean("Erase Flash?");
         bVerify = selectBoolean("Verfify Data?");
         if (bErase)
         {
            lcdClear(lcdHandle);
            lcdPosition(lcdHandle, 0, 0);
            lcdPuts(lcdHandle, "Erasing Flash");
            lcdPosition(lcdHandle, 0, 1);
            lcdPuts(lcdHandle, "(about 15 secs)....");
            Erase_Flash();
            lcdClear(lcdHandle);
            lcdPosition(lcdHandle, 0, 0);
            lcdPuts(lcdHandle, "...Erase Done");
         }
         bytesProgrammed = Program_Flash_Data(filename, data_start_address, lcdHandle, display_rows, display_cols);

         // DJF To Do - check verify handling
         if (bVerify)
         {
            // read back the data to file - show progress

            // compare verification file with original - show progress

            // report results
         }
      } // end of if not cancel load

   }
   lcdClear(lcdHandle);
   lcdPosition(lcdHandle, 0, 0);
   lcdPuts(lcdHandle, " ...Press Select...");
   waitForEnter();
}
