#include <stdio.h>
#include <wiringPi.h>
#include <time.h>

#include <stdbool.h>

#include <string.h>

#include <lcd.h>

#include <jtag.h>
#include <jtag_bsr_pins.h>
#include <jtag_types.h>
#include <jtag_funcs.h>

const int delay_us = 1;
const int delay_ns = 50;

//extern int cycles_to_wait;
int cycles_to_wait;

//extern word ID_String_Length;
word ID_String_Length;

//extern unsigned long int flash_magic_1, flash_magic_2;
unsigned long int flash_magic_1, flash_magic_2;
//extern char PinState[BSR_Length];
char PinState[BSR_Length];

//extern int Flash_Device_ID;
int Flash_Device_ID;

void Fill_JTAG(Pchar P)
{
   /* Configures  pins for typical configuration
	ADS: Out, low
	BHE: Out, low
	BLE: Out, low
	WR:  Out, don't care
	RD:  Out, don't care
	WRD: Out, low
	DC:  Out, high
	MIO: Out, high
	UCS: Out, high
	CS0: Out, high
	CS1: Out, high
	CS2: Out, high
	CS3: Out, high
	CS4: Out, high
	CS6: Out, high
	LBA: Out, low
	All others configured as inputs

	Dir Bit Output 	= i*2 ( 0 -> input, 1 -> output)
	Data Bit	= i*2 +1
   */
   unsigned index;

//   for(index = 0; index <= BSR_Length; index++)
   for(index = 0; index <= (BSR_Length - 1); index++)
   {
      P[index] = '0';	// set all to inputs
   }

   P[P11 * 2]		= '1';		// Sol enable off
   P[P11 * 2 + 1]	= '0';
   //P[P22 * 2 + 1]	= '0'; 
   P[P32 * 2]		= '1';		// Sleep output
   P[P32 * 2 + 1]	= '1';
   P[P33 * 2]		= '1';		// Printer Power output
   P[P33 * 2 + 1]	= '0';
   P[P35 * 2]		= '1';		// Power on output
   P[P35 * 2 + 1]	= '1';
   P[ADS * 2]		= '1';		// then modify some important pins
   P[ADS * 2 + 1]	= '0';
   P[FLT * 2]		= '1';
   P[FLT * 2 + 1]	= '1';		// don't float the outputs
   P[BHE * 2]		= '1';
   P[BHE * 2 + 1]	= '0';		// BHE and BLE acttive for 16 bit data bus
   P[BLE * 2]		= '1';
   P[BLE * 2 + 1]	= '0';
   P[WR * 2]		= '1';
   P[RD * 2]		= '1';
   P[WRD * 2]		= '1';
   P[WRD * 2 + 1]	= '0';		// Read by default
   P[DC * 2]		= '1';
   P[DC * 2 + 1]	= '1';
   P[MIO * 2]		= '1';
   P[MIO * 2 + 1]	= '1';
   P[WDTOUT * 2]	= '1';
   P[WDTOUT * 2 + 1]	= '0';
   P[UCS * 2]		= '1';		// Also WAY6 write protect
   P[UCS * 2 + 1]	= '1';
   P[CS0 * 2]		= '1';		// WAY386/SCR RAM chip select
   P[CS0 * 2 + 1]	= '1';
   P[CS1 * 2]		= '1';		// WAY386 Expansion RAM/SSRC Expansion
   P[CS1 * 2 + 1]	= '1';
   P[CS2 * 2]		= '1';		// WAY386/SCR Flash chip select
   P[CS2 * 2 + 1]	= '1';
   P[CS3 * 2]		= '1';		// WAY386/SRC RAM chip select. SSSCR I/O 31x
   P[CS3 * 2 +1]	= '1';
   P[CS4 * 2 ]		= '1';		// WAY386/SRC I/O chip select. SRC I/O expansion
   P[CS4 * 2 + 1]	= '1';
   P[jCS6 * 2]		= '1';		// WAY386 FLASH chip select (SMM)
   P[jCS6 * 2 + 1]	= '1';
   P[LBA * 2]		= '1';
   P[LBA * 2 + 1]	= '0';
   P[READY * 2]		= '1';		// Ready signal
   P[READY * 2 + 1]	= '1';
   P[SMIACT * 2]	= '1';		// SMI active
   P[SMIACT * 2 + 1]	= '1';		// (Not active)

}

// global variables

void Reset_JTAG(void)
{
   // Reset TAP logic by optional TRST# signal
   // This should reset both 386EX and M5-192/120
   unsigned out;

   /* linux code - not sure it works?
   out = 0x00;
   digitalWriteByte(out); // Set all signals low
   wait_cycles(cycles_to_wait);
   // Set TRST high first
   out = TRST;
   digitalWriteByte(out);
   wait_cycles(cycles_to_wait);
//printf("Debug: TRST set high\n");
   // set TRST low
   out = 0x00;
   TRST_State = 0;
//printf("Debug: TRST set low\n");

   digitalWriteByte(out);
   wait_cycles(cycles_to_wait);
   out = TRST;
   digitalWriteByte(out);
   TRST_State = TRST;
//printf("Debug: TRST set high\n");
   */

   // Flowbird code
   out = 0x00;
   //out |= JTAG_WR; // set write high
   TRST_State = 0;
   out |= TRST;
   out |= J_WR;
   digitalWriteByte(out); 	// TRST# set high
   TRST_State = TRST;
   wait_cycles(cycles_to_wait);
   out &= notTRST;		// TRST# set low (Asserted - this resets TAP state)
   out |= J_WR;
   digitalWriteByte(out);
   TRST_State = 0;
   out |= TRST;
   out |= J_WR;
   digitalWriteByte(out);	// TRST set high (cleared)
   TRST_State = TRST;
   WR_State = J_WR;
}

void Restore_Idle(void)
{
   // restore Test_Reset_Logic state by 5 TCK's
   // Goes into TLR state from any unknown state
   // (both JTAG controllers)

   int index;
   unsigned out;

//printf("DEBUG: Restore_Idle()\n");

   for (index = 0; index < 5; index++)
   {
      out = TMS | TRST_State | WR_State;
      digitalWriteByte(out);
      wait_cycles(cycles_to_wait);
      out = TCKTMS | TRST_State | WR_State;
      digitalWriteByte(out);
      wait_cycles(cycles_to_wait);
      out = TMS | TRST_State | WR_State;
      digitalWriteByte(out);
   }
}

/**** Function to send instruction to JTAG ****/

void Send_Instruction(unsigned length, char *data)
{
   // Send instruction string to JTAG port, replace the original string
   // witht the data that comes out of TDO
   TMS_Low();			// Go to Run_Test_Idle
   TMS_Low();
   TMS_High();			// Go to Select_DR_Scan
   TMS_High();			// go to Select_IR_Scan
   TMS_Low();			// go to Capture_IR
   TMS_Low();			// go to Shift_IR
   Shift_Data_Array(length, data);	// This goes to the 386EX
   TMS_High();			// Update IR, new instruction in effect
   TMS_Low();			// Back to Run_Test_Idle
}

/**** Function to send instruction to JTAG port - DO NOT READ TDO****/

void Send_Instruction_IN(unsigned length, char *data)
{
   // Send instruction string to JTAG port, replace the original string
   // with the data that comes out of TDO
   TMS_Low();			// Go to Run_Test_Idle
   TMS_Low();
   TMS_High();			// Go to Select_DR_Scan
   TMS_High();			// go to Select_IR_Scan
   TMS_Low();			// go to Capture_IR
   TMS_Low();			// go to Shift_IR
   Shift_Data_Array_IN(length, data);	// This goes to the 386EX
   TMS_High();			// Update IR, new instruction in effect
   TMS_Low();			// Back to Run_Test_Idle
}

/**** Funciton to send data string into JTAG port + replace original ****/

void Send_Data(unsigned length, char *data)
{
   TMS_Low();				// Go to Run_Test_Idle
   TMS_Low();
   TMS_High();				// Go to Select_DR_Scan
   TMS_Low();				// Go to Capture_DR
   TMS_Low();				// Go to Shift_DR
   Shift_Data_Array(length, data);
   TMS_High();				// Update ID, new instruction in effect
   TMS_Low();				// Back to Run_Test_Idle
}

/**** Function to send data string into JTAG port w/o replacing original ****/

void Send_Data_IN(unsigned length, char *data)
{
   TMS_Low();				// Go to Run_Test_Idle
   TMS_Low();
   TMS_High();				// Go to Select_DR_Scan
   TMS_Low();				// Go to Capture_DR
   TMS_Low();				// Go to Shift_DR
   Shift_Data_Array_IN(length, data);
   TMS_High();				// Update ID, new instruction in effect
   TMS_Low();				// Back to Run_Test_Idle
}

/**** Function to set data DIR bits on 16 bit data bus ****/
void Get_Data(PJTAGdata P)
{
   // configures data lines as inputs
   int index;

   for(index = D0; index >=  D15; --index)
   {
      P[index * 2] = '0';
   }
}

/**** Function to set data pins to given 16 bit data ****/
// Sets data on pins and makes them O/Ps
void Set_Data(PJTAGdata P, word D)
{
   int index;
   word M;

   M = 1;
   for(index = D0; index >= D15; --index)
   {
      if((D & M) != FALSE)
      {
         P[index * 2 + 1] = '1';
      }
      else
      {
         P[index * 2 + 1] = '0';
      }
      P[index * 2] = '1';	// data pins set to output
      M <<= 1;
   }
}

/**** Function to read data from I/O port ****/
word IO_Read(Pchar P, unsigned long int address, int dev_type)
{
   Get_Data(P);			// Configure Data bus as inputs
   Set_Address(P, address);	// Set addresson bus
   if(address & 1)		// if ODD address
   {
      P[BLE * 2 + 1] = '1';	// Don't set Bus Low Enable
   }
   if((dev_type == ETM_TGXtra) || (dev_type == ETM_TGX))
   {
      P[CS4 * 2 + 1] = '0';	// Set I/O chip select
   }
   else // SCV
   {
      P[CS3 * 2 + 1] = '0';	// Set SCV I/O chip select
   }
   P[MIO * 2 + 1] = '0';	// Set I/O cycle
   P[RD * 2 + 1] = '0';		// RD# low - read data
   P[WR * 2 + 1] = '1';		// WR# high
   P[WRD * 2 + 1] = '0';	// For read

   Send_Data_IN(BSR_Length, P);	// Sets data, address
   Send_Data(BSR_Length, P);	// Latches data into BSR then shifts it to P

   if((dev_type == ETM_TGXtra) || (dev_type == ETM_TGX))
   {
      P[CS4 * 2 + 1] = '1';	// switch off CS4
   }
   else // SCV
   {
      P[CS3 * 2 + 1] = '1';	// switch off CS3
   }
   P[MIO * 2 + 1] = '1';	// Restore memory mode
   P[RD * 2 + 1] = '1';		// undo RD#
   P[BLE * 2 + 1] = '0';	// Set BLE#
   Send_Data_IN(BSR_Length, P);
   return(Parse_Data(P));	// Converts result into binary
}

/**** Function to reverse data in a data string so MSB is first ****/

void Flip_ID_String(int length, char input[ID_String_Length])
{
   // Flips the JTAG Unit ID string since it is
   // read in backwards.

   int index, temp_index;
   char temp[ID_String_Length];

   temp_index = 0;			// initialise temporary place holder
   for(index = length; index >= 1; --index)
   {
      temp[temp_index] = input[index - 1];
      ++temp_index;
   }
   for(index = 0; index <= (length - 1); index++)
   {
      input[index] = temp[index];
   }
}

/**** Function to shift data into JTAG port while reading ****/

void Shift_Data_Array(unsigned length, char *data)
{
   // Shifts data string into JTAG port while reading data
   // from JTAG port back into D, the procedure should be
   // called when the JTAG controller is in the
   // SelectDRScan state.
   int index;
   unsigned in, out;

   if(length != 0)
   {
      for(index = 0; index < (length - 1); index++)
      {
         // send outgoing bit
         out = TDI;
         if(data[index] == '0')
         {
            out = 0x00;
         }
         out = out & notTCKTMS;
         out |= TRST_State;
         out |= WR_State;
         digitalWriteByte(out);
         wait_cycles(cycles_to_wait);

         out = out | TCK | TRST_State | WR_State;
         digitalWriteByte(out);
         wait_cycles(cycles_to_wait);


         // read the incoming bit
         in = digitalRead(JTAG_TDO);
         // need to invert the incoming bit
         // DJF Note: data read in appeared inverted so chaning the logic
         if(in == 0)
         {
            //data[index] = '1';
            data[index] = '0';
         }
         else
         {
            //data[index] = '0';
            data[index] = '1';
         }
      }// end for loop

   } // end if data length

   // DJF Note: not sure why this can't be done in the above loop?
   //
   out = TDI;
   if(data[length - 1] == '0')
   {
      out = 0x00;
   }
   out = out & notTCKTMS;
   out = out | TMS | TRST_State | WR_State;
   digitalWriteByte(out);
   wait_cycles(cycles_to_wait);

   out = out | TCK | TRST_State | WR_State;
   digitalWriteByte(out);
   wait_cycles(cycles_to_wait);

   in = digitalRead(JTAG_TDO);
   // DJF Note: data read in appeared inverted so chaning the logic
   if(in == 0)
   {
      //data[length - 1] = '1';
      data[length - 1] = '0';
   }
   else
   {
      //data[length - 1] = '0';
      data[length - 1] = '1';
   }

   out = TDITMS | TRST_State | WR_State;
   digitalWriteByte(out);
   wait_cycles(cycles_to_wait);
}

/**** Function to shift data into JTAG port without reading ****/

void Shift_Data_Array_IN(unsigned length, char *data)
{
   // Shifts data string into JTAG port without reading data

   int index;
   unsigned out;

   if(length != 0)
   {
      for(index = 0; index < (length - 1); index++)
      {
         // send outgoing bit
         out = TDI;
         if(data[index] == '0')
         {
            out = 0x00;
         }

         out = out & notTCKTMS;
         out |= TRST_State;
         out |= WR_State;
         digitalWriteByte(out);
         wait_cycles(cycles_to_wait);

         out = out | TCK | TRST_State | WR_State;
         digitalWriteByte(out);
         wait_cycles(cycles_to_wait);
      }// end for loop

   } // end if data length

   out = TDI;
   if(data[length  - 1] == '0')
   {
      out = 0x00;
   }
   out = out & notTCKTMS;
   out = out | TMS | TRST_State | WR_State;
   digitalWriteByte(out);
   wait_cycles(cycles_to_wait);

   out = out| TCK | TRST_State | WR_State;
   digitalWriteByte(out);
   wait_cycles(cycles_to_wait);

   out = TDITMS | TRST_State | WR_State;
   digitalWriteByte(out);
   wait_cycles(cycles_to_wait);
}

/**** Function to convert JTAG ID string into long ****/

long Parse_ID(char P[ID_String_Length])
{
   int index;
   long M = 1, D = 0;

   for(index = 31; index >= 0; --index)
   {
      if(P[index] == '1')
      {
         D = D | M;
      }
      M <<= 1;
   }
   return D;
}

/**** Function to convert JTAG output string into word ****/
word Parse_Data(Pchar P)
{
   int index;
   word M = 1, D = 0;
   word returnWord = 0;

   for(index = D0; index >= D15; --index)
   {
      if(P[index * 2 + 1] == '1')
      {
         D = D | M;
      }
      M <<= 1;
   }


   // DJF Note: there is a byte ordering issue on raspberry pi
   //           as it is opposite endian to windows
   //
   //return D;
   returnWord = (D & 0xFF) * 0x100;
   returnWord += (D/0x100) & 0xFF;
   return returnWord;
}

/**** Fucntion to set the address on the address pins ****/
void Set_Address(PJTAGdata P, unsigned long int address)
{
   int index;
   long int M = 1;

   for(index = A1; index >= A25; --index)
   {
      if(((address >> 1) & M) != 0)	// drop off address bit 0
      {
         P[index * 2 + 1] = '1';
      }
      else
      {
         P[index * 2 + 1] = '0';
      }
      M <<= 1;
      P[index * 2] = '1';		// Set pin to output mode
   }
}

void TMS_High(void) // and TCK it through
{
   // Set TMS and TDI high then togle TCK
   /*
   digitalWrite(JTAG_TMS, HIGH);
   digitalWrite(JTAG_TDI, HIGH);
   digitalWrite(JTAG_TCK, HIGH);
   wait_cycles(cycles_to_wait);
   digitalWrite(JTAG_TCK, LOW);
   */
   // one clock transition with TMS high (toggle TCK)
   digitalWriteByte(TMS | TRST_State | WR_State);	// set TMS high, TCK low
   wait_cycles(cycles_to_wait);

   digitalWriteByte(TCKTMS | TRST_State | WR_State);	// set TMS high, TCK high
   wait_cycles(cycles_to_wait);

   digitalWriteByte(TMS | TRST_State | WR_State);	// set TMS high, TCK low
   wait_cycles(cycles_to_wait);

}

void TMS_Low(void) // and TCK it through
{
   // Set TMS low then toggle TCK
   /*
   digitalWrite(JTAG_TMS, LOW);
   digitalWrite(JTAG_TCK, HIGH);
   wait_cycles(cycles_to_wait);
   digitalWrite(JTAG_TCK, LOW);
   */

   // one clock transition with TMS high (toggle TCK)
   digitalWriteByte(0x00 | TRST_State | WR_State);	// set TMS low, TCK low
   wait_cycles(cycles_to_wait);

   digitalWriteByte(TCK | TRST_State | WR_State);	// set TMS low, TCK high
   wait_cycles(cycles_to_wait);

   digitalWriteByte(0x00 | TRST_State | WR_State);	// set TMS low, TCK low
   wait_cycles(cycles_to_wait);

}

void Clear_Strobes(Pchar P)
{
   P[READY * 2 + 1] = '0';		// Set READY#
   Send_Data_IN(BSR_Length, P);
   P[READY * 2 + 1] = '1';		// .. and clear it again
   Send_Data_IN(BSR_Length, P);		// to clear any RAM_WR# signals
}

int Memory_Read(Pchar P, unsigned long int Address, int dev_type)
{
//printf ("\nDevice type is %d, FLASH_START is 0x%08lX ", dev_type, FLASH_START);
   if((dev_type == ETM_TGXtra) || (dev_type == ETM_TGX))
   {
      if(Address < FLASH_START)
      {
//printf("ETM RAM_Read\n");
         return RAM_Read(P, Address);
      }
      else
      {
//printf("ETM Flash_Read\n");
         return Flash_Read(P, Address);
      }
   }
   else // SCV
   {
      if(Address >= FLASH_START)
      {
//printf("SCV Flash_Read\n");
         return Flash_Read(P, Address);
      }
      if(Address < 0x100000)
      {
//printf("SCV RAM_Read\n");
         return RAM_Read(P, Address);
      }
      else // Address > 0xFFFFF
      //if(Address > 0xFFFFF)
      {
//printf("SCV Exp_RAM_Read\n");
         return Exp_RAM_Read(P, Address);
      }
   }
}

int RAM_Read(Pchar P, unsigned long int Address)
{
   Get_Data(P);			// Configure Data bus as inputs
   Set_Address(P, Address);	// Set address on bus
   P[UCS * 2 + 1] = '1';	// Unset upper chip select
   P[CS0 * 2 + 1] = '0';	// Set RAM chip select
   P[RD * 2 + 1]  = '0';	// RD# low - read data
   P[WR * 2 + 1]  = '1';	// WR# high
   P[WRD * 2 + 1] = '0';	// for read

   Send_Data_IN(BSR_Length, P);	// Sets data, address
   Send_Data(BSR_Length, P);	// Latches data into BSR then shifts it to P
   P[CS0 * 2 + 1] = '1';	// Unset chip select
   P[RD * 2 + 1]  = '1';	// Unset RD#
				// Ready for next time
   Send_Data_IN(BSR_Length, P);	// Resets CS, RD

   return (Parse_Data(P));	// Converts result into binary
}

int Exp_RAM_Read(Pchar P, unsigned long int Address)
{
   Get_Data(P);			// Configure Data bus as inputs
   Set_Address(P, Address);	// Set address on bus
   P[UCS * 2 + 1] = '1';	// Unset upper chip select
   P[CS1 * 2 + 1] = '0';	// Set Expansion RAM chip select
   P[RD * 2 + 1]  = '0';	// RD# low - read data
   P[WR * 2 + 1]  = '1';	// WR# high
   P[WRD * 2 + 1] = '0';	// for read

   Send_Data_IN(BSR_Length, P);	// Sets data, address
   Send_Data(BSR_Length, P);	// Latches data into BSR then shifts it to P
   P[CS1 * 2 + 1] = '1';	// Unset chip select
   P[RD * 2 + 1]  = '1';	// Unset RD#
   P[ADS * 2 + 1] = '0';	// and ADS
				// Ready for next time
   Send_Data_IN(BSR_Length, P);	// Resets CS, RD

   return (Parse_Data(P));	// Converts result into binary
}

/**** Function to write data to RAM ****/

void RAM_Write(Pchar P, unsigned long int Address, word D)
{
   Set_Data(P, D);		// output data onto the bus
   Set_Address(P, Address);	// output address
   P[UCS * 2 + 1] = '1';	// Unset UCS

   // clear all chip selects
   P[CS0 * 2 + 1] = '1';        // CS0 is RAM chip select
   P[CS1 * 2 + 1] = '1';        // CS1 is expansion RAM chip select
   P[CS2 * 2 + 1] = '1';	// CS2 is flash select
   // now select CS based on address
   if(Address > 0x7FFFF) // not sure this is true for TGX or SCV?
   {
printf("\nDJF DEBUG: CS1 has been set\n");
      P[CS1 * 2 + 1] = '0';		// CS1 is expansion RAM
   }
   else
   {
printf("\nDJF DEBUG: CS0 has been set\n");
      P[CS0 * 2 + 1] = '0';		// Set RAM chip select
   }
   P[RD * 2 + 1] = '1';		// not read
   P[WR * 2 + 1] = '0';		// Set WR# low
   P[WRD * 2 + 1] = '1';	// for write access
// 2021-07-14 DJF - this didn't work so going back to original
/*
// 2021-07-14 DJF updates from JXBUG?
   Send_Data_IN(BSR_Length, P);

   P[ADS * 2 + 1] = '1';	// Latch in the WR#
   Send_Data_IN(BSR_Length, P);

   P[WR * 2 + 1] = '1';         // set WR# high again
   Send_Data_IN(BSR_Length, P);
   P[WRD * 2 +1] = '0';		// reset WRD
   P[CS0 * 2 + 1] = '1';	// unset CS's
   P[CS1 * 2 + 1] = '1';
   Send_Data_IN(BSR_Length, P);
*/
   P[ADS * 2 + 1] = '0';	// ADS# is used to latch WR# for the RAM
   P[READY * 2 + 1] = '1';	// ..and READY clears it again
   Send_Data_IN(BSR_Length, P);

   P[ADS * 2 + 1] = '1';	// latch the WR#
   Send_Data_IN(BSR_Length, P);

   P[WR * 2 + 1] = '1';		// Set WR# high again
   P[READY * 2 + 1] = '0';	// READY clears latched WR#
   P[WRD * 2 + 1] = '0';	// and reast WRD
   P[CS0 * 2 + 1] = '1';	// unset chip select
   P[CS1 * 2 + 1] = '1';	// unset expansion chip select
}

int Flash_Read(Pchar P, unsigned long int Address)
{
   Get_Data(P);			// Configure Data bus as inputs
   Set_Address(P, Address);	// Set address on bus
   //P[UCS * 2 + 1] = '1';	// Unset upper chip select
   P[CS2 * 2 + 1] = '0';	// Set chip select
   P[RD * 2 + 1]  = '0';	// RD# low - read data
   P[WR * 2 + 1]  = '1';	// WR# high
   P[WRD * 2 + 1] = '0';	// for read

   Send_Data_IN(BSR_Length, P);	// Sets data, address
   Send_Data(BSR_Length, P);	// Latches data into BSR then shifts it to P
   P[CS2 * 2 + 1] = '1';	// Unset chip select
   P[RD * 2 + 1]  = '1';	// Unset RD#
				// Ready for next time
// This isn't in JXBUG Flash_Read
/*
   Send_Data_IN(BSR_Length, P);	// Resets CS, RD
*/

   return (Parse_Data(P));	// Converts result into binary
}

/**** Function to write data to flash ****/

void Flash_Write(Pchar P, unsigned long int Address, word D)
{
   Set_Data (P, D);		// Output data onto bus
   Set_Address(P, Address);	// Output address
   P[UCS * 2 + 1] = '1';	// Unset UCS
   P[CS2 * 2 + 1] = '0';	// CS2 is normal flash select
   P[RD * 2 +1] = '1';		// Not read

   P[WR * 2 + 1] = '0';		// Set WR# low
   P[WRD * 2 + 1] = '1';	// for write access

   Send_Data_IN(BSR_Length, P);
   // Pulse_Write();

   P[WR * 2 + 1] = '1';		// Set WR# high again

   P[WRD * 2 + 1] = '0';	// and reset WRD
   P[CS2 * 2 + 1] = '1';	// Unset flash select
   Send_Data_IN(BSR_Length, P);
}

void Erase_Flash()
{
   int status;
   int flash_id, flash_dev;

   printf("\nErasing flash - (about 15 seconds)......\n");

   flash_magic_1 = FLASH_START + 0xAAAA; // AMD flash address 5555
   flash_magic_2 = FLASH_START + 0x5554; // AMD flash address 2AAA
   //flash_magic_1 = FLASH_START + 0x5555; // AMD flash address 5555
   //flash_magic_2 = FLASH_START + 0x2AAA; // AMD flash address 2AAA

   Flash_Write(PinState, FLASH_START, 0xF0); // make sure it's reset

   // DJF TESTING - get flash ID
   flash_id = Get_Flash_ID();
   flash_dev = Get_Flash_Dev();

//   // get other endian byte
//   flash_id = (flash_id & 0xFF00)/0x100;
//   flash_dev = (flash_dev & 0xFF00)/0x100;
   printf("DJF DEBUG: Flash ID: 0x%0X\n", flash_id);
   printf("DJF DEBUG: Flash Dev: 0x%0X\n", flash_dev);
   Flash_Write(PinState, FLASH_START, 0xF0); // make sure it's reset


   Flash_Write(PinState, flash_magic_1, 0xAA);
   Flash_Write(PinState, flash_magic_2, 0x55);
   Flash_Write(PinState, flash_magic_1, 0x80);
   Flash_Write(PinState, flash_magic_1, 0xAA);
   Flash_Write(PinState, flash_magic_2, 0x55);
   Flash_Write(PinState, flash_magic_1, 0x10);

   // The auto-timed erase sequence should start now.....

   do
   {
      status = (Flash_Read(PinState, flash_magic_1));	// Check "done" status bit

      // DJF To Do - need to check for button press to allow exit if hangs
   } while ((status & 0x80) == 0);

   printf("done\n");
}

/**** Function to read the flash chip manufacturer ID ****/
//
int Get_Flash_ID(void)
{
   int Flash_ID;
   flash_magic_1 = FLASH_START + 0xAAAA; // AMD flash address 5555
   flash_magic_2 = FLASH_START + 0x5554; // AMD flash address 2AAA
   //flash_magic_1 = FLASH_START + 0x5555; // AMD flash address 5555
   //flash_magic_2 = FLASH_START + 0x2AAA; // AMD flash address 2AAA

   // These calculations are because the flash device is WORD addressed
   //
   Flash_Write(PinState, FLASH_START, 0xF0); // make sure it's reset

   Flash_Write(PinState, flash_magic_1, 0xAA);
   Flash_Write(PinState, flash_magic_2, 0x55);
   Flash_Write(PinState, flash_magic_1, 0x90);

   Flash_ID = SwapWordBytes((Flash_Read(PinState, FLASH_START + 0x01)));

   printf("Flash ID is 0x%0X, made by ", Flash_ID);
   switch(Flash_ID)
   {
      case 0x01:
         printf("AMD\n");
         break;
      case 0x04:
         printf("Fujitsu\n");
         break;
      case 0x20:
         printf("ST\n");
         break;
      default:
         printf("Unknown!\n");
         break;
   }
   return Flash_ID;
}

/**** Function to read the flash chip device ID ****/
//
int Get_Flash_Dev(void)
{
   int Flash_dev;
   flash_magic_1 = FLASH_START + 0xAAAA; // AMD flash address 5555
   flash_magic_2 = FLASH_START + 0x5554; // AMD flash address 2AAA
   //flash_magic_1 = FLASH_START + 0x5555; // AMD flash address 5555
   //flash_magic_2 = FLASH_START + 0x2AAA; // AMD flash address 2AAA

   // These calculations are because the flash device is WORD addressed
   //
   Flash_Write(PinState, FLASH_START, 0xF0); // make sure it's reset

   Flash_Write(PinState, flash_magic_1, 0xAA);
   Flash_Write(PinState, flash_magic_2, 0x55);
   Flash_Write(PinState, flash_magic_1, 0x90);
   Flash_dev = SwapWordBytes((Flash_Read(PinState, FLASH_START + 0x02)));
   Flash_Device_ID = Flash_dev;

   printf("Flash device type is 0x%0X ", Flash_dev);
   switch(Flash_dev)
   {
      case 0x2223:
         printf("512K Top Boot, Correct\n");
         break;
      case 0x22AB:
         printf("512K Bottom Boot, INCORRECT\n");
         break;
      case 0x22D6:
         printf("1M Top Boot, Correct\n");
         break;
      case 0x2258:
         printf("1M Bottom Boot, INCORRECT\n");
         break;
      case 0xEC:
         printf("1M Top Boot, Correct\n");
         break;
      case 0x58:
         printf("1M Bottom Boot, INCORRECT\n");
         break;
      default:
         printf("Unknown\n");
         break;
   }
   return Flash_dev;
}

unsigned long int Program_Flash_Data(char * input_file_name, unsigned long int start_address, int lcdHandle, int rows, int columns)
{

  FILE* in;

  unsigned long int A;
  int c; // character read from binary file
  word new_word, high_part;

  int progress_blocks;
  int index;
  int progress_block_size;
  long int bytes_programmed, file_length;
  char progress[20 + 1];
  char progress_char[1 + 1] = {0xFF, 0x00}; // block character from Hitachi data sheet
  char bytes_programmed_string[50 + 1];

// DJF Note - why is this using flash_magic_1 before it has been set?

  // outputs data from binary file to flash
  Flash_Read(PinState, flash_magic_1);
  flash_magic_1 = FLASH_START + 0xAAAA;
  flash_magic_2 = FLASH_START + 0x5554;
  //flash_magic_1 = FLASH_START + 0x5555; // AMD flash address 5555
  //flash_magic_2 = FLASH_START + 0x2AAA; // AMD flash address 2AAA

  A = start_address;
  in = fopen(input_file_name, "rb");

  if(in != NULL)
  {
printf("DJF DEBUG: writing data from \"%s\" starting at %lX\n", input_file_name, start_address);

    // work out how many bytes are being written
    //
    fseek(in, 0L, SEEK_END); // move to end of file
    file_length = ftell(in); // get current position
    fseek(in, 0L, SEEK_SET); // move back to start of file

    progress_block_size = file_length/columns;
    bytes_programmed = 0;
    lcdClear(lcdHandle);
    lcdPosition(lcdHandle, 0, 0);
    lcdPuts(lcdHandle, "Programming Flash...");

    while ((c = fgetc(in)) != EOF)
    {
       new_word = 0;			// initialises the two byte word
       new_word = (new_word | c);	// Puts 1st byte into low 8 bits
       c = fgetc(in);			// gets 2nd byte
       high_part = 0;			// initialises the temporary space
       high_part = (high_part | c);	// puts 2nd byte into low 8 bits
       high_part = high_part << 8;	// shifts 2nd byte to top
       new_word = new_word | high_part;	

       if(A >= FLASH_START)
       {
          bytes_programmed += 2;

          if(new_word != 0xFFFF) // Skip to save time - it's already 0xFFFF
          {
             Flash_Write(PinState, flash_magic_1, 0xAA);
             Flash_Write(PinState, flash_magic_2, 0x55);
             Flash_Write(PinState, flash_magic_1, 0xA0);// Unlock flash write

             Flash_Write(PinState, A, new_word);	// Write the data
          }
          if((A & 0xFF) == 0)
          {
             printf("%08lX (flash)\r)", (long)A);

             sprintf(bytes_programmed_string, "%ldKB programmed", bytes_programmed/1024);
             lcdPosition(lcdHandle, 0, rows - 2);
             lcdPuts(lcdHandle, bytes_programmed_string);
          }
          if((bytes_programmed % progress_block_size) == 0)
          {
             // update the progress bar display
             strcpy(progress, "");
             progress_blocks = bytes_programmed/progress_block_size;
             for(index = 0; index < progress_blocks; index++)
             {
                strncat(progress, progress_char, columns); // show 20 progress blocks total (screen width)
             }
             lcdPosition (lcdHandle, 0, rows - 1);
             lcdPuts (lcdHandle, progress);
          }
       }
       else
       {
          RAM_Write(PinState, A, new_word);
          if((A & 0xFF) == 0)
          {
             printf("%08lX (RAM)\r)", (long)A);
          }
       }

       // DJF To Do = if verify == true

       A += 2;
    }

    if(fclose(in))
    {
       printf("File close error!\n");
    }
    return (A - start_address);
  }
  else
  {
printf("DJF DEBUG: Failed to open \"%s\"\n", input_file_name);
    return 0;
  }
}

// DJF Note: tried various delay methods but all took too long
//           as raspberry pi timers are not all that fine.
//           Using timers took four or five hours to dump 8MB
//
//           I tried a simple loop counter but I think it got
//           optimised out as not doing anything.
//           This wait_cyles function will ensure <n> cpu nop
//           instructions are executed.
//           Initilal test have show 250 nops was unreliable
//           on pi zerow but 300 nops worked.
//           Timings for pi 3B+ to follow
//
void wait_cycles(unsigned int n)
{
   if(n)
   {
      while(n--)
      {
         asm volatile("nop");
      }
   }
}

word SwapWordBytes(word inputWord)
{
   return ((inputWord>>8) | (inputWord<<8)) & 0xFFFF;
}

