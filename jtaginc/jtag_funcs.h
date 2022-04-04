#ifndef JTAG_FUNCS_H
#define JTAG_FUNCS_H

//#include <stdio.h>
//#include <stdlib.h>
#include "./jtag_types.h"
#include "./jtag.h"

// Function Prototypes
void Fill_JTAG(Pchar);
void Reset_JTAG(void);
void Restore_Idle(void);

void Send_Instruction(unsigned, char *);
void Send_Instruction_IN(unsigned, char *);
void Send_Data(unsigned, char *);
void Send_Data_IN(unsigned, char *);
word IO_Read(Pchar, unsigned long int, int);
void Flip_ID_String(int, char[]);
void Shift_Data_Array(unsigned, char *);
void Shift_Data_Array_IN(unsigned, char *);
long Parse_ID(char[]);
void Get_Data(PJTAGdata);
void Set_Data(PJTAGdata, word);
void Set_Address(PJTAGdata, unsigned long int);
word Parse_Data(Pchar);
void Clear_Strobes(Pchar);
int Memory_Read(Pchar, unsigned long int, int);
int Flash_Read(Pchar, unsigned long int);
void Flash_Write(Pchar, unsigned long int, word);
int RAM_Read(Pchar, unsigned long int);
void RAM_Write(Pchar, unsigned long int, word);
int Exp_RAM_Read(Pchar, unsigned long int);
void Erase_Flash(void);
int Get_Flash_ID(void);
int Get_Flash_Dev(void);
unsigned long int Program_Flash_Data(char * filename, unsigned long int start_address, int lcdHandle, int rows, int columns);
//void Read_FLASH_Data(TBD);
void wait_cycles(unsigned int);
//static inline uint32_t ccnt_read (void);
void test_wait_cycles(void);

void TMS_Low(void);
void TMS_High(void);

word SwapWordBytes(word);

#endif // JTAG_FUNCS_H include sentry
