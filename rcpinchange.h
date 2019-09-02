
#include <avr/io.h>
#include <stdio.h>
#include <math.h>
//#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
 unsigned long ch1_strt,ch2_strt,ch3_strt,ch4_strt;
  int ch1,ch2,ch3,ch4;
 uint8_t ch1_oldstate ,ch2_oldstate ,ch3_oldstate ,ch4_oldstate ;//search for smaller variable
  unsigned long time;
  unsigned long timer0ovf;
 void initimer0(void);
 unsigned long micros(void);
 void initPCIportb(void);
 //note  all chx_oldstate was =0 i removed all to remove error ask if it needed to be in the c file 
