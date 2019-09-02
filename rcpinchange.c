/*
 * rcpinchange.c
 *
 * Created: 5/1/2017 9:03:07 PM
 *  Author: RAMY
 */ 
 #include "rcpinchange.h"
 void initimer0(void)
 {
	 
	 TCCR0B |=0b00000011;//set prescaler to 64 >>>>>> 1 clk tick every 8 microsec
	 TIMSK0 |=0b00000001;//set timer0 overflow interrupt enable
 }
 unsigned long micros(void)
 {
	 return (((timer0ovf*256)+TCNT0)*8);//return time in microsecond 
 }
 void initPCIportb(void)
 {
	 PCICR |=(1<<PCIE0);//enable PCI on portb
	 PCMSK0 |=(1<<PCINT0)|(1<<PCINT4)|(1<<PCINT5)|(1<<PCINT6);//enable PCI on PBO BP4,5,6
 }