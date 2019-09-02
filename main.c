/*
 * quadcopter.c
 *
 * Created: 5/1/2017 8:20:27 PM
 * Author : RAMY
 */ 
 //gyroscope,degree/sec or degree
 //rc
 //esc
 //pid
 //fusion
 //change all global variables u can to return values
 //make timer 0 again
 #define F_CPU (8000000UL)
#include "i2c.h"
#include "gyro-acc.h"//gyro_lib
#include "pwm.h"//pwm->esc_lib
#include "rcpinchange.h"//rc->pinchange_lib
#include "pid.h"
#include <avr/io.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
//notes:
/////////////////////////global_variables////////////////////////////
volatile char x=0;
//unsigned long time;
//unsigned long timer0ovf;
//int ch1,ch2,ch3,ch4;
//int esc1,esc2,esc3,esc4;
//float GyX,GyY,GyZ;
//float last_x;
//float last_y;
//float last_z;

int main(void)
{
 //initiate modules
 inittimer2();//initialize timer2 in ctc mode
 TWIInit();//initialize i2c protocol
 initgyro();//initialize gyroscope
 initimer0();//initialize timer0//used as stop watch for other functions
 initPCIportb();//initialize pin change interrupt
 ///////////////////
 //calibrate();
 //////////////////
 sei();
 _delay_ms(1000);
 for(int i=0;i<10;i++){
	 PORTD=0xff;
	 _delay_us(1000);
	 PORTD=0x00;
	 _delay_ms(4);
 }


    while (1) 
    {
	//readsensor();
	getangles();//update gyro angles
	pid_limit();//limit rc values 
	calculate_pid();//calculate pid values
	calculate_pwm();//calculate pwm for each esc
	if(x==1)
	{
	escvalues(esc1,esc2,esc3,esc4);
	x=0;
	}
    }
}

ISR(TIMER0_OVF_vect){
	timer0ovf++;
}
ISR(PCINT0_vect){
	time=micros();
	///////////////////////channel1/////////////////////
	if(PINB & 0b00000001){//input is high
		if(ch1_oldstate==0)
		{ch1_strt=time;
		ch1_oldstate=1;}
	}
	else if(ch1_oldstate==1) //input is low
	{
		ch1=time-ch1_strt;
		ch1_oldstate=0;
	}
	///////////////////////channel2/////////////////////
	if(PINB & 0b00010000){//input is high
		if(ch2_oldstate==0)
		{ch2_strt=time;
		ch2_oldstate=1;}
	}
	else if(ch2_oldstate==1) //input is low
	{
		ch2=time-ch2_strt;
		ch2_oldstate=0;
	}
	///////////////////////channel3/////////////////////
	if(PINB & 0b00100000){//input is high
		if(ch3_oldstate==0)
		{ch3_strt=time;
		ch3_oldstate=1;}
	}
	else if(ch3_oldstate==1) //input is low
	{
		ch3=time-ch3_strt;
		ch3_oldstate=0;
	}
	///////////////////////channel4/////////////////////
	if(PINB & 0b01000000){//input is high
		if(ch4_oldstate==0)
		{ch4_strt=time;
		ch4_oldstate=1;}
	}
	else if(ch4_oldstate==1) //input is low
	{
		ch4=time-ch4_strt;
		ch4_oldstate=0;
	}
}
ISR(TIMER2_COMPA_vect)
{
x=1;
	}
	