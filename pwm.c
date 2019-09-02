#include "pwm.h"
 #include "rcpinchange.h"
void inittimer2(void)
{
TCCR2A|=(1<<WGM21);//ctc mode
TCCR2B|=(1<<CS20)|(1<<CS21)|(1<<CS22);//1024 prescaler
TIMSK2|=(1<<OCIE2A);//enable output compare A interrupt
OCR2A=45;//compare value 30//49
}

 void escvalues(unsigned long esc1,unsigned long esc2,unsigned long esc3,unsigned long esc4)
 {
    unsigned long endtime1,endtime2,endtime3,endtime4;
    unsigned long timesc;
    timesc=micros();//read time using timer0
    PORTD|=(1<<0)|(1<<1)|(1<<2)|(1<<3);//all esc pins high till time of each one end
    endtime1=esc1+timesc;//calculate end time
    endtime2=esc2+timesc;
    endtime3=esc3+timesc;
    endtime4=esc4+timesc;
	while((PIND & ((1<<0)|(1<<1)|(1<<2)|(1<<3)))!=0x00)//stay till 0-3 pins are low
    //while( PORTD !=0b00000000)//stay till 0-3 pins are low
	{
	timesc=micros();
	if(endtime1<=timesc){PORTD|=(1<<0);}//reset pins when time pass the width we need
	if(endtime2<=timesc){PORTD|=(1<<1);}
	if(endtime3<=timesc){PORTD|=(1<<2);}
	if(endtime4<=timesc){PORTD|=(1<<3);}
	}
 }
