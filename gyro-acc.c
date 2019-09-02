#include "gyro-acc.h"
#include <math.h>
#include <util/delay.h>
void readsensor()  //get raw data from gyro-acc
{
	TWIStart(); //start i2c protocol
	TWIWrite(MPU_address);
	TWIWrite(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
	_delay_us(10);
	TWIStart(); //repeated start
	TWIWrite(MPU_address+1);//sensor address + read(1)

	AcX= TWIReadACK()<<8|TWIReadACK();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
	AcY=TWIReadACK()<<8|TWIReadACK(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	AcZ=TWIReadACK()<<8|TWIReadACK();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	Tmp=TWIReadACK()<<8|TWIReadACK(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	GyX=TWIReadACK()<<8|TWIReadACK(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	GyY=TWIReadACK()<<8|TWIReadACK();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	GyZ=TWIReadACK()<<8|TWIReadNACK(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
	TWIStop();
	

}

void calibrate(void){
	float gx=0,gy=0,gz=0,ax=0,ay=0,az=0;

	readsensor();

	_delay_ms(50);
	for (int i=0;i<=1000;i++){
		readsensor();
		gx+=GyX;
		gy+=GyY;
		gz+=GyZ;
		ax+=AcX;
		ay+=AcY;
		az+=AcZ;
		_delay_ms(5);
	}
	baseGX=gx/1000;
	baseGY=gy/1000;
	baseGZ=gz/1000;
	baseAX=ax/1000;
	baseAY=ay/1000;
	baseAZ=az/1000;

}
void initgyro(void){
	last_x=0;
	last_y=0;
	last_z=0;
	TWIStart();
	TWIWrite(MPU_address);
	TWIWrite(0x6B);// PWR_MGMT_1 register
	TWIWrite(0b00000001);// set to zero (wakes up the MPU-6050) $and use gyro xaxis as clock source
	_delay_ms(50);
	TWIStop();
	_delay_ms(20);
	TWIStart();
	TWIWrite(MPU_address);
	TWIWrite(0x1b);//configure gyro range
	TWIWrite(0b00011000);// 2000 deg/sec

	TWIStop();
	/*
	TWIStart();
	TWIWrite(MPU_address);
	TWIWrite(0x38);//configure int enable reg
	TWIWrite(00000001);// interrupt on data ready
	TWIStop();
	*/
	TWIStart();
	TWIWrite(MPU_address);
	TWIWrite(0x19);//configure sample rate dvdr reg    sample rate=8000/reg+1
	TWIWrite(31);// 250hz=31
	TWIStop();
	calibrate();
	
}
void getangles(void){
	double dt=0.004;//1/250
	readsensor();
	///////////////////////////*gyro*///////////////////////////////
	GyX-=baseGX;
	GyY-=baseGY;
	GyZ-=baseGZ;
	GyX /=16.4;  
	GyY /=16.4;
	GyZ /=0.8; 
	//dt=(timenow-last_time)/1000;
	float gx=(dt*GyX)+last_x;
	float gy=(dt*GyY)+last_y;
	float gz=(dt*GyZ)+last_z;
	///////////////////////////yaw///////////////////////////////
	//degree to radian 0.0174532925
	//gx+=gy*sin(gz*0.0174532925);
	//gy-=gx*sin(gz*0.0174532925);
	////////////////////////////accelerometer///////////////////////////

	float RADIANS_TO_DEGREES = 180/3.14159;
	float ax = atan(AcY/sqrt(pow(AcX,2) + pow(AcZ,2)))*RADIANS_TO_DEGREES;
	float ay = atan(-1*AcX/sqrt(pow(AcY,2) + pow(AcZ,2)))*RADIANS_TO_DEGREES;//why -1 ?
	//float az=  0;
	///////////////////////////complementary filter///////////////////////////
	float alpha = 0.90;
	fangle_x = alpha*gx + (1.0 - alpha)*ax;
	fangle_y = alpha*gy + (1.0 - alpha)*ay;
	//float fangle_z = alpha*gz + (1.0 - alpha)*az;
	fangle_z =gz;
	/////////////////////////////update old values///////////////////////////
	//last_time=timenow;
	last_x=fangle_x;
	last_y=fangle_y;
	last_z=fangle_z;
}