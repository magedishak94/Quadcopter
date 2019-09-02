
//#include <util/delay.h>
#include "stdint.h"
#include "i2c.h"

#define  MPU_address 0xd0//for read   and for write d1
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //for signed numbers

unsigned long last_time, time_now;
float last_x;
float last_y ;
float last_z ;
float fangle_x,fangle_y,fangle_z;
float baseGX, baseGY, baseGZ, baseAX, baseAY, baseAZ;
void read_sensor();
void calibrate(void);
void initgyro(void);
void getangles(void);