#include "pid.h"
#include "gyro-acc.h"
#include "rcpinchange.h"
//ch1 roll
//ch2 pitch
//ch3 throttle
//ch4 yaw

void pid_limit(void){

pid_roll_setpoint = 0;
//We need a little dead band of 16us for better results.
if(ch1 > 1508)pid_roll_setpoint = (ch1 - 1508)/3.0;
else if(ch1 < 1492)pid_roll_setpoint = (ch1 - 1492)/3.0;

//The PID set point in degrees per second is determined by the pitch receiver input.
//In the case of dividing by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
pid_pitch_setpoint = 0;
//We need a little dead band of 16us for better results.
if(ch2 > 1508)pid_pitch_setpoint = (ch2 - 1508)/3.0;
else if(ch2 < 1492)pid_pitch_setpoint = (ch2 - 1492)/3.0;

//The PID set point in degrees per second is determined by the yaw receiver input.
//In the case of dividing by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
pid_yaw_setpoint = 0;
//We need a little dead band of 16us for better results.
if(ch3 > 1050){ //Do not yaw when turning off the motors.
	if(ch4 > 1508)pid_yaw_setpoint = (ch4 - 1508)/3.0;
	else if(ch4 < 1492)pid_yaw_setpoint = (ch4 - 1492)/3.0;
	}
}
void calculate_pid(void)
{
float gyro_roll_input,gyro_pitch_input,gyro_yaw_input;
float pid_error_temp;
float pid_i_roll, pid_p_roll, pid_d_roll, pid_last_roll_d_error;
float pid_i_pitch, pid_p_pitch, pid_d_pitch, pid_last_pitch_d_error;
float pid_i_yaw, pid_p_yaw, pid_d_yaw, pid_last_yaw_d_error;
float pid_p_gain_roll = 1.4;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.05;              //Gain setting for the roll I-controller (0.05)
float pid_d_gain_roll = 15;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = 1.4;              //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.05;             //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 15;               //Gain setting for the pitch D-controller.
int pid_max_pitch = 400;                   //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)
//change all to 0 and start allover again
pid_i_roll = 0;
pid_last_roll_d_error = 0;
pid_i_pitch = 0;
pid_last_pitch_d_error = 0;
pid_i_yaw = 0;
pid_last_yaw_d_error = 0;

gyro_roll_input=GyX;
gyro_pitch_input=GyY;
gyro_yaw_input=GyZ;

//Roll calculations
pid_error_temp = gyro_roll_input - pid_roll_setpoint; //roll error
pid_i_roll += pid_i_gain_roll * pid_error_temp;
if(pid_i_roll > pid_max_roll)pid_i_roll = pid_max_roll;
else if(pid_i_roll < pid_max_roll * -1)pid_i_roll = pid_max_roll * -1;  //i calculation
pid_p_roll = pid_p_gain_roll * pid_error_temp;  //p calculation
pid_d_roll = pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error); //d calculations 

pid_output_roll = pid_p_roll + pid_i_roll + pid_d_roll ; //pid roll output
if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1; // pid output limitations

pid_last_roll_d_error = pid_error_temp;

//Pitch calculations
pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
pid_i_pitch += pid_i_gain_pitch * pid_error_temp;
if(pid_i_pitch > pid_max_pitch)pid_i_pitch = pid_max_pitch;
else if(pid_i_pitch < pid_max_pitch * -1)pid_i_pitch = pid_max_pitch * -1;  // i calculations
pid_p_pitch = pid_p_gain_pitch * pid_error_temp;  //p calculation
pid_d_pitch = pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error); //d calculations

pid_output_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch ; //pid pitch output
if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

pid_last_pitch_d_error = pid_error_temp;

//Yaw calculations
pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
pid_i_yaw += pid_i_gain_yaw * pid_error_temp;
if(pid_i_yaw > pid_max_yaw)pid_i_yaw = pid_max_yaw;
else if(pid_i_yaw < pid_max_yaw * -1)pid_i_yaw = pid_max_yaw * -1;  // i calculations
pid_p_yaw = pid_p_gain_yaw * pid_error_temp;  //p calculation
pid_d_yaw = pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error); //d calculations

pid_output_yaw = pid_p_yaw + pid_i_yaw + pid_d_yaw ; //pid yaw output
if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

pid_last_yaw_d_error = pid_error_temp;
}
void calculate_pwm(void)
{
int throttle;
throttle = ch3;
if(throttle > 1800)throttle = 1800;                   //We need some room to keep full control at full throttle.
esc1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
esc2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
esc3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
esc4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
}