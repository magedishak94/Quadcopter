
float pid_roll_setpoint;//output of converting function
float pid_pitch_setpoint;
float pid_yaw_setpoint;
//extern int ch1,ch2,ch3,ch4;
float pid_pitch_setpoint,pid_roll_setpoint,pid_yaw_setpointl;//receiver values after converting
int esc1,esc2,esc3,esc4;
//extern float GyX,GyY,GyZ;

int pid_output_roll, pid_output_pitch, pid_output_yaw;

void pid_limit(void);
void calculate_pid(void);
void calculate_pwm(void);
