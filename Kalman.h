#ifndef _Kalman_
#define _Kalman_

extern float pitch,roll,yaw; 		//欧拉角(DMP)
extern short aacx,aacy,aacz;			//加速度传感器原始数据
extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据
extern short temp;								//温度

struct PI_Offset
{
	float PI_Acc_Offset[3];
	float PI_Gyro_Offset[3];
};

void Aac_Gyro_Zero_Shift_Init(void);
void Aac_Gyro_Zero_Shift_compensate(void);
float Angle_Calcu(int mode);
float lpf_2nd_x(void);
float lpf_2nd_y(void);
float lpf_2nd_z(void);
float map(float key, float x_a, float x_b, float y_a, float y_b);
void PID_Calibration_Accel(float K_P, float K_I, int Loops);
void PID_Calibration_Gyro(float K_P, float K_I, int Loops);
void Calibrate_Accel(int Loops);
void Calibrate_Gyro(int Loops);


#endif
