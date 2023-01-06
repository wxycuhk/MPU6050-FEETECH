#include "delay.h"
#include "math.h"
#include "mpu6050.h"
#include "mpuiic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "Kalman.h"
#include "stdlib.h"
#include "usart.h"


float pitch,roll,yaw; 		//欧拉角(DMP)
short aacx,aacy,aacz;			//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
short temp;								//温度	
float angle_acc, angle_gyro; //通过加速度正交分解计算的角度和角速度积分计算的角度， 方便调参
float aacx_lpf, aacy_lpf, aacz_lpf;
float gyrox_hpf, gyroy_hpf, gyroz_hpf;
float angle_lpf,angle_hpf, angle_cps, angle_1st_cps;
short aacx_0=0,aacy_0=0,aacz_0=0;			//加速度计零偏
short gyrox_0=0,gyroy_0=0,gyroz_0=0;	//陀螺仪零偏

struct PI_Offset    PI_Offset;

#define RAD2DEG 57.295779513
#define gauge 5

float Angle_X_Final,Angle_Y_Final;

//获取陀螺仪加速度计零飘平均值
/*MPU6050初始化后立即调用*/

float map(float key, float x_a, float x_b, float y_a, float y_b)
{
	float k;
	float b;
	float result;
	k = (float)(y_a - y_b)/(x_a - x_b);
	b = y_a - k * x_a;
	result = k * key + b;
	return result;
}

void Aac_Gyro_Zero_Shift_Init(void)
{
	u16 i;
	
	long compensate[6]={0};
	
	delay_ms(100);
	

	for(i=0;i<2000;i++)
	{
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
		
		compensate[0]+=aacx;
		compensate[1]+=aacy;
		compensate[2]+=aacz-16384;
		compensate[3]+=gyrox;
		compensate[4]+=gyroy;
		compensate[5]+=gyroz;
		
//		delay_us(10);
	}
	
	aacx_0=compensate[0]/2000;
	aacy_0=compensate[1]/2000;
	aacz_0=compensate[2]/2000;
	gyrox_0=compensate[3]/2000;
	gyroy_0=compensate[4]/2000;
	gyroz_0=compensate[5]/2000;
	
	//printf("%d  %d  %d  %d  %d  %d\r\n",aacx_0,aacy_0,aacz_0,gyrox_0,gyroy_0,gyroz_0);
	
}
/******
PI校准：计算所得补偿量会实时更新到所得数据中，获取新的误差，可实现闭环动态控制以期达到稳定值
采用PI控制,根据过往稳态数据消除静差（零飘）
******/
void PID_Calibration_Accel(float K_P, float K_I, int Loops){
	
	int16_t Data;
	int i, L, c;
	float Reading;
	float data_get[3];
	float Error, PTerm, ITerm[3];
	//static float PI_Acc_Offset[3] = {0};
	int16_t eSample;
	uint32_t eSum;
	printf("Accel Calib Started\r\n");
	for(L = 0; L < Loops; L++){
		printf("Current is accel calibration round %d\r\n",L);
		eSample = 0;
		for (c = 0; c < 100; c++){
			eSum = 0;
			MPU_Get_Accelerometer(&aacy, &aacz, &aacx);
			data_get[0] = (float)aacx + PI_Offset.PI_Acc_Offset[0]; //实时更新补偿后数据
			data_get[1] = (float)aacy + PI_Offset.PI_Acc_Offset[1];
			data_get[2] = (float)aacz + PI_Offset.PI_Acc_Offset[2];
			printf("The Collected data are %.3f %.3f %.3f\r\n", (float)aacx/16384, (float)aacy/16384, (float)aacz/16384);
			printf("The Calibrated data are %.3f %.3f %.3f\r\n", data_get[0], data_get[1], data_get[2]);
			printf("\r\n");
			for(i = 0; i < 3; i++){
				Reading = data_get[i];
				if(i==2) Reading -= 16384;
				Error = -Reading;
				eSum += fabs(Reading);
				PTerm = K_P *Error;
				ITerm[i] += (Error * 0.001) * K_I; //由于采样率及定时器中断设定，每秒计算200次
				PI_Offset.PI_Acc_Offset[i] = (float)((PTerm + ITerm[i])/8);
				printf("Current offset for %d is %.3f, Current Error is %.3f\r\n", i, PI_Offset.PI_Acc_Offset[i], Error);
			}
			printf("\r\n");
			if((c = 99) && eSum > 300){  //一轮（每轮对100个采样数据组进行计算）pi更新后，误差仍大于需求范围，则重启该轮计算
				c = 0;
				//printf("Error is still too large for this loop\r\n");
			}
			if(eSum * 0.05 < 5) eSample++; // Successfully found offsets prepare to advance
			if((eSum < 100) && (c > 10) && (eSample >= 10)) break;
			delay_ms(1);
		}
		K_P *= 0.75;
		K_I *= 0.75;
		/*
		for (i = 0; i < 3; i++){
			PI_Offset.PI_Acc_Offset[i] = round((ITerm[i] ) / 8);		//Compute PID Output
			printf("Current Offset for %d is %.3f", i, PI_Offset.PI_Acc_Offset[i]);
			//Data = ((Data)&0xFFFE) |BitZero[i];	// Insert Bit0 Saved at beginning
			//} else Data = round((ITerm[i]) / 4);
			//I2Cdev::writeWords(devAddr, SaveAddress + (i * shift), 1, (uint16_t *)&Data);
		}
		*/
		printf("Current Offset for %d is %.3f", i, PI_Offset.PI_Acc_Offset[0]);
	}
}
void PID_Calibration_Gyro(float K_P, float K_I, int Loops){
	
	int16_t Data;
	int i, L, c;
	float Reading;
	float data_get[3];
	float Error, PTerm, ITerm[3];
	//static float PI_Acc_Offset[3] = {0};
	int16_t eSample;
	uint32_t eSum;
	printf("Gyro Calibration started\r\n");
	for(L = 0; L < Loops; L++){
		eSample = 0;
		printf("Current is Gyro calibration round %d",L);
		for (c = 0; c < 100; c++){
			eSum = 0;
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
			data_get[0] = (float)gyrox + PI_Offset.PI_Gyro_Offset[0]; //实时更新补偿后数据
			data_get[1] = (float)gyroy + PI_Offset.PI_Gyro_Offset[1];
			data_get[2] = (float)gyroz + PI_Offset.PI_Gyro_Offset[2];
			
			for(i = 0; i < 3; i++){
				Reading = data_get[i];
				//if(i==2) Reading -= 16384;
				Error = -Reading;
				eSum += fabs(Reading);
				PTerm = K_P *Error;
				ITerm[i] += (Error * 0.001) * K_I; //由于采样率及定时器中断设定，每秒计算200次
				PI_Offset.PI_Gyro_Offset[i] = (float)((PTerm + ITerm[i])/4);
			}
			if((c = 99) && eSum > 500){  //一轮（每轮对100个采样数据组进行计算）pi更新后，误差仍大于需求范围，则重启该轮计算
				c = 0;
				printf("First Loop trial failed! Error still too large, restart...\r\n");
			}
			if(eSum * 0.05 < 5) eSample++; // Successfully found offsets prepare to advance
			if((eSum < 100) && (c > 10) && (eSample >= 10)) break;
			delay_ms(1);
		}
		K_P *= 0.75;
		K_I *= 0.75;
		for (i = 0; i < 3; i++){
			PI_Offset.PI_Gyro_Offset[i] = round((ITerm[i] ) / 4);		//Compute PID Output
			//Data = ((Data)&0xFFFE) |BitZero[i];	// Insert Bit0 Saved at beginning
			//} else Data = round((ITerm[i]) / 4);
			//I2Cdev::writeWords(devAddr, SaveAddress + (i * shift), 1, (uint16_t *)&Data);
		}
	}
}

void Calibrate_Accel(int Loops)
{
	double K_P = 0.3;
	double K_I = 20;
	float x;
	
	x = (100 - map((float)Loops, 1, 5, 20, 0)) * 0.01;
	K_P *= x;
	K_I *= x;
	
	PID_Calibration_Accel(K_P, K_I, Loops);
}

void Calibrate_Gyro(int Loops)
{
	double K_P = 0.3;
	double K_I = 90;
	float x;
	
	x = (100 - map((float)Loops, 1, 5, 20, 0)) * 0.01;
	K_P *= x;
	K_I *= x;
	
	PID_Calibration_Gyro(K_P, K_I, Loops);
}
//陀螺仪加速度计零飘补偿
/*获取MPU6050数据后调用补偿*/
void Aac_Gyro_Zero_Shift_compensate(void)
{
	aacx+=PI_Offset.PI_Acc_Offset[0];
	aacy+=PI_Offset.PI_Acc_Offset[1];
	aacz+=PI_Offset.PI_Acc_Offset[2];
	
	gyrox+=PI_Offset.PI_Gyro_Offset[0];
	gyroy+=PI_Offset.PI_Gyro_Offset[1];
	gyroz+=PI_Offset.PI_Gyro_Offset[2];
	
	//printf("gyro_x is %d after 0-shift compensate\r\n", gyrox);

}

void Kalman_Filter_X()
{
	static float Accel;
	static float Gyro;
	
	static float angle_dot;
	static float Q_angle=0.001;// 过程噪声的协方差
	static float Q_gyro=0.003;//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
	static float R_angle=0.5;// 测量噪声的协方差 既测量偏差
	static float dt=0.005;//                 
	static char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

	Accel =atan2(aacy,aacz) * RAD2DEG;
	//if (Accel > 5)
		//Accel = 5;//gauge to delete abnormal
	angle_acc = Accel;
	//printf("%.5f,%.5f,%.3f\r\n",(float)aacy/16384,(float)aacz/16384,angle_acc);
	Gyro=gyrox/16.4;
	//printf("Gyro is %.2f after lsp calculation\r\n", Gyro);

	Angle_X_Final+=(Gyro - Q_bias) * dt; //先验估计， Q_bias为根据前结果估计的预测误差
	angle_gyro = Angle_X_Final;
	//printf("Angle_X calculated by integration of gyroscope is: %.2f\r\n", Angle_X_Final);
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0] + PP[1][1] * dt; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_X_Final;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_X_Final	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}

void Kalman_Filter_Y()
{
	static float Accel;
	static float Gyro;
	
	static float angle_dot;
	static float Q_angle=0.1;// 过程噪声的协方差
	static float Q_gyro=0.003;//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
	static float R_angle=0.5;// 测量噪声的协方差 既测量偏差
	static float dt=0.0048;//                 
	static char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

	Accel=-atan2(aacx,aacz)*RAD2DEG;
	Gyro=gyroy/16.4;
	

	Angle_Y_Final+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0] + PP[1][1] * dt; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_Y_Final;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	//printf("Kalman Gain Y for angle is: %.2f\r\n", K_0);
	//printf("Kalman Gain Y for gyro bias is %.2f\r\n", K_1);
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_Y_Final	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
	//printf("Angular velocity of y after eliminate 0-drift effect is: %.2f\r\n", angle_dot);
}

/******
 * Effect of the digital low pass filter is not very good after analyzing by matlab
 * Add 2nd-order lpf for accelerometer, data from accelerameter will be processed by lpf
 * Set the 3dB cut-off frequency as 20Hz
 * Refresh of data follows rule of FIFO
 ******/

float lpf_2nd_x()
{
	static float lpf_input_x[3] = {0,0,0};
	static float lpf_output_x[3] = {0,0,0};
	
	lpf_input_x[0] = lpf_input_x[1]; // refresh last the last last data, first in first out
	lpf_input_x[1] = lpf_input_x[2];
	lpf_input_x[2] = (float)aacx;
	
	lpf_output_x[0] = lpf_output_x[1];
	lpf_output_x[1] = lpf_output_x[2];
	lpf_output_x[2] = (float) ((lpf_input_x[2]+ 2 * lpf_input_x[1]+ lpf_input_x[0]) + 16.944 * lpf_output_x[1] - 6.120 * lpf_output_x[0])/14.825;
		
	return lpf_output_x[2];
}


float lpf_2nd_y(){
  
  static float lpf_input_y[3] = {0,0,0};
  static float lpf_output_y[3] = {0,0,0};
  lpf_input_y[0] = lpf_input_y[1];
  lpf_input_y[1] = lpf_input_y[2];
  lpf_input_y[2] = (float)aacy;

  lpf_output_y[0] = lpf_output_y[1];
  lpf_output_y[1] = lpf_output_y[2];
  lpf_output_y[2] =(float)((lpf_input_y[2]+ 2 * lpf_input_y[1]+ lpf_input_y[0]) + 16.944 * lpf_output_y[1] - 6.120 * lpf_output_y[0])/14.825; //2nd order low-pass filter with cutoff frequency of 20Hz and fs = 200Hz
  
	return lpf_output_y[2];
}


float lpf_2nd_z(){
  
  static float lpf_input_z[3] = {0,0,0};
  static float lpf_output_z[3] = {0,0,0};
  lpf_input_z[0] = lpf_input_z[1];
  lpf_input_z[1] = lpf_input_z[2];
  lpf_input_z[2] = (float)aacz;

  lpf_output_z[0] = lpf_output_z[1];
  lpf_output_z[1] = lpf_output_z[2];
  lpf_output_z[2] =(float)((lpf_input_z[2]+ 2 * lpf_input_z[1]+ lpf_input_z[0]) + 16.944 * lpf_output_z[1] - 6.120 * lpf_output_z[0])/14.825;
  
	return lpf_output_z[2];
}


/********
2022/12/21 Testing result:

2nd_order hpf will cut off the moving data as well ---> reduce cut off frequency
********/

float hpf_2nd_x(){
	static float hpf_input_x[3]={0,0,0};
	static float hpf_output_x[3];
	hpf_input_x[0] = hpf_input_x[1];
	hpf_input_x[1] = hpf_input_x[2];
	hpf_input_x[2] = (float)gyrox/16.4;
	
	hpf_output_x[0] = hpf_output_x[1];
	hpf_output_x[1] = hpf_output_x[2];
	hpf_output_x[2] = (float)((hpf_input_x[2] - 2*hpf_input_x[1] + hpf_input_x[0]) + 1.789 * hpf_output_x[1] - 0.646 * hpf_output_x[0])/1.565; //20Hz cut off frequency
	//hpf_output_x[2] = (float)(0.95654 * (hpf_input_x[2] - 2 * hpf_input_x[1] + hpf_input_x[0]) + 1.9112 * hpf_output_x[1] - 0.915 * hpf_output_x[0]);//2Hz cut off fr
	//printf("%.3f %.3f %.3f %.3f %.3f %.3f\r\n",hpf_input_x[0],hpf_input_x[1],hpf_input_x[2],hpf_output_x[0],hpf_output_x[1],hpf_output_x[2]);
	return hpf_output_x[2];
}

float hpf_2nd_y(){
	static float hpf_input_y[3]={0,0,0};
	static float hpf_output_y[3] = {0,0,0};
	hpf_input_y[0] = hpf_input_y[1];
	hpf_input_y[1] = hpf_input_y[2];
	hpf_input_y[2] = gyroy;
	
	hpf_output_y[0] = hpf_output_y[1];
	hpf_output_y[1] = hpf_output_y[2];
	hpf_output_y[2] = (float)((hpf_input_y[2] - 2*hpf_input_y[1] + hpf_input_y[0]) + 1.789 * hpf_output_y[1] - 0.646 * hpf_output_y[0])/1.565;
	
	return hpf_output_y[2];
}

float hpf_2nd_z(){
	static float hpf_input_z[3] = {0,0,0};
	static float hpf_output_z[3] = {0,0,0};
	hpf_input_z[0] = hpf_input_z[1];
	hpf_input_z[1] = hpf_input_z[2];
	hpf_input_z[2] = gyroz;
	
	hpf_output_z[0] = hpf_output_z[1];
	hpf_output_z[1] = hpf_output_z[2];
	hpf_output_z[2] = (float)((hpf_input_z[2] - 2*hpf_input_z[1] + hpf_input_z[0]) + 1.789 * hpf_output_z[1] - 0.646 * hpf_output_z[0])/1.565;
		
	return hpf_output_z[2];
}
/****** Intgration of 2nd order Butterworth High pass filter seems to be quite strange ******/

float Angle_Calcu(int mode)
{
	mpu_dmp_get_data(&pitch,&roll,&yaw);			//DMP角度--（对比用）
			
	temp=MPU_Get_Temperature();								//得到温度值
	//printf("%.3f %.3f %.3f ",PI_Offset.PI_Acc_Offset[0],PI_Offset.PI_Acc_Offset[1],PI_Offset.PI_Acc_Offset[2]);
	MPU_Get_Accelerometer(&aacy,&aacz,&aacx);	//得到加速度传感器数据
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
	//printf("%.3f ",(float)gyrox);
	//printf("%.3f %.3f %.3f ", (float)aacx, (float)aacy, (float)aacz);
	/*PID Calibration of collected data*/
	//Aac_Gyro_Zero_Shift_compensate();
	//printf("%.3f %.3f %.3f\r\n", (float)aacx, (float)aacy, (float)aacz);
	/*Butterworth filter data postprocessing*/
	
	aacx_lpf = lpf_2nd_x();//对加速度计数据进行巴特沃斯二阶低通滤波
	aacy_lpf = lpf_2nd_y();
	aacz_lpf = lpf_2nd_z();

	gyrox_hpf = hpf_2nd_x();//对陀螺仪数据进行巴特沃斯二阶高通滤波
	gyroy_hpf = hpf_2nd_y();
	gyroz_hpf = hpf_2nd_z();
	
	/*Compare the different angles get by different methods*/
	
  angle_lpf = atan2(aacy_lpf,sqrt(pow(aacx_lpf,2)+pow(aacz_lpf,2)))*RAD2DEG;
	angle_gyro = angle_gyro + (float)gyrox/16.4 * 0.005;
	//gyrox_hpf = hpf_2nd_x();
	angle_hpf += gyrox_hpf/16.4 * 0.005;
	angle_cps = angle_lpf + angle_hpf;
	angle_acc = atan2(aacy,sqrt(pow(aacx,2)+pow(aacz,2)))*RAD2DEG;
	angle_1st_cps = 0.95 * angle_acc + 0.05 * angle_gyro;
	//printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\r\n",(float)aacx/16384,(float)aacy/16384,(float)aacz/16384,(float)aacx_lpf/16384,(float)aacy_lpf/16384,(float)aacz_lpf/16384, angle_acc, angle_lpf, (float)pitch, (float)angle_cps);
	//printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f\r\n",(float)pitch, (float)angle_gyro, (float)angle_acc, (float)angle_hpf, (float)angle_lpf, (float)angle_1st_cps, (float)angle_cps);
	//printf("%.3f %.3f \r\n",(float)angle_gyro,(float)angle_hpf);
	//Aac_Gyro_Zero_Shift_compensate();					//陀螺仪加速度计零飘补偿
	printf("%.3f,%.3f\r\n",(float)pitch,(float)roll);
	//USART_SendData(USART1,13);
	//USART_SendData(USART1,10);
	
	
	//Kalman_Filter_X();
	//printf(" %.3f\r\n",Angle_X_Final);
	//Kalman_Filter_Y();
	//printf("%.3f,%.3f,%.3f\r\n",Angle_X_Final, angle_acc, angle_gyro);
	switch(mode)
	{
		case 1:
		{
			return Angle_X_Final;
		}
		case 2:
		{
			return Angle_Y_Final;
		}
	}
	
	//printf("%f, %f\r\n",Angle_X_Final,Angle_Y_Final);
}
