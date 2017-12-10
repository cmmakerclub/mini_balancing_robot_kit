/**
  ******************************************************************************
  * File Name          : 
  * Description        : 
  ******************************************************************************
  *
  ******************************************************************************
  */

#ifndef __BS_BB_LIB_H
#define __BS_BB_LIB_H


#define ACCELEROMETER_SENSITIVITY   4096.0f  
#define GYROSCOPE_SENSITIVITY       16.4f  
#define Compass_SENSITIVITY       	1090.0f
#define M_PIf       								3.14159265358979323846f
#define M_PI                        M_PIf    
#define sampleFreq                  500.0f // 500 hz sample rate!   
#define period_dt                   0.002f // 500 hz sample rate!   

#define spi_buffer_size 96

#define ARM_MATH_CM0

#include "stm32f0xx_hal.h"
#include "arm_math.h" 
#include "math.h" 
#include "MPU6050.h"


volatile uint8_t _Sampling_task_do = 0;

volatile uint8_t Mode = calibation_mode;

int16_t rawAccx_X = 1;
int16_t rawAccx_Y = 1;
int16_t rawAccx_Z = 1;
int16_t rawGyrox_X = 1;
int16_t rawGyrox_Y = 1;
int16_t rawGyrox_Z = 1;

typedef struct
{
  __IO float X2;
  __IO float X1;
  __IO float Y2;
  __IO float Y1;
  __IO float Y0;
} filted_data;

filted_data filed_yaw;
filted_data filed_pitch;

volatile int16_t magneticDeclination = 0;
volatile int16_t q_yaw, q_pitch, q_roll;                               		// States value
volatile float beta = 0.8f   ;
volatile float q0=1, q1=0, q2=0, q3=0;
volatile float T_center =0, yaw_center=0;
volatile float Error_yaw=0, Errer_pitch=0, Error_roll=0; 									//States Error
volatile float Sum_Error_yaw=0, Sum_Error_pitch=0, Sum_Error_roll=0;     	// Sum of error
volatile float D_Error_yaw=0, D_Error_pitch=0, D_Error_roll=0; 						// error dot
volatile float Del_yaw=0, Del_pitch=0, Del_roll=0;												// Delta states value for rotate axis
volatile float t_compensate = 0;
volatile float T_center_minus = 0;
volatile float y_roll=0, y_pitch=0, y0_roll=0, y0_pitch=0 ; 
volatile float rMat[3][3] = {0};

volatile float Ki_Speed = 0;
volatile float Kp_Speed = 0;

volatile float Kp_roll = 13.75;
volatile float Ki_roll = 5;
volatile float Kd_roll = 9;

volatile float Kp_pitch = 0;
volatile float Ki_pitch = 0;
volatile float Kd_pitch = 0;

volatile float Kp_yaw = 0;
volatile float Ki_yaw = 0;
volatile float Kd_yaw = 0;

// set gyro offset
int16_t gx_diff = 2;
int16_t gy_diff = 12;
int16_t gz_diff = -14;
int16_t ax_diff = -155;
int16_t ay_diff = -15;
int16_t az_diff = 0;

volatile uint8_t  calibation_pass = 0;
volatile float  Acc_start = ACCELEROMETER_SENSITIVITY;

volatile uint8_t  spi_rx_data[spi_buffer_size] = {0};
volatile uint8_t  spi_rx_data_index = 0;

uint16_t watchdog  = 0;
int16_t xxx = 0;

/* USER CODE for Receiver  */
volatile uint8_t _index = 0 ;
volatile float	ch1=0,ch2=0,ch3=0,ch4=0; 
volatile float	_ch1=0,_ch2=0,_ch3=0,_ch4=0;      
volatile float	motor_L=0, motor_R=0;// Motors output value 
uint8_t rx_tmp[14] = {0};
float T_center_buffer = 0;
volatile float a, b, c, d, e, f;

volatile float gyx_d = 1000;
volatile float gyy_d = 1000;
volatile float gyz_d = 1000;
volatile float acx_d = 1000;
volatile float acy_d = 1000;

float _gyx_d = INT16_MAX;
float _gyy_d = INT16_MAX;
float _gyz_d = INT16_MAX;
float _acx_d = INT16_MAX;
float _acy_d = INT16_MAX;
int16_t count;

void Initial_MPU6050(void);
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
void Read_MPU6050(void);
void PID_controller(void);
void Drive_motor_output(void);
void Sampling_task(void);
void AHRS(void);
float Smooth_filter(float alfa, float new_data, float prev_data);
float invSqrt(float x) ;
void imuComputeRotationMatrix(void);
float sq(float x);
float constrain(float x, float lower_b, float upper_b);

void getPIDgain(uint8_t spi_rx_data_index);
void getRCcommand(uint8_t spi_rx_data_index);
void readCommand (void);

float abs_user(float x);
uint32_t milli(void);
void calibation_fn(void);
void stabilize_fn(void);

float Butterworth_filter(filted_data* filted, float x_data);
void SET_filter_value(filted_data *filted, float value);


void calibation_fn(void)
{
	Read_MPU6050();

	float gx_x = abs_user(rawGyrox_X-gyx_d);
	float gy_x = abs_user(rawGyrox_Y-gyy_d);	
	float gz_x = abs_user(rawGyrox_Z-gyz_d);		
	float ax_x = abs_user(rawAccx_X-acx_d);
	float ay_x = abs_user(rawAccx_Y-acy_d);

	const	float deadband = 2;

	if ((gx_x < deadband)&&(gy_x < deadband)&&(gz_x < deadband)&&(ax_x < deadband)&&(ay_x < deadband))
	{
		Mode = stabilize_mode;
		calibation_pass = 1;
		
		gx_diff = gyx_d;
		gy_diff = gyy_d;
		gz_diff = gyz_d;
		ax_diff = acx_d;
		ay_diff = acy_d;

	}

	_gyx_d = gyx_d;
	_gyy_d = gyy_d;
	_gyz_d = gyz_d;
	_acx_d = acx_d; 
	_acy_d = acy_d;
	
}


void Initial_MPU6050(void)
{
	HAL_Delay(150); // for stability
	//    Reset to defalt 
	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, ENABLE);
	HAL_Delay(150);
	
	//    ENABLE
	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, DISABLE);
	HAL_Delay(1);
	//	  SetClockSource(MPU6050_CLOCK_PLL_ZGYRO)
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
	HAL_Delay(1);		

	//    SetFullScaleAccelRange(MPU6050_ACCEL_FS_8)
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_ACCEL_FS_8);
	HAL_Delay(1);
	
	//    SetFullScaleGyroRange(MPU6050_GYRO_FS_2000)
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_2000);
	HAL_Delay(1);
	
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_98);
	HAL_Delay(1);
			
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 7, 8, 1);
	HAL_Delay(1);

	//    interupt(Enable)
	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, ENABLE);
	HAL_Delay(1);
				
}

void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t tmp;
	  HAL_I2C_Mem_Read(&hi2c1, slaveAddr, regAddr, 1, &tmp, 1, 1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask;   // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data;    // combine data with existing byte
    HAL_I2C_Mem_Write(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
}

void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t tmp;
	  HAL_I2C_Mem_Read(&hi2c1, slaveAddr, regAddr, 1, &tmp, 1, 1);
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    HAL_I2C_Mem_Write(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
}

void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    uint8_t tmp;
	  HAL_I2C_Mem_Read(&hi2c1, slaveAddr, regAddr, 1, &tmp, 1, 1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
}

void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t tmp;
	  HAL_I2C_Mem_Read(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
    *data = tmp & (1 << bitNum);
}


void Read_MPU6050(void)
{
	int16_t AccelGyro[6]={0};  
    uint8_t tmpBuffer[14];
	HAL_I2C_Mem_Read(&hi2c1,MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_XOUT_H,1,tmpBuffer,14,2);
    /* Get acceleration */
    for (int i = 0; i < 3; i++)
        AccelGyro[i] = ((int16_t) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
    /* Get Angular rate */
    for (int i = 4; i < 7; i++)
        AccelGyro[i - 1] = ((int16_t) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
				
	rawAccx_X  =  AccelGyro[0];
	rawAccx_Y  =  AccelGyro[1];
	rawAccx_Z  =  AccelGyro[2];
	rawGyrox_X =  AccelGyro[3];
	rawGyrox_Y =  AccelGyro[4];
	rawGyrox_Z =  AccelGyro[5];
	
	
//	if (Mode == calibation_mode) 
//	{
		gyx_d = Smooth_filter(0.005f, rawGyrox_X, gyx_d);
		gyy_d = Smooth_filter(0.005f, rawGyrox_Y, gyy_d);	
		gyz_d = Smooth_filter(0.005f, rawGyrox_Z, gyz_d);
		acx_d = Smooth_filter(0.005f, rawAccx_X, acx_d);
		acy_d = Smooth_filter(0.005f, rawAccx_Y, acy_d);	
//		Acc_start = Smooth_filter(0.1f, (float)rawAccx_Z/ACCELEROMETER_SENSITIVITY, Acc_start);		
//		
//	}else{
//	
		rawGyrox_X -= gx_diff;
		rawGyrox_Y -= gy_diff;
		rawGyrox_Z -= gz_diff;
		
		rawAccx_X -= ax_diff;
		rawAccx_Y -= ay_diff;
//	}
}

static float min_acc = 999999999;
float recipNorm;

void AHRS(void)
{
	static float min_acc;
	
	const float dt = period_dt; 	
	float gx = -(((float)rawGyrox_X)/GYROSCOPE_SENSITIVITY)*(M_PIf/180.0f);
	float gy =  (((float)rawGyrox_Y)/GYROSCOPE_SENSITIVITY)*(M_PIf/180.0f);
	float gz = -(((float)rawGyrox_Z)/GYROSCOPE_SENSITIVITY)*(M_PIf/180.0f);
	float ax = -((float)rawAccx_X)/ACCELEROMETER_SENSITIVITY;
	float ay =  ((float)rawAccx_Y)/ACCELEROMETER_SENSITIVITY;
	float az = -((float)rawAccx_Z)/ACCELEROMETER_SENSITIVITY;


	float ex = 0, ey = 0, ez = 0;
	float qa, qb, qc;
	
	recipNorm = sq(ax) + sq(ay) + sq(az);
	
	//if(recipNorm < 1.05) min_acc = recipNorm;
	
	// Use measured acceleration vector

if(HAL_GetTick() >3000)beta = 0.15;
	
//  	if (recipNorm > 0.01f && recipNorm < sq(Acc_start) * 2.0f && delay_compensate == 0) 

  	if (recipNorm > 0.01f) {
			
//		if (recipNorm <= 1.0f){
//        // Normalise accelerometer measurement
//					beta = 0.5;
//				}else{
//					beta = 0.15;
//				}
				
        recipNorm = invSqrt(recipNorm);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * rMat[2][2] - az * rMat[2][1]);
        ey += (az * rMat[2][0] - ax * rMat[2][2]);
        ez += (ax * rMat[2][1] - ay * rMat[2][0]);
			
			  // Apply proportional and integral feedback
				gx += beta * ex;
				gy += beta * ey;
				gz += beta * ez;

			
		
	}



    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = q0;
    qb = q1;
    qc = q2;
		
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(sq(q0) + sq(q1) + sq(q2) + sq(q3));
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();
		/* Compute pitch/roll angles */
		
    q_roll = (atan2f(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf));
    q_pitch = -(((0.5f * M_PIf) - acosf(-rMat[2][0])) * (1800.0f / M_PIf));
		
		q_pitch += 15;
		
}

void Drive_motor_output(void)
{
	if(motor_L> 0){
		TIM3->CCR1 = constrain(2999, 0, 2999);
		TIM3->CCR2 = constrain(2999-motor_L, 0, 2999);
	}else{
		TIM3->CCR2 = constrain(2999, 0, 2999);
		TIM3->CCR1 = constrain(2999+motor_L, 0, 2999);
	}
	
	if(motor_R < 0){
		TIM3->CCR3 = constrain(2999, 0, 2999);
		TIM3->CCR4 = constrain(2999+ motor_R, 0, 2999);
	}else{
		TIM3->CCR4 = constrain(2999, 0, 2999);
		TIM3->CCR3 = constrain(2999-motor_R, 0, 2999);
	}
}


float constrain(float x, float lower_b, float upper_b)
{
	if(x < lower_b) x = lower_b;
	if(x > upper_b) x = upper_b;
	return x;
}

float sq (float x)
 {
	 return x*x;
 }
 
void imuComputeRotationMatrix(void)
{
    float q1q1 = sq(q1);
    float q2q2 = sq(q2);
    float q3q3 = sq(q3);
    
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3; 

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

float Smooth_filter(float alfa, float new_data, float prev_data)
{
  float output = prev_data + (alfa * (new_data - prev_data));
  return output;
}


float invSqrt(float x)
{
	return 1.0f / sqrtf(x);
}


float abs_user(float x)
{
	if(x<0) x = -x;
	return x;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
	if (GPIO_Pin == GPIO_PIN_10) _Sampling_task_do = 1;
}


uint32_t milli(void)
{
	return HAL_GetTick();
}

float Butterworth_filter(filted_data *filted, float x_data)
{
	// lowpass filter butterworth order 2nd fc  248 hz sampling 500hz

	filted->Y2 = filted->Y1;
	filted->Y1 = filted->Y0 ;
	
	float nume = (x_data + 2.0f * filted->X1 +  filted->X2);
	float denom = (1.9644605802052317f * filted->Y1 +  0.96508117389913461f * filted->Y2);
	filted->Y0 = 0.98238543852609161f * nume - denom;
	
	filted->X2 = filted->X1;
	filted->X1 = x_data;
	
	return filted->Y0;
}

void SET_filter_value(filted_data *filted, float value)
{
	// lowpass filter butterworth order 2nd fc  248 hz sampling 500hz

	filted->Y2 = value;
	filted->Y1 = value;
	filted->Y0 = value;
	filted->X2 = value;
	filted->X1 = value;

}   

void getRCcommand(uint8_t spi_rx_data_index)
{
	
	int8_t	roll_tmp     = (int8_t)spi_rx_data[spi_rx_data_index-4];  
	int8_t	pitch_tmp    = (int8_t)spi_rx_data[spi_rx_data_index-3]; 
	int8_t	throttle_tmp = (int8_t)spi_rx_data[spi_rx_data_index-2];  
	int8_t	yaw_tmp      = (int8_t)spi_rx_data[spi_rx_data_index-1];  
	int8_t	sum_tmp      = (int8_t)spi_rx_data[spi_rx_data_index];
	int8_t	sum  = roll_tmp + pitch_tmp + throttle_tmp + yaw_tmp;

	if ((int8_t)sum_tmp == (int8_t)sum)
	{
		ch1 = roll_tmp;
		ch2 = pitch_tmp;
		ch3 = throttle_tmp;
		ch4 = yaw_tmp;
		watchdog = 500;
	}
}

void readCommand (void)
{
	if (spi_rx_data_index >= 6)
	{
		for (uint8_t count = spi_rx_data_index ; count >= 6 ; count--)
		{
			uint8_t command_code = 0xfd ;
			if (spi_rx_data[count-1] == command_code && spi_rx_data[count] == command_code )
			{
				getRCcommand(count);
				spi_rx_data_index = 0;
				break;
			}
		}
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	static uint8_t command_code;


	spi_rx_data_index = spi_rx_data_index + 1;	
			
			
		if (spi_rx_data_index >= 7)
		{
			/*----------------------------------RC COMMAND-----------------------------------*/
			command_code = 0xfd ;
			if (spi_rx_data[spi_rx_data_index-2] == command_code && spi_rx_data[spi_rx_data_index-1] == command_code )
			{
				getRCcommand(spi_rx_data_index-3);
				spi_rx_data_index = 0;
			}


		}
	
	
	if (spi_rx_data_index == spi_buffer_size) spi_rx_data_index = 0;

	HAL_SPI_Receive_IT(&hspi1, (uint8_t*)(spi_rx_data+spi_rx_data_index), 1);
}

#endif
