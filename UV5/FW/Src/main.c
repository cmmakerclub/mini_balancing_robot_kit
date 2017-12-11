/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

#include "eeprom.h"
/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x0010, 0x0020, 0x0030, 0x0040, 0x0050};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0, 0, 0};
uint16_t VarValue = 0;

#define rr           	            0x01 			    
#define PD_Controller           	0x02 
#define Enable_Motor 							0x04
#define PI_Steering						    0x08
#define Car_mode                  0x10    		
#define acc_calibation_mode       0x20 
#define gyro_calibation_mode      0x40  
#define Manual_Motor_control 			0x80 




/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

typedef struct __attribute((__packed__))Control_data{ 
	
		uint16_t mode;  // / BIT define 7:   6:    5:    4:Car   3:ON/OFF PI Steering Controller    2:ON/OFF Motors     1:ON/OFF PD Angle Controller    0:Set Calibate Sensor Offset
	
	uint16_t AreRobotStanding;

		int16_t Pitch_Ref, Steering_Ref;
	
		int16_t Power_MotorL, Power_MotorR;
	
		int16_t roll, pitch, yaw;
	
		int16_t GyroX, GyroY, GyroZ;
	
		int16_t AccX, AccY, AccZ;
	
		int16_t OffsetGyroX, OffsetGyroY, OffsetGyroZ;
	
		int16_t OffsetAccX, OffsetAccY, OffsetAccZ;
		
		uint16_t VR1, VR2, VR3, VR4;

		int32_t Position_motorL, Position_motorR;

}Control_data;

Control_data State_data;

float Ramp;
				
float Robot_speed  = 0;
float Wheel_L_speed  = 0;
float Wheel_R_speed  = 0;

float cal_pitch ;
float cal_roll  ;
float turn_speed;
float turn  ;
float motor_A_tmp ;
float motor_B_tmp ;
float motor_Y_tmp ;
float error_turn;
float error_turn_sum;
float Robot_Target_Angle;		

void Read_epprom(void){
  HAL_FLASH_Unlock();
  EE_Init();
	
  EE_ReadVariable(VirtAddVarTab[0], (uint16_t*)&(State_data.OffsetAccX));
  EE_ReadVariable(VirtAddVarTab[1], (uint16_t*)&(State_data.OffsetAccY));
  EE_ReadVariable(VirtAddVarTab[2], (uint16_t*)&(State_data.OffsetGyroX));
  EE_ReadVariable(VirtAddVarTab[3], (uint16_t*)&(State_data.OffsetGyroY));
  EE_ReadVariable(VirtAddVarTab[4], (uint16_t*)&(State_data.OffsetGyroZ));
	HAL_FLASH_Lock();
}

void Write_epprom(void){
  HAL_FLASH_Unlock();
  EE_Init();

	EE_WriteVariable(VirtAddVarTab[0], (uint16_t)(State_data.OffsetAccX));
	EE_WriteVariable(VirtAddVarTab[1], (uint16_t)(State_data.OffsetAccY));
	EE_WriteVariable(VirtAddVarTab[2], (uint16_t)(State_data.OffsetGyroX));
	EE_WriteVariable(VirtAddVarTab[3], (uint16_t)(State_data.OffsetGyroY));
	EE_WriteVariable(VirtAddVarTab[4], (uint16_t)(State_data.OffsetGyroZ));
	HAL_FLASH_Lock();
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

int32_t EncoderL, EncoderR;
const uint8_t AdcBufferSize = 40 ;
uint16_t ADC_Raw_Data[AdcBufferSize] = {0};
uint16_t VR_Data[4] = {0};
uint8_t Blink_led1_freq = 80;
uint8_t Cal_pin = 0, cal_gyro_man = 0, cal_acc_man = 0;

#include "bs_bb_lib.h"

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	State_data.mode = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
 // MX_IWDG_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
	
	HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)&spi_rx_cmd, 1);


	/* EEPROM Init */

	Read_epprom();

	
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC_Raw_Data, AdcBufferSize);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	
	TIM1->CNT = 15000;
	TIM2->CNT = 15000;
	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

	Initial_MPU6050();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

	int count = 0;

	uint32_t timenow, time1, time2;

	MX_IWDG_Init();
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		HAL_IWDG_Refresh(&hiwdg);
		timenow = milli();

		

			
//		if(timenow - time1 >= 20){
//			time1 = timenow;
//			
//			float dR =  ((int)TIM2->CNT - 15000);
//			float dL = -((int)TIM1->CNT - 15000);
//			TIM2->CNT = 15000;
//			TIM1->CNT = 15000;		
//			
//			EncoderR += dR;
//			EncoderL += dL;
//	
//			Wheel_R_speed  = (int)Smooth_filter(0.4f, dR, Wheel_R_speed);		
//			Wheel_L_speed  = (int)Smooth_filter(0.4f, dL, Wheel_L_speed);		
//			
//			Robot_speed  = Smooth_filter(0.1f, (dR+dL) * 0.5f, Robot_speed);		
//			
//			
//			speed = Smooth_filter(0.3f, (float)ch2*0.9f, speed);
//			turn = Smooth_filter(0.3f, (float)ch1*0.8f, turn);
//			
//			if(((State_data.mode)&(0x01<<3)) == PI_Controller) {
//				
//				error_speed = constrain(Kp_Speed *(-speed + Robot_speed), -15, 15);
//				error_speed_sum = constrain((error_speed_sum + error_speed * 0.02f), -20/Ki_Speed, 20/Ki_Speed);
//				Robot_Target_Angle = constrain(Ki_Speed * error_speed_sum + error_speed, -25, 25);
//				
//			} 	
//		}

		if(_Sampling_task_do){
			static float buffer_az;
			_Sampling_task_do = 0;

			int16_t dL =  ((int)TIM2->CNT - 15000);
			int16_t dR = -((int)TIM1->CNT - 15000);
			TIM2->CNT = 15000;
			TIM1->CNT = 15000;		
			
			State_data.Position_motorL += dL;
			State_data.Position_motorR += dR;
			
			Read_MPU6050();
			AHRS();

			buffer_az = Smooth_filter(0.2f, (float)rawAccx_Z/ACCELEROMETER_SENSITIVITY, buffer_az);
			State_data.AreRobotStanding = (abs_user(q_pitch) < 500) && (buffer_az < 0);

			if(((State_data.mode)&(PD_Controller)) == PD_Controller) {
				
				float Buf_D_Errer_pitch = Errer_pitch;


				Robot_Target_Angle = constrain((float)State_data.Pitch_Ref * 0.1f, -45, 45);
				
				cal_pitch = Smooth_filter(0.95f, (float)q_pitch*0.1f, cal_pitch);
				cal_roll = Smooth_filter(0.95f, (float)q_roll*0.1f, cal_roll);

				Errer_pitch =  Robot_Target_Angle + (float)cal_pitch;
				
				D_Error_pitch = Butterworth_filter(&filed_pitch,(Errer_pitch-Buf_D_Errer_pitch) * sampleFreq * 0.1f);

				Del_pitch	= (Kp_pitch * Errer_pitch) + (Kd_pitch * D_Error_pitch);
				
				
			} else{
				
				Del_pitch = constrain((float)State_data.Pitch_Ref, -2999, 2999);
				
			}
			
			
			if(((State_data.mode)&(PI_Steering)) == PI_Steering) {
				static float cal_yaw;
				
				turn = constrain((float)State_data.Steering_Ref * 0.1f, -2000, 2000);
				cal_yaw = Smooth_filter(0.90f, ((float)rawGyrox_Z)/GYROSCOPE_SENSITIVITY, cal_yaw);
				Error_yaw = turn - (cal_yaw);
				
				if(((State_data.mode)&(Enable_Motor)) == Enable_Motor){
					Sum_Error_yaw = constrain(Sum_Error_yaw + Error_yaw*period_dt, -300/Ki_yaw, 300/Ki_yaw);
				}else{
					Sum_Error_yaw = 0;
				}

				Del_yaw = (Error_yaw*Kp_yaw) + (Sum_Error_yaw*Ki_yaw);
			}else{
				
				Del_yaw = constrain((float)State_data.Steering_Ref, -2999, 2999);
			}
			
			
			
			motor_Y_tmp =  constrain(motor_Y_tmp + (float)Del_yaw  *0.02f, -2000, 2000);
			motor_A_tmp =  constrain(motor_A_tmp + (float)Del_pitch*0.02f, -2999, 2999);
			motor_B_tmp =  constrain(motor_B_tmp + (float)Del_pitch*0.02f, -2999, 2999);



			
			if(((State_data.mode)&(Car_mode)) == Car_mode){
				
				if(Robot_speed>0){
					
					motor_L = motor_A_tmp  + motor_Y_tmp;
					motor_R = motor_B_tmp  - motor_Y_tmp;		
					
				}else{			
					
					motor_L = motor_A_tmp  - motor_Y_tmp;
					motor_R = motor_B_tmp  + motor_Y_tmp;										
				}
				
			}else{
				motor_L = motor_A_tmp  + motor_Y_tmp;
				motor_R = motor_B_tmp  - motor_Y_tmp;		
			}

			if(timenow > 3000){

				if(Ramp < 1) Ramp += 0.004f;
				
				motor_L *=  Ramp;
				motor_R *=  Ramp;
				

				
				if(((State_data.mode)&(Manual_Motor_control)) == Manual_Motor_control){

					motor_L =  State_data.Power_MotorL;
					motor_R =  State_data.Power_MotorR;;

				}else{

				}
				
				if(((State_data.mode)&(Enable_Motor)) == Enable_Motor) {
					
					Drive_motor_output();
					
				}else{
					Ramp = 0;
					motor_Y_tmp =  0;
					motor_A_tmp =  0;
					motor_B_tmp =  0;
					
					motor_L =  0;
					motor_R =  0;
					
					Drive_motor_output();
				}
			}
			

			count++;
			if(count>Blink_led1_freq){
				count = 0;
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				HAL_Delay(1);
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			}

		}


		if(timenow - time1 >= 10){
			time1 = timenow;	
			uint32_t VR_Sum[4] = {0};
			for(int x = 0; x < (AdcBufferSize/4); x++){
				VR_Sum[0] += ADC_Raw_Data[x*4 + 0];
				VR_Sum[1] += ADC_Raw_Data[x*4 + 1];      
				VR_Sum[2] += ADC_Raw_Data[x*4 + 2];
				VR_Sum[3] += ADC_Raw_Data[x*4 + 3];				
			}
			
			for(int x = 0; x < 4; x++){
				VR_Data[x] = VR_Sum[x] / (AdcBufferSize/4);		
			}	
			
			State_data.VR1 = VR_Data[0];
			State_data.VR2 = VR_Data[1];
			State_data.VR3 = VR_Data[2];
			State_data.VR4 = VR_Data[3];
			
			Kp_pitch = (float)VR_Data[0] * 0.1f;
			Kd_pitch = (float)VR_Data[1] * 0.1f;
			Kp_yaw =   (float)VR_Data[2] * 0.015f;
			Ki_yaw =   (float)VR_Data[3] * 0.001f;
			
			Cal_pin = Cal_pin << 1;
			
			if( HAL_GPIO_ReadPin(Calibration_Pin_GPIO_Port, Calibration_Pin_Pin) ==  GPIO_PIN_RESET){
				Cal_pin = Cal_pin | 0x01;
			}

	
			if(Cal_pin == 0x0F) {
				cal_gyro_man = 1; 
				cal_acc_man  = 1;
			}
			
		}

		if(timenow - time2 >= 10){
			time2 = timenow;
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		}			
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4 ;
  hiwdg.Init.Window = 100;
  hiwdg.Init.Reload = 100;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C1_INT_Pin */
  GPIO_InitStruct.Pin = I2C1_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(I2C1_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Calibration_Pin_Pin */
  GPIO_InitStruct.Pin = Calibration_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Calibration_Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
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
	
	
	Blink_led1_freq = 80;
	if((((State_data.mode)&(gyro_calibation_mode)) == gyro_calibation_mode) || cal_gyro_man == 1) {
		static uint16_t calibate_cout;
		calibate_cout++;
		Blink_led1_freq = 25;

		gyx_d = Smooth_filter(0.005f, rawGyrox_X, gyx_d);
		gyy_d = Smooth_filter(0.005f, rawGyrox_Y, gyy_d);	
		gyz_d = Smooth_filter(0.005f, rawGyrox_Z, gyz_d);
//		acx_d = Smooth_filter(0.005f, rawAccx_X, acx_d);
//		acy_d = Smooth_filter(0.005f, rawAccx_Y, acy_d);	
//		acz_d = Smooth_filter(0.005f, rawAccx_Z, acz_d);	
		

   if(calibate_cout > 4000){
		 calibate_cout = 0;
		 cal_gyro_man = 0;
		 State_data.mode = State_data.mode & (0xff - gyro_calibation_mode);
		 
		 State_data.OffsetGyroX = gyx_d;
		 State_data.OffsetGyroY = gyy_d;
		 State_data.OffsetGyroZ = gyz_d;
//		 State_data.OffsetAccX  = acx_d;
//		 State_data.OffsetAccY  = acy_d;
//		 State_data.OffsetAccZ  = acz_d;
		 Write_epprom();
	 }
	} 
	
		if((((State_data.mode)&(acc_calibation_mode)) == acc_calibation_mode)|| cal_acc_man == 1 ) {
		static uint16_t calibate_cout;
		calibate_cout++;
		Blink_led1_freq = 25;
			
//		gyx_d = Smooth_filter(0.005f, rawGyrox_X, gyx_d);
//		gyy_d = Smooth_filter(0.005f, rawGyrox_Y, gyy_d);	
//		gyz_d = Smooth_filter(0.005f, rawGyrox_Z, gyz_d);
		acx_d = Smooth_filter(0.005f, rawAccx_X, acx_d);
		acy_d = Smooth_filter(0.005f, rawAccx_Y, acy_d);	
		acz_d = Smooth_filter(0.005f, rawAccx_Z, acz_d);	

   if(calibate_cout > 4000){
		 calibate_cout = 0;
		 cal_acc_man = 0;
		 State_data.mode = State_data.mode & (0xff - acc_calibation_mode);
		 
//		 State_data.OffsetGyroX = gyx_d;
//		 State_data.OffsetGyroY = gyy_d;
//		 State_data.OffsetGyroZ = gyz_d;
		 State_data.OffsetAccX  = acx_d;
		 State_data.OffsetAccY  = acy_d;
		 State_data.OffsetAccZ  = acz_d;
		 Write_epprom();
	 }
	} 
	
		rawGyrox_X -= State_data.OffsetGyroX;
		rawGyrox_Y -= State_data.OffsetGyroY;
		rawGyrox_Z -= State_data.OffsetGyroZ;
		
		rawAccx_X -= State_data.OffsetAccX;
		rawAccx_Y -= State_data.OffsetAccY;

}


void Drive_motor_output(void)
{
	if(motor_L > 0){
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


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)&spi_rx_cmd, 1);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){

	uint8_t tmp_spi_rx_cmd = spi_rx_cmd;
	spi_rx_cmd = 0;
	switch (tmp_spi_rx_cmd)
  {
		
  	case 0x01: //Set_Mode
				HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)&State_data.mode, 2);
  		break;
  	case 0x02: //Set_Control
				HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)&State_data.Pitch_Ref, 4);
  		break;
		case 0x03: //Get_Angles
				HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&State_data.roll, 6);
  		break;
		case 0x04: //Get_AngularVelocity
				HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&State_data.GyroX, 6);
  		break;
  	case 0x05: //Get_OffsetAngularVelocity
				HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&State_data.OffsetGyroX, 6);
  		break;
		case 0x06: //Get_Accelerations
				HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&State_data.AccX, 6);
  		break;
		case 0x07: //Get_OffsetAccelerations
				HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&State_data.OffsetAccX, 6);
  		break;
  	case 0x08: //Get_VRs
				HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&State_data.VR1, 8);
  		break;
		case 0x09: //Get_Positions
				HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&State_data.Position_motorL, 8);
  		break;
		case 0x0A: //Get_Power_motorControl
				HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)&State_data.Power_MotorL, 4);
  		break;
		
  	default:
				HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)&spi_rx_cmd, 1);
  		break;
  }

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{

}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
