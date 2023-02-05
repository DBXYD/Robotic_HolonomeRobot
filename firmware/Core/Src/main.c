/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "mpu6050.h"
#include "nrf24l01.h"
#include "motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define UART_TX_BUFFER_SIZE 64
#define UART_RX_BUFFER_SIZE 1

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uartTxBuffer[UART_TX_BUFFER_SIZE];
uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
unsigned int sizeUartTx=0;
uint8_t errorMsg[]="Erreur \r\n";
uint8_t tickMsg[]="Tick \r\n";

uint8_t spiRxBuffer[32];
uint8_t spiTxBuffer[32];
uint8_t rxAddrP0[5] = {0x00,0xDD,0xCC,0xBB,0xAA};
uint8_t rxAddrP1[5] = {0x00,0xDD,0xCC,0xBB,0xAA};
uint8_t rxAddrP2[5] = {0x00,0xDD,0xCC,0xBB,0xAA};
uint8_t rxAddrP3[5] = {0x00,0xDD,0xCC,0xBB,0xAA};
uint8_t rxAddrP4[5] = {0x00,0xDD,0xCC,0xBB,0xAA};
uint8_t rxAddrP5[5] = {0x00,0xDD,0xCC,0xBB,0xAA};
uint8_t txAddr[5] = {0x00,0xDD,0xCC,0xBB,0xAA};
uint8_t nrf24TxPayload[] = "Hello world from nrf24\r\n";
uint8_t nrf24RxPayload[32] = {};
MPU_Accel_float accel;
MPU_Gyro_float gyro;

uint8_t uartRxReceived;

Motor_HandleTypeDef motorA ={&htim1, TIM_CHANNEL_1, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 25, 3, 0};
Motor_HandleTypeDef motorB ={&htim1, TIM_CHANNEL_2, 0, 549, 0, 0, 0, 0, 0, 0, 0, 0, 25, 3, 0};
Motor_HandleTypeDef motorC ={&htim1, TIM_CHANNEL_3, 0, -2049, 0, 0, 0, 0, 0, 0, 0, 0, 25, 3, 0};


uint8_t consigne = 0;
int divPrint = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart){
	uartRxReceived = 1;
	HAL_UART_Transmit(&huart2, uartRxBuffer, 1, 100);
	HAL_UART_Receive_IT(&huart2, uartRxBuffer, UART_RX_BUFFER_SIZE);
}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	int stringLength;
	int inc = 0;
	int reg = 0;
	int mode = 0;

	int increment=1;
	// 0 power down
	// 1 rx mode
	// 2 tx mode
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_SPI1_Init();
	MX_TIM7_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, uartRxBuffer, UART_RX_BUFFER_SIZE);
	stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"*** Starting ***\r\n");
	HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
	//MPU6050_Init();
	//NRF24L01_Init();
	__HAL_TIM_SET_COUNTER(&htim2,37767);
	__HAL_TIM_SET_COUNTER(&htim3,37767);
	__HAL_TIM_SET_COUNTER(&htim4,37767);

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start_IT(&htim7);

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		motorA.speed64 = 1500;
		motorB.speed64 = 549;
		motorC.speed64 = -2049;
		HAL_Delay(1000);
		motorA.speed64 = 1500;
		motorB.speed64 = 1500;
		motorC.speed64 = 1500;
		HAL_Delay(1000);
		motorA.speed64 = -1500;
		motorB.speed64 = -549;
		motorC.speed64 = 2049;
		HAL_Delay(1000);
		motorA.speed64 = -1500;
		motorB.speed64 = -1500;
		motorC.speed64 = -1500;
		HAL_Delay(1000);

		/****************************************************************************************************************/
		/* Start -- Receive data from MPU6050 																			*/
		/****************************************************************************************************************/
		/*if(HAL_OK != MPU6050_Read_Accel(&accel)){
			HAL_UART_Transmit(&huart2, errorMsg, strlen((char *)errorMsg), HAL_MAX_DELAY);
		}
		if(HAL_OK !=  MPU6050_Read_Gyro(&gyro)){
			HAL_UART_Transmit(&huart2, errorMsg, strlen((char *)errorMsg), HAL_MAX_DELAY);
		}
		stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Speed (%1.3f,%1.3f,%1.3f)\r\n",accel.x,accel.y,accel.z);
		HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
		stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Gyro (%3.3f,%3.3f,%3.3f)\r\n",gyro.x,gyro.y,gyro.z);
		HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);*/
		/****************************************************************************************************************/
		/* End -- Receive data from MPU6050 																			*/
		/****************************************************************************************************************/

		/****************************************************************************************************************/
		/* Start -- Check status of NRF24L01 																			*/
		/****************************************************************************************************************/
		//HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, RESET);
		/*NRF24L01_WriteCSN(RESET);
		if(HAL_OK != HAL_SPI_TransmitReceive(//&hspi1, spiTxBuffer, spiRxBuffer, 1, 100)){
			stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"SPI Error\r\n",spiRxBuffer[0]);
			HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
		}
		else{
			stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"SPI data received : Ox%2x\r\n",spiRxBuffer[0]);
			HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
		}*/
		/*if(HAL_OK != NRF24L01_GetStatus(spiRxBuffer, 100)){
			stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"SPI Error\r\n",spiRxBuffer[0]);
			HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
		}
		else{
			stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"SPI data received : Ox%2x\r\n",spiRxBuffer[0]);
			HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
		}*/

		/****************************************************************************************************************/
		/* End -- Check status of NRF24L01	 																			*/
		/****************************************************************************************************************/

		/****************************************************************************************************************/
		/* Start -- Read register of NRF24L01 and get Rx Addr on all pipes												*/
		/****************************************************************************************************************/
		/*NRF24L01_ReadRegister(0, spiRxBuffer, 32);
		for(inc = 0; inc < 32; inc++){
			switch(inc){
			case 0 :
				stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Reg 0x00-0F : %2x",spiRxBuffer[inc]);
				HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				break;
			case 16 :
				stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Reg 0x10-1F : %2x",spiRxBuffer[inc]);
				HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				break;
			case 15 :
			case 31 :
				stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE," | %2x\r\n",spiRxBuffer[inc]);
				HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				break;
			default :
				stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE," | %2x",spiRxBuffer[inc]);
				HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				break;
			}
		}


		NRF24L01_GetRxAddr(0, rxAddrP0);
		stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Rx Addr P0 :  0x%2x%2x%2x%2x%2x \r\n",rxAddrP0[0],rxAddrP0[1],rxAddrP0[2],rxAddrP0[3],rxAddrP0[4]);
		HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);

		NRF24L01_GetRxAddr(1, rxAddrP1);
		stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Rx Addr P1 :  0x%2x%2x%2x%2x%2x \r\n",rxAddrP1[0],rxAddrP1[1],rxAddrP1[2],rxAddrP1[3],rxAddrP1[4]);
		HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);

		NRF24L01_GetRxAddr(2, rxAddrP2);
		stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Rx Addr P2 :  0x%2x%2x%2x%2x%2x \r\n",rxAddrP2[0],rxAddrP2[1],rxAddrP2[2],rxAddrP2[3],rxAddrP2[4]);
		HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);

		NRF24L01_GetRxAddr(3, rxAddrP3);
		stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Rx Addr P3 :  0x%2x%2x%2x%2x%2x \r\n",rxAddrP3[0],rxAddrP3[1],rxAddrP3[2],rxAddrP3[3],rxAddrP3[4]);
		HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);

		NRF24L01_GetRxAddr(4, rxAddrP4);
		stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Rx Addr P4 :  0x%2x%2x%2x%2x%2x \r\n",rxAddrP4[0],rxAddrP4[1],rxAddrP4[2],rxAddrP4[3],rxAddrP4[4]);
		HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);

		NRF24L01_GetRxAddr(5, rxAddrP5);
		stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Rx Addr P5 :  0x%2x%2x%2x%2x%2x \r\n",rxAddrP5[0],rxAddrP5[1],rxAddrP5[2],rxAddrP5[3],rxAddrP5[4]);
		HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
		 */
		/****************************************************************************************************************/
		/* End -- Read register of NRF24L01 and get Rx Addr on all pipes												*/
		/****************************************************************************************************************/

		/****************************************************************************************************************/
		/* Begin -- Write register of NRF24L01 																			*/
		/****************************************************************************************************************/
		/*
		NRF24L01_ReadRegister(reg, spiRxBuffer, 1);
		stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Before Writing #00 : 0x%2x\r\n",spiRxBuffer[inc]);
		HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);

		spiTxBuffer[0]=0x0A;
		NRF24L01_WriteRegister(reg, spiTxBuffer, 1);

		NRF24L01_ReadRegister(reg, spiRxBuffer, 1);
		stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"After Writing #00 : 0x%2x\r\n",spiRxBuffer[inc]);
		HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
		 */
		/****************************************************************************************************************/
		/* End -- Write register of NRF24L01 																			*/
		/****************************************************************************************************************/

		/****************************************************************************************************************/
		/* Begin -- Set Channel of NRF24L01 																			*/
		/****************************************************************************************************************/
		/*reg = RF_CH;
		NRF24L01_ReadRegister(reg, spiRxBuffer, 1);
		stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Before Writing #00 : 0x%2x\r\n",spiRxBuffer[inc]);
		HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);

		//spiTxBuffer[0]=0x0A;
		//NRF24L01_WriteRegister(reg, spiTxBuffer, 1);
		NFR24L01_SetChannel(0x1F);

		NRF24L01_ReadRegister(reg, spiRxBuffer, 1);
		stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"After Writing #00 : 0x%2x\r\n",spiRxBuffer[inc]);
		HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
		 */
		/****************************************************************************************************************/
		/* End -- Write register of NRF24L01 																			*/
		/****************************************************************************************************************/
		if(uartRxReceived){
			switch(uartRxBuffer[0]){
			case 't' :
				stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"\r\nSet Tx Mode\r\n");
				HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				mode = 2;
				NRF24L01_SetTxMode(txAddr, 10);
				break;
			case 'r' :
				stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"\r\nSet Rx Mode\r\n");
				HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				mode = 1;
				NRF24L01_SetRxMode(rxAddrP0, 10, 0);
				break;
			case 's' :
				if(mode!=2){
					stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"\r\nSet in Tx Mode before sending data\r\n");
					HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				}
				else{
					stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"\r\nSend data\r\n");
					HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);

					if(NRF24L01_WritePayload(nrf24TxPayload, sizeof(nrf24TxPayload))==HAL_OK){
						stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"\r\nTransmission Successful\r\n");
						HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
					}
					else{
						stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"\r\Transmission Error\r\n");
						HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
					}

				}
				break;
			case 'p' :
				if(mode!=1){
					stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"\r\nSet in Rx Mode before receving data\r\n");
					HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				}
				else if(NF24L01_IsDataAvailable(0)==HAL_OK){
					NRF24L01_ReadPayload(nrf24RxPayload, 32);
					stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"\r\nData received\r\n");
					HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
					stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"%s",nrf24RxPayload);
					HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				}
				else{
					stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"\r\nNo Data received\r\n");
					HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
					NRF24L01_ReadPayload(nrf24RxPayload, 32);
					stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"\r\nData received\r\n");
					HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);

				}
				for(int idx = 0; idx<32; idx++){
					stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"0x%2x",nrf24RxPayload[idx]);
					HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				}
				break;

			case '+' :
				motorA.Kp++;
				break;
			case '-' :
				motorA.Kp--;
				break;


			case '1' :
				motorA.speed64--;
				motorLimitSpeed(&motorA);
				stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Target Speed x64 = %5d\r\n",motorA.speed64);
				HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				break;
			case '3' :
				motorA.speed64++;
				motorLimitSpeed(&motorA);
				stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Target Speed x64 = %5d\r\n",motorA.speed64);
				HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				break;
			case '4' :
				motorA.speed64-=10;
				motorLimitSpeed(&motorA);
				stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Target Speed x64 = %5d\r\n",motorA.speed64);
				HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				break;
			case '6' :
				motorA.speed64+=10;
				motorLimitSpeed(&motorA);
				stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Target Speed x64 = %5d\r\n",motorA.speed64);
				HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				break;
			case '7' :
				motorA.speed64-=100;
				motorLimitSpeed(&motorA);
				stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Target Speed x64 = %5d\r\n",motorA.speed64);
				HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				break;
			case '9' :
				motorA.speed64+=100;
				motorLimitSpeed(&motorA);
				stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Target Speed x64 = %5d\r\n",motorA.speed64);
				HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				break;
			case '0' :
				motorA.speed64 = 0;
				stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Target Speed x64 = %5d\r\n",motorA.speed64);
				HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
				break;
			default:
				break;
			}
			uartRxReceived = 0;
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/*if(speedValue == 1023) increment = 1;
		if(speedValue == -1023) increment = -1;if(encSpeedMotorA < targetSpeed) speed +);
		speedValue += increment;
		motorSetSpeed(&htim1, TIM_CHANNEL_1, speedValue);
		motorSetSpeed(&htim1, TIM_CHANNEL_2, speedValue);
		motorSetSpeed(&htim1, TIM_CHANNEL_3, speedValue);
		HAL_Delay(10);
		 */


	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */
	int stringLength;
	// TIM7 = 10Hz for Encodeur mesuring
	if(htim->Instance ==  TIM7){

		motorA.encPos = __HAL_TIM_GET_COUNTER(&htim2);
		__HAL_TIM_SET_COUNTER(&htim2,37767);
		motorB.encPos = __HAL_TIM_GET_COUNTER(&htim3);
		__HAL_TIM_SET_COUNTER(&htim3,37767);
		motorC.encPos = __HAL_TIM_GET_COUNTER(&htim4);
		__HAL_TIM_SET_COUNTER(&htim4,37767);

		motorA.encSpeed64 = (motorA.encPos-37767)*24; // 24 = 2*pi/48*0.029*fe*64
		motorA.diffError64 = (motorA.speed64 - motorA.encSpeed64) - motorA.error64;
		motorA.error64 = (motorA.speed64 - motorA.encSpeed64);
		motorA.sumError64 += motorA.error64;
		motorA.commande64 = motorA.speed64*motorA.Kp + motorA.sumError64*motorA.Ki + motorA.diffError64*motorA.Kd;

		motorB.encSpeed64 = (motorB.encPos-37767)*24; // 24 = 2*pi/48*0.029*fe*64
		motorB.diffError64 = (motorB.speed64 - motorB.encSpeed64) - motorB.error64;
		motorB.error64 = (motorB.speed64 - motorB.encSpeed64);
		motorB.sumError64 += motorB.error64;
		motorB.commande64 = motorB.speed64*motorB.Kp + motorB.sumError64*motorB.Ki + motorB.diffError64*motorB.Kd;

		motorC.encSpeed64 = (motorC.encPos-37767)*24; // 24 = 2*pi/48*0.029*fe*64
		motorC.diffError64 = (motorC.speed64 - motorC.encSpeed64) - motorC.error64;
		motorC.error64 = (motorC.speed64 - motorC.encSpeed64);
		motorC.sumError64 += motorC.error64;
		motorC.commande64 = motorC.speed64*motorC.Kp + motorC.sumError64*motorC.Ki + motorC.diffError64*motorC.Kd;

		/*
		if(divPrint == 0){
			stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Kd A : {%4d} ", motorA.Kd);
			HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
			stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"PWM A : {%4d} ", motorA.pwm);
			HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
			stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Target A : {%4d} || ", motorA.speed64);
			HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
			stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Speed Enc A : {%4d} || ", motorA.encSpeed64);
			HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
			stringLength = snprintf((char *) uartTxBuffer,UART_TX_BUFFER_SIZE,"Erreur A : {%4d}\r\n", motorA.error64);
			HAL_UART_Transmit(&huart2, uartTxBuffer, stringLength, 100);
		}
		divPrint += 1;
		divPrint %= 100;
		 */

		motorSetSpeed(&motorA);
		motorSetSpeed(&motorB);
		motorSetSpeed(&motorC);
	}
	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
