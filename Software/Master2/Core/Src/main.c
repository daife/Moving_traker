/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern DMA_HandleTypeDef hdma_usart1_tx;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
				uint8_t toTrans[9];
		float Pitch, Roll, Yaw;
		long Temp;
		uint8_t angle[2];
		short duty[2];
		float Yawp=0;
	//	float Yawp=-180;//yaw正修正，往180度推
	//	float Yawd=0;//yaw负修正，往0度以下推
	//	uint8_t adapt=1;//修正状态为负修正
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
		{
			
			gyro_data_ready_cb();
			

//			//按照正负修正修正Yaw,//注意修正不能超过90°
//			if(adapt==1){//0以下
//	Yaw=Yaw-Yawd;
//}else{
//Yaw=Yaw-(Yawp+180);
//}//得执行两次，这样就不会有90度限制了
//			if (Yaw < 0&&Yaw>=-90) {
//				Yawp=-180;
//				adapt=1;//改为负修正
//				if(Yaw<Yawd){
//				Yawd=Yaw;}
//			}else if (Yaw >= -180&&Yaw<-90) {
//    Yawd = 0;
//				adapt=0;
//				if(Yaw>Yawp){
//				Yawp=Yaw;}
//}

//			//按照正负修正修正Yaw,//注意修正不能超过90°
//			if(adapt==1){//0以下
//	Yaw=Yaw-Yawd;
//}else{
//Yaw=Yaw-(Yawp+180);
//}//得执行两次，这样就不会有90度限制了，前一次获得修正角度可供本次再次修正

if(Yaw>=0&&Yaw<=180){
Yawp=Yaw;
}
float roll_plus_90 = Roll + 90;
//if (roll_plus_90 < 0&&roll_plus_90>-90) {
//    roll_plus_90 = 0;
//} else if (roll_plus_90 >= -180&&roll_plus_90<=-90) {
//    roll_plus_90 = 180;
//}
//必须z指向水平面以下的时候初始化

angle[0]=Yawp;
		angle[1]=roll_plus_90;
		duty[0] = (10 *  Yawp/ 180.0 + 2.5) / 100.0 * 2000;
    duty[1] = (10 * roll_plus_90/ 180.0 + 2.5) / 100.0 * 2000;


			uint8_t dutyxoy1=0;//拆分发送
	uint8_t dutyxoy2=0;
		uint8_t dutyz1=0;
		uint8_t dutyz2=0;
		dutyxoy1=(duty[0]>>8)&0xff;
		dutyxoy2=duty[0]&0xff;
		dutyz1=(duty[1]>>8)&0xff;
		dutyz2=duty[1]&0xff;
		toTrans[0]=0xff;
		toTrans[1]=angle[0];
		toTrans[2]=angle[1];
		toTrans[3]=dutyxoy1;
		toTrans[4]=dutyxoy2;
		toTrans[5]=dutyz1;
		toTrans[6]=dutyz2;
		toTrans[7]=0;
		for (int i=0;i<7;i++){
		toTrans[7]+=toTrans[i];
		}//计算完就发�?�本次数�?

		HAL_UART_Transmit_IT(&huart1,(uint8_t*)toTrans,9);
		}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
mpu6050_init();
HAL_Delay(1000);
toTrans[8]=0xFE;
HAL_UART_Transmit_IT(&huart1,(uint8_t*)toTrans,9);
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {






    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
  /* USER CODE END 3 */
}
	}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
