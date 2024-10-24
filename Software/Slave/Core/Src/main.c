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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include <math.h>
#include "key.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//extern DMA_HandleTypeDef hdma_usart2_rx;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATA_SIZE 6  // 数据大小，不包括起始、结束标志和校验�?
#define PKT_SIZE (DATA_SIZE + 2 + 1) // 整个数据包大小，包括起始、结束标志和校验�?
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t angle[2];
uint16_t duty[2];
uint8_t isBT = 0; //
uint8_t p=0;
uint8_t KeyCode=0;

uint8_t rxBuffer[PKT_SIZE];
uint8_t rxIndex = 0;
uint8_t state = 0; // 0: 寻找起始0xFF, 1: 接收数据, 2: 接收校验和和结束0xFF
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim3)
  {
    KEY_Scan();
  }
}


// 计算校验和的函数
uint8_t CalculateChecksum(uint8_t *data, uint8_t len) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len+1; i++) {
        checksum += data[i];
    }
    return checksum;
}

void changeMark(){
		if(isBT==0){
		OLED_ShowString(48,0,"Key",OLED_8X16);
			if(p==0){OLED_ShowChar(0,16,'>',OLED_8X16);
				OLED_ClearArea(0,32,8,16);
			}else{
			OLED_ShowChar(0,32,'>',OLED_8X16);
				OLED_ClearArea(0,16,8,16);
			}
		}else{
			OLED_ShowString(48,0,"BT ",OLED_8X16);
			OLED_ClearArea(0,16,8,32);
		}
				OLED_UpdateArea(48,0,24,16);
		OLED_UpdateArea(0,16,8,32);
}


// 串口接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart==&huart2) {
        switch (state) {
            case 0: // 寻找起始0xFF
                if (rxBuffer[rxIndex] == 0xFF) {
                    rxIndex++;
                    state = 1; // 进入接收数据状态
                }
                break;
            case 1: // 接收数据
                rxIndex++;
                if (rxIndex == DATA_SIZE + 1) { // 数据接收完毕
                    state = 2; // 准备接收校验和
                    
                } 
                break;
            case 2: // 接收校验和
                rxIndex++;
                if (rxIndex == DATA_SIZE + 2) {
                    // 校验和接收完毕，进入接收结束0xFE状态
                    state = 3;
                } else {
                    // 数据错位，重置状态机
                    state = 0;
                    rxIndex = 0;
                }
                break;
            case 3: // 接收结束0xFE
                if (rxBuffer[rxIndex] == 0xFE) {
                    // 检查校验和
         
                    if (rxBuffer[DATA_SIZE+1] == CalculateChecksum(rxBuffer, DATA_SIZE)) {
                        // 校验和正确，处理数据
											if(isBT==1){
                        angle[0]=rxBuffer[1];
		angle[1]=rxBuffer[2];
		duty[0]=(rxBuffer[3]<<8)|rxBuffer[4];
		duty[1]=(rxBuffer[5]<<8)|rxBuffer[6];
											}                       
  
                    }
                    // 重置状态机，准备接收下一包数据
                    state = 0;
                    rxIndex = 0;
                } else {
                    // 数据错位，重置状态机
                    state = 0;
                    rxIndex = 0;
                }
                break;
        }
        // 重新启动中断接收
        HAL_UART_Receive_IT(&huart2, &rxBuffer[rxIndex], 1);
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
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	
	HAL_GPIO_WritePin(GPIOC,led_Pin,GPIO_PIN_RESET);
HAL_Delay(300);
HAL_GPIO_WritePin(GPIOC,led_Pin,GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC,led_Pin,GPIO_PIN_RESET);
HAL_Delay(300);
HAL_GPIO_WritePin(GPIOC,led_Pin,GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC,led_Pin,GPIO_PIN_RESET);
HAL_Delay(300);
HAL_GPIO_WritePin(GPIOC,led_Pin,GPIO_PIN_SET);
	


KEY_Init();
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	


	OLED_Init();
	OLED_ShowString(0,0,"Mode: ",OLED_8X16);
			OLED_ShowString(8,16,"AngleXoY:",OLED_8X16);
		OLED_ShowString(8,32,"AngleZ:",OLED_8X16);
		OLED_Update();
		changeMark();
		angle[0]=0;
		angle[1]=0;//初始化为0°

	HAL_UART_Receive_IT(&huart2,&rxBuffer[rxIndex],1);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		KeyCode = KEY_FIFO_Get();	
		if (KeyCode != KEY_NONE)
		{
			switch (KeyCode)
			{
				case KEY_DOWN_K1:			
				if(isBT==1){}
					else if(--angle[p]>180){angle[p]=0;}
					changeMark();
					break;
				case KEY_DOWN_K2:	
					p=!p;
				changeMark();
					break;
				case KEY_2_LONG:
				if(isBT==1){isBT=0;}else{isBT=1;
					}
				changeMark();
					break;
				case KEY_DOWN_K3:
if(isBT==1){
}else if(++angle[p]>180){angle[p]=180;}
changeMark();
					break;								
				default:
					break;
			}
		}
		
		if(isBT==0){duty[0]=(angle[0]/ 18.0 + 2.5)*20;
		duty[1]=(angle[1]/18.0+2.5)*20;}
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t)duty[0]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (uint32_t)duty[1]);
		
				OLED_ShowNum(80,16,(uint32_t)angle[0],3,OLED_8X16);
		OLED_ShowNum(80,32,(uint32_t)angle[1],3,OLED_8X16);
		OLED_UpdateArea(80,16,24,32);
		
		



		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
