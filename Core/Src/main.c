/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t Rx_size = 64;
uint8_t Rx_buffer[64];
uint8_t system_work_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	
	 //HAL_TIM_Base_Start_IT(&htim4);
	
	HAL_UART_Receive_DMA(&huart1, Rx_buffer, Rx_size);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	Servo_SetAngleX(0);
	Servo_SetAngleY(0);
	HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		
		
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        // 1. 如果当前系统允许执行舵机控制，则照常控制
        if (system_work_state == 1)
        {
            control_servo();
        }
        
        // 2. 异步按键状态机扫描（完全不阻塞，不使用 HAL_Delay）
        // 支持 PB4 和 PB5 独立或共同控制
        static uint8_t key_state = 0; // 状态机变量：0-等待按下, 1-消抖与松开检测
        
        // 读取引脚状态：只要有一个按下即为低电平 (RESET)
        uint8_t pin_b4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
        uint8_t pin_b5 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
        
        switch (key_state)
        {
            case 0:
                // 如果检测到任意按键被按下
                if (pin_b4 == GPIO_PIN_RESET || pin_b5 == GPIO_PIN_RESET)
                {
                    key_state = 1; // 切换到消抖与等待松开状态
                }
                break;
                
            case 1:
                // 必须等到所有按键都松开（恢复为高电平 SET）
                if (pin_b4 == GPIO_PIN_SET && pin_b5 == GPIO_PIN_SET)
                {
                    key_state = 0; // 恢复初始状态，准备下一次动作
                    
                    /* 执行开关翻转控制（松开时触发） */
                    if (system_work_state == 1)
                    {
                        system_work_state = 0;
                        
                        // 停止 UART 接收（不再响应视觉）
                        HAL_UART_DMAStop(&huart1);
                        __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
                        
                        // 注意：这里【不要】调用 HAL_TIM_Base_Stop_IT(&htim4) 
                        // 因为我们正处在 TIM4 中断里，如果把它关了，下次按键就无法开机了。
                        // 我们通过上面 if(system_work_state == 1) 屏蔽控制逻辑来实现“关闭效果”。
                    }
                    else
                    {
                        system_work_state = 1;
                        
                        // 重新启动 UART 的 DMA 接收与空闲中断
                        HAL_UART_Receive_DMA(&huart1, Rx_buffer, Rx_size);
                        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
                    }
                }
                break;
                
            default:
                key_state = 0;
                break;
        }
    }
}
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
#ifdef USE_FULL_ASSERT
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
