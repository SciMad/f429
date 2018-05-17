
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  /* USER CODE BEGIN 2 */


//	--------------------------------------------------------------------------------------------------------------------------
//	To send digital (HIGH / LOW) to a particular pin of particular port using ODR bits
//	--------------------------------------------------------------------------------------------------------------------------
//	volatile uint32_t *GPIOG_MODER = 0x0, *GPIOG_ODR = 0x0;
//	MX_GPIO_Init();
//	GPIOG_MODER = (uint32_t*)GPIOG_BASE;			//	Address of the GPIOG->MODER register
//	GPIOG_ODR = (uint32_t*)(GPIOG_BASE + 0x14); 	//	Address of the GPIOG->ODR register. This ensure that the peripheral is enabled and connected to the AHB1 bus
//	*GPIOG_MODER = *GPIOG_MODER | 0x4000000; 		//	Sets MODER[27:26] = 0x1
//	while(1){
//		*GPIOG_ODR = *GPIOG_ODR | 0x2000;				// To turn off *GPIOG_ODR = *GPIOG_ODR & ~ 0x2000;
//		HAL_Delay(500);
//		*GPIOG_ODR = *GPIOG_ODR & ~0x2000;				// To turn off *GPIOG_ODR = *GPIOG_ODR & ~ 0x2000;
//		HAL_Delay(500);
//	}



//	--------------------------------------------------------------------------------------------------------------------------
//	This is to toggle digital data in a pin 13 of port G using HAL Toggle function
//	--------------------------------------------------------------------------------------------------------------------------
//  volatile uint32_t *GPIOG_MODER = 0x0, GPIOG_ODR = (uint32_t*)(GPIOG_BASE + 0x14);
//  MX_GPIO_Init();
//  GPIOG_MODER = (uint32_t*)GPIOG_BASE;			//	Address of the GPIOG->MODER register
//	GPIOG_ODR = (uint32_t*)(GPIOG_BASE + 0x14); 	//	Address of the GPIOG->ODR register. This ensure that the peripheral is enabled and connected to the AHB1 bus
//  *GPIOG_MODER = *GPIOG_MODER | 0x4000000; 		//	Sets MODER[2n+1:2n] = 0x1
//
//  while(1) {
//	  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
//	  HAL_Delay(500);
//  }
//

  	MX_GPIO_Init();
	volatile uint32_t *GPIOG_MODER = 0x0, *GPIOG_ODR = 0x0;

	GPIOG_MODER = (uint32_t*)GPIOG_BASE;							//	Address of the GPIOG->MODER register
	GPIOG_ODR = (uint32_t*)(GPIOG_BASE + 0x14); 					//	Address of the GPIOG->ODR register. This ensure that the peripheral is enabled and connected to the AHB1 bus
	*GPIOG_MODER = *GPIOG_MODER | 0x4000000|(0x4000000<<2); 		//	Sets MODER[27:26] = 0x1


	volatile uint32_t *GPIOA_MODER = 0x0, *GPIOA_IDR = 0x0;
	GPIOA_MODER = (uint32_t*)GPIOA_BASE;			//	Address of the GPIOA->MODER register
	GPIOA_IDR = (uint32_t*)(GPIOA_BASE + 0x10); 	//	Address of the GPIOA->IDR register. This ensure that the peripheral is enabled and connected to the AHB1 bus
	*GPIOA_MODER = *GPIOA_MODER & 0x0; 				//	Sets MODER[1:0] = 0x1

	while(1){

		if (*GPIOA_IDR & 0x01){
			*GPIOG_ODR = *GPIOG_ODR | 0x2000;				// To turn off *GPIOG_ODR = *GPIOG_ODR & ~ 0x2000;
			*GPIOG_ODR = *GPIOG_ODR & ~0x4000;				// To turn off *GPIOG_ODR = *GPIOG_ODR & ~ 0x2000;

		}else{
			*GPIOG_ODR = *GPIOG_ODR & ~0x2000;				// To turn off *GPIOG_ODR = *GPIOG_ODR & ~ 0x2000;
			*GPIOG_ODR = *GPIOG_ODR | 0x4000;				// To turn off *GPIOG_ODR = *GPIOG_ODR & ~ 0x2000;
		}
	}

//
//	GPIO_TypeDef *PA = 0x48000000;
//	GPIO_InitTypeDef *PA_Init;
//
//	PA_Init->Pin = GPIO_PIN_0;
//	PA_Init->Mode = GPIO_MODE_INPUT;
//
//	HAL_GPIO_Init(PA, PA_Init);




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
	@brief GPIO Configuration
	@retval None
*/


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

static void MX_GPIO_Init(){
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
