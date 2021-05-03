/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "stm32f0xx.h"
#include "stdlib.h"
#include "stdio.h"

#define timer_freq 48.0  //timer clock freq in MHz
#define T0H 0.45  //each different clone can have their own timings
#define T1H 1.8 //timing here are in us
#define T0L 1.8
#define T1L 0.45
#define Treset 50

uint8_t LED_data[150];
uint8_t LED_data_temp[3];

uint16_t pos;
uint8_t mask = 0x80;
uint8_t lastbit;

long double period;
uint16_t low_CCR1, low_ARR, high_CCR1, high_ARR, treset_ARR;

volatile char a;
volatile char r;
volatile uint32_t flag;

void init_uart(char character)
{
	GPIOC->ODR ^= ((1 << 8) | (1 << 9));
	
	while (!(USART3->ISR & (1 << 7)))
	{
		
	}
	
	USART3->TDR = character;
	GPIOC->ODR ^= ((1 << 8) | (1 << 9));
}


void init_uart2(char ch[])
{
	int b = 0;
	
	while (ch[b]!= 0)
	{
		init_uart(ch[b]);
		b++;
	}
}

void USART3_4_IRQHandler(void)
{
	if ((USART3->ISR & (1 << 5)))
	{
		a = USART3->RDR;
		if ( a == 'r')
		{
			init_uart2("Red ");
			for (uint8_t i = 0; i < 147; i+=3){
  	    LED_data[147-i] = LED_data[147-3-i];  
	  		LED_data[148-i] = LED_data[148-3-i];
	  		LED_data[149-i] = LED_data[149-3-i];
	    }
			LED_data[0] = 255;
			LED_data[1] = 0;
			LED_data[2] = 0;
			
		}
		else if (a == 'b')
		{
			init_uart2("Blue ");
			for (uint8_t i = 0; i < 147; i+=3){
  	    LED_data[147-i] = LED_data[147-3-i];  
	  		LED_data[148-i] = LED_data[148-3-i];
	  		LED_data[149-i] = LED_data[149-3-i];
	    }
			LED_data[0] = 0;
			LED_data[1] = 0;
			LED_data[2] = 255;
		}
		else if (a == 'p')
		{
			init_uart2("Purple ");
			for (uint8_t i = 0; i < 147; i+=3){
  	    LED_data[147-i] = LED_data[147-3-i];  
	  		LED_data[148-i] = LED_data[148-3-i];
	  		LED_data[149-i] = LED_data[149-3-i];
	    }
			LED_data[0] = 255;
			LED_data[1] = 0;
			LED_data[2] = 255;
		}
		else if (a == 'g')
		{
			init_uart2("Green ");
			for (uint8_t i = 0; i < 147; i+=3){
  	    LED_data[147-i] = LED_data[147-3-i];  
	  		LED_data[148-i] = LED_data[148-3-i];
	  		LED_data[149-i] = LED_data[149-3-i];
	    }
			LED_data[0] = 0;
			LED_data[1] = 255;
			LED_data[2] = 0;
		}
		else if (a == 'y')
		{
			init_uart2("Yellow ");
			for (uint8_t i = 0; i < 147; i+=3){
  	    LED_data[147-i] = LED_data[147-3-i];  
	  		LED_data[148-i] = LED_data[148-3-i];
	  		LED_data[149-i] = LED_data[149-3-i];
	    }
			LED_data[0] = 255;
			LED_data[1] = 255;
			LED_data[2] = 0;
		}
		else if (a == 'c')
		{
			init_uart2("Cyan ");
			for (uint8_t i = 0; i < 147; i+=3){
  	    LED_data[147-i] = LED_data[147-3-i];  
	  		LED_data[148-i] = LED_data[148-3-i];
	  		LED_data[149-i] = LED_data[149-3-i];
	    }
			LED_data[0] = 0;
			LED_data[1] = 255;
			LED_data[2] = 255;
		}
		
		flag = 1;
		HAL_Delay(50);
		//init_uart2("Flag Set");
	}
}


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void led_init(void)
{
	
	//calculate all the timings.
	period = 1 / timer_freq;
	low_CCR1 = round(T0H / period);
	low_ARR = round((T0H + T0L) / period);
	high_CCR1 = round(T1H / period);
	high_ARR = round((T1H + T1L) / period);
	treset_ARR = ceil(Treset / period);
	
	// Set up pin PA4 for H-bridge PWM output (TIMER 14 CH1)    
	GPIOA->MODER |= (1 << 9);    
	GPIOA->MODER &= ~(1 << 8);
	
	// Set PA4 to AF4,    
	GPIOA->AFR[0] &= 0xFFF0FFFF; // clear PA4 bits,    
	GPIOA->AFR[0] |= (1 << 18);
	
	// Set up PWM timer    
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;    
	TIM14->CR1 = 0;                         // Clear control registers    
	TIM14->CCMR1 = 0;                       // (prevents having to manually clear bits)    
	TIM14->CCER = 0;    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer    
	TIM14->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);    
	TIM14->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1    
	TIM14->PSC = 0;                         // Run timer on 48Mhz    
	TIM14->ARR = treset_ARR;                      // PWM at 20kHz    
	TIM14->CCR1 = 0;                        // pin starts low
  TIM14->CR1 |= TIM_CR1_CEN;              // Enable timer
	TIM14->CR1 |= TIM_CR1_ARPE;
	TIM14->DIER &= ~TIM_DIER_UIE;
	
	NVIC_EnableIRQ(TIM14_IRQn); // Enable interrupt(NVIC level)
	NVIC_SetPriority(TIM14_IRQn, 3);
}

void show_led()
{
	pos = 0;
	lastbit = 0;
	mask = 0x80;
	TIM14->SR &= ~TIM_SR_UIF;
	TIM14->DIER |= TIM_DIER_UIE;
}
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
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

	/* Configure the system clock */
  SystemClock_Config();

	RCC->AHBENR	|= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOBEN);
	
	GPIOB->AFR[1] |= ((1 << 10) | (1 << 14));
	GPIOB->AFR[1] &= ~((1 << 8) | (1 << 9) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 15));
	GPIOB->MODER |= ((1 << 21) | (1 << 23));
	GPIOB-> MODER &= ~((1 << 20) | (1 << 22));
	
	RCC->APB1ENR |= (RCC_APB1ENR_USART3EN);
	
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;
	
	USART3->CR1 |= ((1 << 2) | (1 << 3) | (1 << 5));
	
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 3);
	
	USART3->CR1 |= (1 << 0);
	init_uart2("LED?");
	
	GPIOC->MODER |= ((1 << 12)|(1 << 14));
	GPIOC->OTYPER &= ~((1 << 6)|(1 << 7));
	GPIOC->OSPEEDR &= ~((1 << 12)|(1 << 14));
	GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15));
	
	GPIOC->ODR |= ((1 << 6) | (1 << 7));
	
	//fill the array with repeate pattern of Green-Red-Blue
	for (uint8_t i = 0; i < 150; i+=9){
	  LED_data[i] = 255;  //use low values so that it does blind the camera
	  LED_data[i+4] = 255;
	  LED_data[i+8] = 255;
	}

	led_init();
	show_led();
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		//GPIOC->ODR ^= (1 << 6);
		
		//HAL_Delay(1000);
		if(flag == 1)
		{
			flag = 0;
			//init_uart2("Flag Cleared");
			show_led();	
			//init_uart2("Test");
		}
//		LED_data_temp[0] = LED_data[0];
//		LED_data_temp[1] = LED_data[1];
//		LED_data_temp[2] = LED_data[2];
//		for (uint8_t i = 0; i < 147; i+=3){
//	    LED_data[i] = LED_data[i+3];  //use low values so that it does blind the camera
//			LED_data[i+1] = LED_data[i+4];
//			LED_data[i+2] = LED_data[i+5];
//	  }
//		LED_data[147] = LED_data_temp[0];
//		LED_data[148] = LED_data_temp[1];
//		LED_data[149] = LED_data_temp[2];
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void TIM14_IRQHandler(void)
{
	TIM14->SR &= ~TIM_SR_UIF;
	GPIOC->ODR ^= (1 << 7);
	
	if(pos<sizeof(LED_data)){
			if(LED_data[pos] & mask){
				TIM14->CCR1 = high_CCR1;
				TIM14->ARR = high_ARR;
			}else{
				TIM14->CCR1 = low_CCR1;
				TIM14->ARR = low_ARR;
			}
			if(mask==1){
				mask = 0x80;
				pos+=1;
			}else mask = mask >> 1;
		}else{
			TIM14->CCR1 = 0; //set to zero so that pin stays low
			TIM14->ARR = treset_ARR; //set to timing for reset LEDs
			TIM14->DIER &= ~TIM_DIER_UIE; //disable interrupt flag to end transmission.
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
