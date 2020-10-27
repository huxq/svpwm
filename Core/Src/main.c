/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// utility defines
#define _2_SQRT3 1.15470053838
#define _SQRT3 1.73205080757
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038

// sign function
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define _round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int gpio_b12 = 0;
int gpio_b13 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#define SIN_STEPS 1024
#define SIN_STEPS 1024
char buf[200];
uint16_t sin_array[SIN_STEPS];
#define SIN_SCALE 780 //400-700should be fine


uint16_t duty_u=0;
uint16_t duty_v=0;
uint16_t duty_w=0;



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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  //HAL_Delay(1);
  //HAL_TIM_Base_Start_IT(&htim2);


  HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_1);


  HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_2);

  HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_3);

  /* Init ADC1 */
  //MX_ADC1_Init();

  for(int i=0;i<SIN_STEPS;i++)
  {
	  float value =  (i*M_TWOPI/SIN_STEPS);
      sin_array[i]= (uint16_t)round((sinf(value)+1)*SIN_SCALE)+5;
//      sprintf(buf,"Sin %d\t\n",sin_array[i]);
//      HAL_UART_Transmit(&huart2,buf,strlen(buf),10);
  }

  // int array instead of float array
  // 2x storage save (int 2Byte float 4 Byte )
  // sin*10000
  int sine_array[200] = {0,79,158,237,316,395,473,552,631,710,789,867,946,1024,1103,1181,1260,1338,1416,1494,1572,1650,1728,1806,1883,1961,2038,2115,2192,2269,2346,2423,2499,2575,2652,2728,2804,2879,2955,3030,3105,3180,3255,3329,3404,3478,3552,3625,3699,3772,3845,3918,3990,4063,4135,4206,4278,4349,4420,4491,4561,4631,4701,4770,4840,4909,4977,5046,5113,5181,5249,5316,5382,5449,5515,5580,5646,5711,5775,5839,5903,5967,6030,6093,6155,6217,6279,6340,6401,6461,6521,6581,6640,6699,6758,6815,6873,6930,6987,7043,7099,7154,7209,7264,7318,7371,7424,7477,7529,7581,7632,7683,7733,7783,7832,7881,7930,7977,8025,8072,8118,8164,8209,8254,8298,8342,8385,8428,8470,8512,8553,8594,8634,8673,8712,8751,8789,8826,8863,8899,8935,8970,9005,9039,9072,9105,9138,9169,9201,9231,9261,9291,9320,9348,9376,9403,9429,9455,9481,9506,9530,9554,9577,9599,9621,9642,9663,9683,9702,9721,9739,9757,9774,9790,9806,9821,9836,9850,9863,9876,9888,9899,9910,9920,9930,9939,9947,9955,9962,9969,9975,9980,9985,9989,9992,9995,9997,9999,10000,10000};

  // function approximating the sine calculation by using fixed size array
  // ~40us (float array)
  // ~50us (int array)
  // precision +-0.005
  // it has to receive an angle in between 0 and 2PI
  float _sin(float a){
    if(a < _PI_2){
      //return sine_array[(int)(199.0*( a / (_PI/2.0)))];
      //return sine_array[(int)(126.6873* a)];           // float array optimized
      return 0.0001*sine_array[_round(126.6873* a)];      // int array optimized
    }else if(a < _PI){
      // return sine_array[(int)(199.0*(1.0 - (a-_PI/2.0) / (_PI/2.0)))];
      //return sine_array[398 - (int)(126.6873*a)];          // float array optimized
      return 0.0001*sine_array[398 - _round(126.6873*a)];     // int array optimized
    }else if(a < _3PI_2){
      // return -sine_array[(int)(199.0*((a - _PI) / (_PI/2.0)))];
      //return -sine_array[-398 + (int)(126.6873*a)];           // float array optimized
      return -0.0001*sine_array[-398 + _round(126.6873*a)];      // int array optimized
    } else {
      // return -sine_array[(int)(199.0*(1.0 - (a - 3*_PI/2) / (_PI/2.0)))];
      //return -sine_array[796 - (int)(126.6873*a)];           // float array optimized
      return -0.0001*sine_array[796 - _round(126.6873*a)];      // int array optimized
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int count = 0;
  int angle_el = 0;
  float angle_r= 0;
  int steps = 0;
  int direction = 1;

  while (1)
  {
	     // we have 6 sectors for svpwm

	     angle_r = (((angle_el%SIN_STEPS)*M_TWOPI/SIN_STEPS));
	     int sector = floor(angle_r / _PI_3) + 1;

	     float T1 = _SQRT3*_sin(sector*_PI_3 - angle_r);
	     float T2 = _SQRT3*_sin(angle_r - (sector-1.0)*_PI_3);
	      // two versions possible
	      // centered around voltage_power_supply/2
	     float T0 = 1 - T1 - T2;

	     float duty_u_f,duty_v_f,duty_w_f;



	      switch(sector){
	        case 1:
	          duty_u_f = T1 + T2 + T0/2;
	          duty_v_f = T2 + T0/2;
	          duty_w_f = T0/2;
	          break;
	        case 2:
	          duty_u_f = T1 +  T0/2;
	          duty_v_f = T1 + T2 + T0/2;
	          duty_w_f = T0/2;
	          break;
	        case 3:
	          duty_u_f = T0/2;
	          duty_v_f = T1 + T2 + T0/2;
	          duty_w_f = T2 + T0/2;
	          break;
	        case 4:
	          duty_u_f = T0/2;
	          duty_v_f = T1+ T0/2;
	          duty_w_f = T1 + T2 + T0/2;
	          break;
	        case 5:
	          duty_u_f = T2 + T0/2;
	          duty_v_f = T0/2;
	          duty_w_f = T1 + T2 + T0/2;
	          break;
	        case 6:
	          duty_u_f = T1 + T2 + T0/2;
	          duty_v_f = T0/2;
	          duty_w_f = T1 + T0/2;
	          break;
	        default:
	         // possible error state
	          duty_u_f = 0;
	          duty_v_f = 0;
	          duty_w_f = 0;
	      }
	      duty_u = (duty_u_f+0.4)*900;
	      duty_v = (duty_v_f+0.4)*900;
	      duty_w = (duty_w_f+0.4)*900;

//	     sprintf(buf,"U:%d U:%d U:%d U:%d U:%f\t\n",sector, duty_u,duty_v,duty_w,angle_r);
//	     HAL_UART_Transmit(&huart2,buf,strlen(buf),10);
	   	gpio_b12 = !gpio_b12;
	   	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, gpio_b12);

	    angle_el=angle_el+8;


//	     if(angle_el>=1024)
//	     {
//	    	 angle_el=2048;
//	     }

//		 if(steps%2)
//		 {
//			 count=0;
//		 }
//		 else
//		 {
//			 count=500;
//		 }



	    // HAL_Delay(1);

//	     count=count+1;
//
//	     sin(count);
//	     sin(count);
//	     sin(count);
//	     sin(count);
//	     sin(count);

//	     if(count%5==0)
//	     {
//		     HAL_ADC_Start(&hadc1);
//		     HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//
//		     uint32_t raw = HAL_ADC_GetValue(&hadc1);
//
//
//		      sprintf(buf,"ADC %d\t\n",raw);
//		      HAL_UART_Transmit(&huart2,buf,strlen(buf),10);
//	     }
//
//		 if((count%(SIN_STEPS-1))==0)
//		 {
//			 //count=0;
//			 //steps++;
//			 //continue;
//		 }
//
//		 if(count>=(SIN_STEPS-1)*2)
//		 {
//			 //steps = 100;
//			 //direction = !direction;
//			 direction = !direction;
//			 count = ((SIN_STEPS-1)*2-1);
//			 HAL_Delay(200);
//		 }
//		 else
//		 {
//			 if(direction)
//			 {
//				 count = count+1;
//			 }
//			 else
//			 {
//				 count = count-1;
//				 if(count<=0)
//				 {
//					 direction = !direction;
//					 count=0;
//					 HAL_Delay(200);
//				 }
//			 }
//
//		 }
//





//		 if(out>=3000)
//		 {
//			 out=0;
//		 }
//
//		 if(out%2)
//		 {
//			 count = 0;
//		 }
//		 else
//		 {
//			 count = 400;
//		 }
//		 out++;


//		 duty_u=sin_array[(count%SIN_STEPS)];
//		 duty_v=sin_array[(count+SIN_STEPS/3)%SIN_STEPS];
//		 duty_w=sin_array[(count+2*SIN_STEPS/3)%SIN_STEPS];
		 {
//			 if(count%2)
//			 {
//				 TIM1->CCR1=0;
//			 }
//			 else
//			 {
//
//				 TIM1->CCR1=25;
//			 }
//			 TIM1->CCR1=duty_u;
//			 TIM1->CCR2=duty_v;
//			 TIM1->CCR3=duty_w;
//
//			 count=count+8;
			 //HAL_Delay(5);
//			 steps++;
//			 if((steps>=1000)&& (steps<1500))
//			 {
//				 count=count+4;
//				 steps++;
//				 HAL_Delay(3);
//
//			 }
//			 else if((steps>=1500)&&(steps<2600))
//			 {
//				 count=count+4;
//				 steps++;
//				 HAL_Delay(2);
//
//			 }
//			 else if(steps>=2600)
//		     {
//				 count=count+8;
//			     steps=2900;
//				 HAL_Delay(2);
//
//			}
//			 else
//			 {
//				 count=count+4;
//				 HAL_Delay(4);
//			 }

		 }


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1600;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 500;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1300;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 32;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{



//	// Only process the time up event for channel 1
//	switch(htim->Channel)
//	{
//	    case HAL_TIM_ACTIVE_CHANNEL_1:
//	    	gpio_b12 = !gpio_b12;
//	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, gpio_b12);
//	    break;
//	    case HAL_TIM_ACTIVE_CHANNEL_2:
//	    	gpio_b13 = !gpio_b13;
//	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, gpio_b13);
//
//	    break;
//	    case HAL_TIM_ACTIVE_CHANNEL_3:
//	    break;
//	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
