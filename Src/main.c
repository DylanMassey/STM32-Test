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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "arm_math.h"
#include "stdbool.h"

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM11_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Set_System_Working(int status);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define BUFFER_ADC_SIZE 100  // 2.5us ADC for 10ms
#define NUM_SAMPLES 100
#define WINDOW_SIZE 500
#define FLAG_POSITION  (1 << 0) // Assuming flag is at bit 0
#define DAC_MID (4095 / 2)
#define TRIGER_TIMES 10 //

#define B1_GPIO_PORT GPIOC             // Button Blue
#define B1_GPIO_PIN  GPIO_PIN_13

#define SYSTEM_WORKING_PIN GPIO_PIN_5  // Trigger out
#define SYSTEM_WORKING_GPIO_PORT GPIOB



float32_t sine_val[NUM_SAMPLES];
float32_t sine_val_five[500];
uint32_t BUFFER_ADC[BUFFER_ADC_SIZE];
uint32_t BUFFER_ADC2[BUFFER_ADC_SIZE];
uint32_t TIMER8_VALUE;
uint32_t adc_value;
float32_t hanning[WINDOW_SIZE];
uint32_t after_hanning[WINDOW_SIZE];
uint32_t sine_val_test[NUM_SAMPLES];
uint32_t sine_val_five_test[500];
uint8_t flag_100ms=0;
uint8_t adc_flag=0;
uint8_t dac_flag=0;
uint32_t adc1_value[BUFFER_ADC_SIZE];
uint32_t adc2_value[BUFFER_ADC_SIZE];
float32_t result[2 * BUFFER_ADC_SIZE - 1];
float32_t adc1_f[BUFFER_ADC_SIZE];
float32_t adc2_f[BUFFER_ADC_SIZE];
uint32_t uarttest[4]={0x12345678,0xaaaaaaaa,0x00000000,0xffffffff};
uint8_t cnt=0;

uint8_t cnt_t=0; // for test



void Set_System_Working(int status) {
  if (status) {
    HAL_GPIO_WritePin(SYSTEM_WORKING_GPIO_PORT, SYSTEM_WORKING_PIN, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(SYSTEM_WORKING_GPIO_PORT, SYSTEM_WORKING_PIN, GPIO_PIN_RESET);
  }
}

void get_sineval()
{
   for (int j=0;j<5;j++)
   {
	    for (int i=0;i<NUM_SAMPLES;i++)
	    {
	    	sine_val[i] = ((sin(i*2*PI/NUM_SAMPLES-1/2*PI)+1)*(4096/2));

	    	sine_val_five[j*100+i]=sine_val[i];

	    	sine_val_five_test[j*100+i]=(uint32_t)(sine_val_five[j*100+i]);
	    }


   }
}

void convertUint32ToFloat32(uint32_t *input, float32_t *output, size_t length)
{
    for (size_t i = 0; i < length; i++) {
        output[i] = (float32_t)input[i];
    }
}

void convertFloat32ToUint32(float32_t *input, uint32_t *output, size_t length, float32_t scale) {
    for (size_t i = 0; i < length; i++) {
        if (input[i] < 0) {
            output[i] = 0;  // Clamping negative values to zero
        } else {
            float32_t temp = input[i] * scale;
            output[i] = (uint32_t)temp;
        }
    }
}

/*
for (int i = 0; i < ARRAY_SIZE; i++) {
       buffer_float32[i] = (float32_t)i / ARRAY_SIZE;  // Example: normalized float values
   }

   // Convert float32 to uint32 with scaling
   convertFloat32ToUint32(buffer_float32, buffer_uint32, ARRAY_SIZE, 4294967295.0f);
*/

/*
void get_sineval_test()
{
   for (int j=0;j<5;j++)
   {
	    for (int i=0;i<NUM_SAMPLES;i++)
	    {
	    	sine_val_test[i] = ((sin(i*2*PI/NUM_SAMPLES-PI/2)+1)*(4096/2));

	    	sine_val_five_test[j*100+i]=sine_val[i];
	    }


   }
}
*/
void applyHanningWindow()
{
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        hanning[i] = 0.5 * (1 - cos(2 * PI * i / (WINDOW_SIZE - 1)));

        after_hanning[i]=(uint32_t)(DAC_MID + (sine_val_five[i] - DAC_MID) * hanning[i]);
    }
}


void computeCrossCorrelation(float32_t* pSrcA, float32_t* pSrcB, uint32_t srcALength, uint32_t srcBLength, float32_t* pDst)
{
    arm_correlate_f32(pSrcA, srcALength, pSrcB, srcBLength, pDst);
}



void SendBufferOverUART(uint32_t* buffer, size_t length) {
    for (size_t i = 0; i < length; i++) {
        uint8_t data[4];
        data[0] = (uint8_t)(buffer[i] >> 24); // Most significant byte
        data[1] = (uint8_t)(buffer[i] >> 16);
        data[2] = (uint8_t)(buffer[i] >> 8);
        data[3] = (uint8_t)(buffer[i]);       // Least significant byte

        // Send each byte over UART
        HAL_UART_Transmit(&huart1, &data[0], 1, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, &data[1], 1, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, &data[2], 1, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, &data[3], 1, HAL_MAX_DELAY);
    }
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
        adc_flag = 1;
		adc_value = BUFFER_ADC[1];

		HAL_TIM_Base_Stop_IT(&htim8);

	}

}
/*
void HAL_ADC_ConvHalfColtCallback(ADC_HandleTypeDef* hadc)
{
	TIMER8_VALUE=__HAL_TIM_GET_COUNTER(&htim8);
	//adc_value = buffer;

}
*/
void EXTI15_10_IRQHandler(void)
{
  // Handle EXTI line interrupt for B1
  HAL_GPIO_EXTI_IRQHandler(B1_GPIO_PIN);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == B1_GPIO_PIN)
  {
	// HAL_TIM_Base_Start_IT(&htim11);  // To start Timer6 which is 100ms
    // Perform action when B1 is pressed
  }
}

void HAL_DMA_ConvCpltCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma->Instance == DMA1_Stream5)
    {
        // Stop DAC
    	cnt_t++;
        HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
        HAL_TIM_Base_Stop_IT(&htim7);

        // Deinitialize DAC and DMA
        HAL_DAC_DeInit(&hdac);
        HAL_DMA_DeInit(hdma);

      // Optionally disable clocks if no further DAC activity is expected
      //  __HAL_RCC_DAC_CLK_DISABLE();
      //  __HAL_RCC_DMA1_CLK_DISABLE();
    }
}

/* TIM11 callback function */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)/* Timer 11 period elapsed */
{
   if (htim->Instance == TIM11)
   {
	   // Optionally disable the 100ms timer interrupt
	   __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);

    	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // To show the status by LED

    	// Set the flag
	   flag_100ms = 1; // Set the corresponding bit to 1

    	if(cnt<TRIGER_TIMES)
    	{


    		HAL_ADC_Start(&hadc1);
  	      HAL_ADC_Start(&hadc2);
  	      HAL_ADCEx_MultiModeStart_DMA(&hadc1, BUFFER_ADC,BUFFER_ADC_SIZE );

    	}

        // Re-enable the 100ms timer interrupt
              __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
    	/* Your code here */
   }
}



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
  MX_DAC_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_TIM11_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  get_sineval();  // To get the 5-cycle sine wave
 // get_sineval_test();
  applyHanningWindow() ;



  //HAL_ADCEx_MultiModeStart_DMA(&hadc1, BUFFER_ADC,BUFFER_ADC_SIZE );
 // arm_hanning_f32(hann_window, WINDOW_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 //HAL_UART_Transmit_DMA(&huart1,  uarttest8, sizeof(uarttest8));
	 //SendBufferOverUART(uarttest, 4);
     // HAL_Delay(100);
	  if (HAL_GPIO_ReadPin(B1_GPIO_PORT, B1_GPIO_PIN) == GPIO_PIN_RESET)
	       {  // Assuming active low
	       // Button is pressed
	          HAL_Delay(200);  // Debounce delay
	          if (HAL_GPIO_ReadPin(B1_GPIO_PORT, B1_GPIO_PIN) == GPIO_PIN_RESET)
	          {
	           HAL_TIM_Base_Start_IT(&htim11);  // To start Timer6 which is 100ms
	           Set_System_Working(1); //  output PB5 to high
	           cnt=0;
	          }
	       }

	  if (flag_100ms==1)
		{
	      // Flag is set

		  flag_100ms = 0; // Clear the flag
		  cnt++;
	      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	     // MX_DAC_Init();
	      HAL_TIM_Base_Start_IT(&htim7);
	      HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, after_hanning, 500, DAC_ALIGN_12B_R);

	      HAL_TIM_Base_Start_IT(&htim8);  //ADC Timer

	    }
	  if(cnt<TRIGER_TIMES)
	  {
	      if(adc_flag)
		      {
			      adc_flag=0;
			      HAL_UART_Transmit_DMA(&huart1, (uint8_t*)BUFFER_ADC, BUFFER_ADC_SIZE*sizeof(uint32_t));
			 // HAL_UART_Transmit_DMA(&huart1, (uint8_t*)uarttest, 16);


		/*  for (uint16_t i = 0; i < BUFFER_ADC_SIZE; i++)
		  		{
		  		adc1_value[i] = BUFFER_ADC[i] & 0x0000ffff;
		  		adc2_value[i] = (BUFFER_ADC[i]&0xffff0000)>>16;
		  		adc1_f[i] = (float32_t)(adc1_value[i]);
		  		adc2_f[i]= (float32_t)(adc2_value[i]);
		  		}

		  computeCrossCorrelation(adc1_f,adc2_f, BUFFER_ADC_SIZE, BUFFER_ADC_SIZE, result);
		  //SendBufferOverUART(BUFFER_ADC, BUFFER_ADC_SIZE);
		   * */

		      }
	  }
	  else
	  {
		  HAL_TIM_Base_Stop_IT(&htim11);
		  Set_System_Working(0);
		 // HAL_UART_DMAPause(&huart1);
	  }

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */
  //hdac.State = HAL_DAC_STATE_RESET;
  //hdac.ErrorCode= 0;

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 21-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 210-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 168-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 50000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Triger_out_GPIO_Port, Triger_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Triger_out_Pin */
  GPIO_InitStruct.Pin = Triger_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Triger_out_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
