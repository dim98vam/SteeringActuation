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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CAN_ID 0x00000011 //dummy define id for receiving messages

#define THRESHOLD  72                   //adc units for error checking
#define SAME_POSITION_CHECKING 10       //adc values threashold to ignore new position to avoid stuttering
#define ADC_Amplification 1 //3.0f/2    //define the software amplification needed to produce correct adc value in volts before voltage division
#define ADC_UnitsPerStep  2             //adc units per step - used in threshold calculation

#define STEP_PERIOD  625                //defines the minimum period of stepping pulse - timer counter is updated every 1us
#define NumOfSTEPS   35                 //defines number of steps during acceleration/deceleration phase
#define STEP_INITIAL_PERIOD 15002       //defines the initial timer period

#define TRANSMIT 4000                   //defines double the milliseconds between transmitions of motor position
                                        //through the CAN bus

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

//----------------------------------Can Variables---------------------------------------------
CAN_TxHeaderTypeDef   TxHeader; //headers to transmit and receive from can bus
CAN_RxHeaderTypeDef   RxHeader;

uint32_t TxMailbox; //variable to hold which mailbox was used when transmiting

uint8_t               TxData[8];           //matrices to hold transmitting and receiving data
uint8_t               RxData[8];


//-------------------------variables to control motor movement--------------------------------
volatile uint16_t     adcConvertedValue;   //variable to hold adc converted value
																								
volatile uint16_t     wheelReqPos;         //req position of the wheel

uint8_t               direction;           //direction of turn - used to write the direction pin
int                   temp;                //used to get the absolute value of error
int                   error;               //helping variable
uint8_t               activeThreshold;     //holds the active threshold for current movement


uint8_t               slowing=0;           //flag indicating whether motor is slowing down - zero initialized
volatile uint8_t      timerInactive=1;     //indicating motor state - 1 initialized

volatile uint8_t      newReqPosReceived=0; //initialize newReqPosReceived variable

                  
volatile int          increment;           //multiplier to control whether pwm timer increases or decreases pulse period
                                           //used to avoid if checking
volatile int          matrixHolder=0;      //matrix holder used to dereference lookup table

//used lookup table for speed control
volatile uint32_t     lookUpTable[36]={15002, 7896, 4156, 2187, 1151, 1042, 994, 954, 919, 889, 863, 840, 820, 802, 786, 772, 759, 
                                     748, 738, 728, 720, 712, 705, 699, 693, 687, 678, 670, 663, 658, 651, 645, 640, 634, 629,625};
                                                   
                                                                                                                
volatile uint16_t     adcData;             //adc converted values


//--------------variables to monitor communication timeout and impossible wheel movement-------------------																		 
volatile uint16_t     counter=0;           //initialize timeoutCounter and TransmitCounter and  timeOutOccured flag
volatile uint16_t     counterTimeout=0;
volatile uint8_t      timeOutOccured=0;

                                           
volatile uint8_t      outOfRangeValue=0;   //check whether requested value was out of range 
uint8_t               stop=0;              //flag to stop execution if error occured


//----------------------------------------debug variables--------------------------------------------------
uint16_t              transmitData[4]={600,1200,1700,2700}; //debug transmit data
volatile uint8_t      position=0;                           //debugging variable to request new position from the motor
volatile int          decrement=1;                          //debbugging variable to select next position



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
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
  SysTick_Config(SystemCoreClock/10000*5); //initialize systick function for systick to interrupt every 500us
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	__enable_irq(); //enable interrupts on mcu core
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (!stop)  //many ifs used but work is done on timer peripherals so actual mechanical movement won't slow down - plus while loop takes
		             //ns to execute
  {
    
    if(newReqPosReceived) //if a new wheel position was requested  
		{
			newReqPosReceived=0; //acknowledge new position has been received and update error
			error= (int)(adcConvertedValue-wheelReqPos);
			
			//determine direction of rotation - connection dependent so changes according to the wiring (change the SET/RESET values)
			if(error&0x80000000)
			{ 	
				temp=-1;
				direction=GPIO_PIN_RESET;
			}
			else
			{				
				temp=1;
				direction=GPIO_PIN_SET;
			}
			
			if(error>=140) //if full speedup or slow down is possible - determined by adc units corresponding to full speedup+slowdown phase
				activeThreshold=THRESHOLD;
			else
				activeThreshold=(error*temp)>>1; //half steps for speedup half for slow down
		
			
			if(timerInactive)  //determine actions needed according to motor state - motor is turning/not turning(timerInactive=0/1)
			{
				
			  if(temp*error>SAME_POSITION_CHECKING) //checked here to avoid check in else statement
			  {	
			    timerInactive=0;
				  HAL_GPIO_WritePin(Direction_Of_Rotation_GPIO_Port,Direction_Of_Rotation_Pin,direction); //write direction port immediately
				  increment=1;  //indicate to the motor it has to accelerate
				
				  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_1); //start counter to produce output pulse
			  }
			}
			else    //timer was active - motor is spinning
      { 
				
				if(direction!=HAL_GPIO_ReadPin(Direction_Of_Rotation_GPIO_Port,Direction_Of_Rotation_Pin))  //if direction of rotation has to change
 				{	 
					increment=-1;   //deccelerate				
					slowing=1;
				}
				else  //if direction of rotation is the same
				{
   				increment=1;   //indicate to the motor it has to keep accelerating - if at full speed nothing happens
					slowing=0; //indicate slow down process stopped
				}
				
				__HAL_TIM_ENABLE_IT(&htim2,TIM_IT_CC1); //make sure timer and interrupt are working to avoid deadlock  
				__HAL_TIM_ENABLE(&htim2);               //if at full speed it will get disabled again
				
			}	
		}
		
		
		error=(int)adcConvertedValue-wheelReqPos;  //update error with running wheel position(may be the same as before)
			
			
		
		if((error*temp)<activeThreshold && !timerInactive && !slowing) //if error dropped below threshold and a movement was being made(timer is active)
    {                                                              //and no slow down process is active indicate timer to slow down
			 increment=-1; 
       slowing=1;
       
			__HAL_TIM_ENABLE_IT(&htim2,TIM_IT_CC1); //reenable interrupts in case motor has reached full speed and disabled them			
		}
       
		
		if(slowing && timerInactive) //if slowing down process completed(flag still not updated)
		{
			slowing=0; //update flag
			
			if(HAL_GPIO_ReadPin(Direction_Of_Rotation_GPIO_Port,Direction_Of_Rotation_Pin) != direction) //if change of direction needed
			{	
			   HAL_GPIO_WritePin(Direction_Of_Rotation_GPIO_Port,Direction_Of_Rotation_Pin,direction); //update direction pin
				 increment=1; //indicate to timer it has to accelerate motor
				 timerInactive=0; //indicate timer has started
				 
				 __HAL_TIM_ENABLE_IT(&htim2,TIM_IT_CC1); //start counter to produce output pulse
				 __HAL_TIM_ENABLE(&htim2);
			}
		}
		
		
		if(timeOutOccured || outOfRangeValue) //if a critical errorOccured
		{
			HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_1); //deactivate timer immediately/stop wheel movement and send CAN error message
			
			TxData[0] = 0xff; //populate data with error value
	    TxData[1]=  0xff;
			
			if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) //transmit data
      {
        // Transmission request Error 
        Error_Handler();
      }
			
			stop=0;         //update stop flag and disable irq
			__disable_irq();
			
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
	CAN_FilterTypeDef  sFilterConfig; //variable to hold filterConfiguration for receiving can messages

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	
	//initializing can filter
	//filter code used is dummy and stdId mode is used
	
	sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh = CAN_ID<<5;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
	
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
	
	//activate can peripheral
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
	
	//add canReceiveNotification
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }
	
	//initialize can transmit parameter
	TxHeader.StdId = CAN_ID;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 2;
  TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END CAN1_Init 2 */

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
  htim2.Init.Prescaler = 90;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = STEP_INITIAL_PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Step_Signal_Pin|Direction_Of_Rotation_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Step_Signal_Pin */
  GPIO_InitStruct.Pin = Step_Signal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(Step_Signal_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Direction_Of_Rotation_Pin */
  GPIO_InitStruct.Pin = Direction_Of_Rotation_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Direction_Of_Rotation_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//can receive callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  counterTimeout=0; //reset communication timeout condition
	
	/* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  /* Display LEDx */
  if ((RxHeader.StdId == CAN_ID) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 2))
  {
     wheelReqPos= 0xff00 & ((uint16_t)RxData[0] << 8); //get the first and second byte of data received from can bus
		 wheelReqPos= wheelReqPos | ((uint16_t)RxData[1]); //value is considered already amplified
  }
	
	if(wheelReqPos>4095)  //check whether requested value is valid - since unsigned no negative checking is needed
		outOfRangeValue=1;
	else
	  newReqPosReceived=1; //indicate new position request was made
	
}


//adc calling function
void ADC_Handle()
{
	if (HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &adcData,1) != HAL_OK) //start conversion with interrupt
  {
    /* Start Conversation Error */
    Error_Handler();
  }
}


//timer initerrupt handler - counter interrupts every 100 us
//seems large handler but bulk of statements is executed only once plus it just manipulates registers which is fast
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim2){  //callback function for timBase interrupt
  
	                                      
		if(__HAL_TIM_GetAutoreload(htim2)==STEP_PERIOD && increment==1) //if minimum period has been reached and acceleration was requested
		{
			__HAL_TIM_DISABLE_IT(htim2,TIM_IT_CC1);    //disable interrupts(not needed anymore) and set period
			__HAL_TIM_CLEAR_IT(htim2,TIM_IT_CC1);
			
			matrixHolder=NumOfSTEPS-1; //update matrixHolder for future interruts - points to the immediately greater counter
			
		}
		else if(matrixHolder==-1) //if maximum period(slowing down) has been reached and executed(thus the -1)
		{
			HAL_TIM_PWM_Stop_IT(htim2,TIM_CHANNEL_1);  //stop pwm generation
			
			timerInactive=1;                           //indicate timer stopped
			matrixHolder=1;                            //reinitialize matrixHolder to 1 because initial value is already programmed in
			
			//reinitialize timer counter
			__HAL_TIM_SET_COUNTER(htim2,0);   
		}
    else
		{
			__HAL_TIM_SET_AUTORELOAD(htim2,(uint32_t)lookUpTable[matrixHolder]); 	//set new period
			matrixHolder+=increment; //update matrixHolder 
		}
		
}


//adc interrupt handler
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  
	HAL_GPIO_TogglePin(Step_Signal_GPIO_Port,Step_Signal_Pin); //used for debugging
	
	// Get the converted value of regular channel 0 
  adcConvertedValue = (uint16_t)(adcData * ADC_Amplification); //get current position of the wheel amplified to account for
	                                                             //the voltage divider
	
	if(counter==TRANSMIT)
	{	
	  counter=0; //update transmit variable
		
		TxData[0] = (transmitData[position] >> 8) & 0x0f; //give TxData the values to transmit - value is transmitted
	  TxData[1]=  (transmitData[position] & 0xff);      //as an unsigned adc value to aid with the transmission - value should be transformed
	                                                    //by the receiver
		
		/*debugging stage*/
		position+=decrement;
		
		if(position==3)
			decrement=-1;
		else if(position==0)
			decrement=1;
		
	
	  
	  if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) //transmit data
      {
        // Transmission request Error 
        Error_Handler();
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
