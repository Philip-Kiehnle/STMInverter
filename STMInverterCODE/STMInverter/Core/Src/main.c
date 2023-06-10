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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdbool.h>
//#include <stdio.h>
#include <string.h>
#include "controller.h"
#include "sogi_pllFXP.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SHUTDOWN_MASK (1<<6)  // Gatedriver Shutdown Pin + LED (inverted)

enum state_t {INIT, POWER_TEST, GRID_CONNECTING, GRID_SYNC};
volatile enum state_t state = INIT;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t ADCxConvertedData[2];
uint16_t adcOff;
volatile uint16_t debug_Vdc_filt;
volatile int16_t debug_i_ref_amp;
volatile int16_t debug_v_amp_pred;

volatile bool status_Vin = false;
volatile bool status_Vgrid = false;

volatile uint32_t cnt_intr;
volatile uint32_t cnt_crowbar_actions;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Redirect standard output to the UART port
//int _write(int file, char *ptr, int len)
//{
//    HAL_UART_Transmit(&huart1,(uint8_t *)ptr,len, 10);
//    return len;
//}

#define uSend(x) uartSend(x, sizeof x -1)
void uartSend(char* ptr, int len)
{
    HAL_UART_Transmit(&huart1,(uint8_t *)ptr,len, 10);

}

//volatile bool conv_done = false;
//void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)

// 30kHz / ((5+1)/2) = 10kHz
#define CTRL_FREQ 10000
// execution time = 77.19us
// PV voltage applied until first execution of this callback:
// 500µs soft ramp of Buck-Conv + 4.2ms when controller has >3V = 4.7ms
// maximum PV current: 6A -> 6mF * 28V /6A = 28.0ms -> enough time for crowbar to limit Vin

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  GPIOA->BSRR = (1<<5); //set PA5
  //conv_done = true;
  cnt_intr++;

  // Lowpass filter for Vdc
  static uint16_t Vdc_prev[3] = {0};
  uint16_t Vdc_filt = ((uint32_t)ADCxConvertedData[0] + Vdc_prev[0] + Vdc_prev[1] + Vdc_prev[2]) >> 2;
  debug_Vdc_filt = Vdc_filt;
  Vdc_prev[2] = Vdc_prev[1];
  Vdc_prev[1] = Vdc_prev[0];
  Vdc_prev[0] = ADCxConvertedData[0];


  // #######################
  // ### error checking  ###
  // #######################

  // Vin
  if (Vdc_filt > VIN_MIN_RAW && Vdc_filt < VIN_MAX_RAW) {
	  if (Vdc_filt > VIN_START_RAW) {
		  status_Vin = true;
	  }
  } else {
	  GPIOA->BSRR = SHUTDOWN_MASK;  // disable PWM & yellow LED
	  GPIOF->BRR = 1<<1;  // disable contactor
	  status_Vin = false;
	  state = INIT;
  }

  // Vin crowbar to limit input PV voltage
  if (Vdc_filt > VIN_CROWBAR_MAX_RAW) {
	  GPIOA->BSRR = 1<<4;  // enable PV crowbar
	  cnt_crowbar_actions++;
  } else if (Vdc_filt < VIN_CROWBAR_MIN_RAW) {
	  GPIOA->BRR = 1<<4;  // disable PV crowbar
  }

  // Vgrid
  int32_t vd = pll_get_vd();
  int32_t w = pll_get_w();
  static uint32_t cnt_pll_locked = 0;
  if (vd > VD_MIN_RAW && vd < VD_MAX_RAW &&
	   w > W_MIN_RAW && w < W_MAX_RAW) {
	  if (cnt_pll_locked == 0.2*CTRL_FREQ) {  // 200ms
		  status_Vgrid = true;
	  } else {
		  cnt_pll_locked++;
	  }
  } else {
	  status_Vgrid = false;
	  cnt_pll_locked = 0;
  }

  // #################
  // ### read ADC  ###
  // #################

  // Now configured in auto trigger mode

  // DMA mode takes 6.02us for 3 channels; 5.92uS without if(HAL_OK) -> 20us-6us = 14us -> 14us*48MHz = 672 cycles
  //static uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

  /* ### - 4 - Start conversion in DMA mode ################################# */
  //HAL_ADC_Start_DMA(&hadc, (uint32_t *)aADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE );


  // polling mode takes 6.25us (for 1channel?)
//	    /*##-3- Start the conversion process #######################################*/
//	    if (HAL_ADC_Start(&hadc) != HAL_OK)
//	    {
//	      /* Start Conversation Error */
//	      Error_Handler();
//	    }
//
//	    /*##-4- Wait for the end of conversion #####################################*/
//	    /*  For simplicity reasons, this example is just waiting till the end of the
//	    conversion, but application may perform other tasks while conversion
//	    operation is ongoing. */
//	    if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
//	    {
//	      /* End Of Conversion flag not set on time */
//	      Error_Handler();
//	    }
//	    else
//	    {
//	      /* ADC conversion completed */
//	      /*##-5- Get the converted value of regular channel  ########################*/
//	    	aADCxConvertedData[0] = HAL_ADC_GetValue(&hadc);
//	    }

  int16_t duty1 = 0;
  static uint32_t cnt_rel = 0;
  cnt_rel++;

#define PWM_MAX 800
#define PWM_MAX_HALF (PWM_MAX>>1)

	  int16_t phase = pll_singlephase_step(ADCxConvertedData[1]-adcOff);

  switch (state) {
  	  case INIT:
  		  GPIOF->BRR = 1<<1;  // disable contactor
  		  GPIOA->BSRR = SHUTDOWN_MASK;  // disable PWM & yellow LED
//  		  if (status_Vin  && (cnt_rel >= 10*CTRL_FREQ)) {  // wait 10sec, cnt_rel exactly equal (==) seems to causes hangup in morning startup
		  if (status_Vin) {
  			  state = POWER_TEST;
  			  cnt_crowbar_actions = 0;
  			  cnt_rel = 0;
  		  }
  		  break;

  	  case POWER_TEST:

  		  // higher power consumption through sine generation -> safety risk
//  		  static int16_t phase_gridforming = 0;
//  		  phase_gridforming += ((1<<15)*50)/CTRL_FREQ;
//  		  duty1 = PWM_MAX_HALF + ( (int32_t)( (PWM_MAX_HALF*(VGRID_AMP/VGRID_TRATIO)/VDC_REF)*(int32_t)cos1(phase_gridforming) ) >> 15 );
//		  GPIOA->BRR = SHUTDOWN_MASK;  // enable PWM

		  // If crowbar has a lot of work to do in the first 30 seconds -> enough power is available
		  if (cnt_crowbar_actions > 1080) {
			  state = GRID_CONNECTING;
			  cnt_rel = 0;
		  }

		  if (cnt_rel >= 30*CTRL_FREQ) {  // if still here after 30 sec -> reset microcontroller
			  NVIC_SystemReset();  // most efficient way to get back to first state
		  }
  		  break;

  	  case GRID_CONNECTING:
  		 GPIOF->BSRR = 1<<1;  // enable contactor
  		 if (cnt_rel == 0.025*CTRL_FREQ) {  // 25ms delay for contactor action
  			state = GRID_SYNC;
  			cnt_rel = 0;
  			piCtrl.y = 0;
  			piCtrl.x_prev = 0;
  		 }
  		 break;

  	  case GRID_SYNC:

		  // #########################
		  // ### control algorithm ###
		  // #########################

		  if (status_Vgrid) {

#define EXTEND_LOC 5
#define SCALE_VAC2VDC (int32_t)( ((VGRID_ADCR/VGRID_TRATIO) / VIN_ADCR ) * (1<<EXTEND_LOC) )

			cnt_rel = 0;
			//int16_t i_ref_amp = step_pi_Vdc2IacAmp( VDC_REF_RAW, Vdc_filt );
			int16_t i_ref_amp = step_pi_Vdc2IacAmp_volt_comp( VDC_REF_RAW, Vdc_filt, phase );
			debug_i_ref_amp = i_ref_amp;

			int16_t duty_pll = ( pll_get_vd()*SCALE_VAC2VDC * PWM_MAX_HALF / Vdc_filt ) >> EXTEND_LOC;

			//int16_t v_amp_pred = 20*step_IacAmp2VacSecAmp(i_ref_amp);  // 10 Ohm resistor test + 130uH  22.4ns rise time 24.18V/22.0Vdc
			//int16_t v_amp_pred = 40*step_IacAmp2VacSecAmp(i_ref_amp);  // 10 Ohm resistor test + 130uH  36.5ns rise time 22.3V/22.0Vdc bzw. 10.4ns rise time 27.53V/21.78Vdc
								                                         // ohne Gate Ausschaltdioden 26ns rise time 22.19V/21.4Vdc -> minimal overshoot
			int16_t v_amp_pred = calc_IacAmp2VacSecAmpDCscale(i_ref_amp);
			debug_v_amp_pred = v_amp_pred;
			int16_t duty_v_amp_pred = ( v_amp_pred * PWM_MAX_HALF ) / Vdc_filt;

			// irefamp 3,3A -> 13,7W  7,6A ->30W  11,1A -> 43W  12,2V->49W(25,2-27,6Vdc)

			// fixed L and R Trafo + winding parallel
			//int16_t phase_shiftRL = get_IacPhase();  // Strom eilt 505us vor 10,5W 12,5VA
			//int16_t phase_shiftRL = get_IacPhase() + (int16_t)(((1<<15)*-0.505)/20); //IAC_AMP_MAX 12.0  // 38W/45VA Strom(4Aamp) eilt 1,8ms nach (23,6Voutamp 27,4Vdc)
			// bei kleinen Strömen eilt Strom vor, bei großen nach -> keine Kompensation
			//int16_t phase_shiftRL = get_IacPhase(); // Strom eilt 1.08ms nach bei 4,2Aamp
			int16_t phase_shiftRL = get_IacPhase()+ (int16_t)(((1<<15)*1.08)/20);  // Strom(3,5Aamp) in Phase bei 40,5W/42,5VA (25,4Vdc bis 27,3Vdc)

			//duty1 = PWM_MAX_HALF + ( ( PWM_MAX_HALF*(int32_t)cos1(phase) ) >> 15 );  // MAX amplitude test
			//duty1 = PWM_MAX_HALF + ( ( (duty_v_amp_pred)*(int32_t)cos1(phase) ) >> 15 );  //resistor test; asymm, maybe because of Oszi
			duty1 = PWM_MAX_HALF + ( ( (duty_pll*(int32_t)cos1(phase) + duty_v_amp_pred*(int32_t)cos1(phase+phase_shiftRL)) ) >> 15 );

			static uint32_t cnt_power_low = 0;

			if (i_ref_amp < I_REF_AMP_MIN_RAW) {
				cnt_power_low++;
				if (cnt_power_low == 30*CTRL_FREQ ) {  // 30 sec low power -> reset microcontroller
					NVIC_SystemReset();
				}
			} else {
				cnt_power_low = 0;
			}

			GPIOA->BRR = SHUTDOWN_MASK;  // enable PWM & yellow LED

  		  } else {
			  GPIOA->BSRR = SHUTDOWN_MASK;  // disable PWM & yellow LED
			  if (cnt_rel == 30*CTRL_FREQ) {  // 30 sec no valid grid -> reset microcontroller
				  NVIC_SystemReset();
			  }
  		  }

		  break;
	}

	  // single PWM step has 1/48MHz = 20.83ns
#define MIN_PULSE 4  // min pulse duration is 83ns - 50ns deadtime = 33ns
	  if (duty1 > (PWM_MAX-MIN_PULSE)) {
		  duty1 = PWM_MAX;
	  } else if (duty1 < MIN_PULSE){
		  duty1 = 0;
	  }

	  // A leg
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, duty1);  //update pwm value

	  // B leg
	  int16_t duty2 = PWM_MAX_HALF+(PWM_MAX_HALF-duty1);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duty2);  //update pwm value
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, duty2);  //update pwm value


	  GPIOA->BRR = (1<<5); //switch off PA5

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
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  GPIOA->BSRR = SHUTDOWN_MASK;  // disable PWM & yellow LED

  char Tx_Buffer[64];
  uint8_t Tx_len;

//  HAL_UART_Transmit(&huart1,(uint8_t *)Tx_Buffer,Tx_len, 10);
  //HAL_UART_Transmit_IT(&huart1,(uint8_t *)Tx_Buffer,Tx_len);


  // ###########################
  // ### Timer configuration ###
  // ###########################

  if (  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  if (   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  if (   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  if (  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }

  if (  HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  if ( HAL_ADC_Start_DMA(&hadc, (uint32_t *)ADCxConvertedData, sizeof(ADCxConvertedData)/sizeof(ADCxConvertedData[0]))  != HAL_OK)
  {
    Error_Handler();
  }

	// get ADC offset -> can be done once for every new microcontroller
//    uint32_t adc_sum = 0;
//	for (int i=0; i < (1<<15); i++ ) {
//	  /* ### - 4 - Start conversion in DMA mode ################################# */
//	  //HAL_ADC_Start_DMA(&hadc, (uint32_t *)ADCxConvertedData, sizeof(ADCxConvertedData)/sizeof(ADCxConvertedData[0]));
//	  while (!conv_done) {
//		  asm("nop");
//	  }
//
//	  conv_done = false;
//	  adc_sum += ADCxConvertedData[1];
//	}
//	adcOff = adc_sum >> 15;
//
//	uSend("\nADCcal ");
//	itoa(adcOff, Tx_Buffer, 10);
//	Tx_len=strlen(Tx_Buffer);
//	HAL_UART_Transmit(&huart1, (uint8_t *)Tx_Buffer, Tx_len, 10);
//	uSend("\n");

	adcOff = 943-9;  //+0 : 200mAdc 3,5W/20VA  +10:390mAdc   -9: 40mAdc 2,7W/13VA  0,5W/10VA/1,3Irefamp
	pll_set_phaseOffset((1<<15) * -10.03/20);  // zero crossing of grid and converter voltage matched

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //HAL_IWDG_Refresh(&hiwdg);

	  //if (cnt_intr % CTRL_FREQ == 0) {  // V1: once a second
	  // V2: on data request
	  uint8_t Rx_Buffer = 0;
	  HAL_UART_Receive(&huart1, &Rx_Buffer, 1, 10);

	  if (Rx_Buffer == 'p') {

		  uSend("state");
		  itoa(state, Tx_Buffer, 10);
		  Tx_len=strlen(Tx_Buffer);
		  HAL_UART_Transmit(&huart1, (uint8_t *)Tx_Buffer, Tx_len, 10);

		  if (status_Vin) {
			  uSend(" Vin:ok");
		  }
		  if (status_Vgrid) {
			  uSend(" Vgrid:ok");
		  }

		  uSend("\ncnt_crowbar_actions");
		  itoa(cnt_crowbar_actions, Tx_Buffer, 10);
		  Tx_len=strlen(Tx_Buffer);
		  HAL_UART_Transmit(&huart1, (uint8_t *)Tx_Buffer, Tx_len, 10);

		  uSend("\nvd");
		  itoa((pll_get_vd()*(uint16_t)VGRID_ADCR) >> ADC_BITS, Tx_Buffer, 10);
		  Tx_len=strlen(Tx_Buffer);
		  HAL_UART_Transmit(&huart1, (uint8_t *)Tx_Buffer, Tx_len, 10);

		  uSend("\nw");
		  itoa(pll_get_w()*100/((1<<15)-1), Tx_Buffer, 10);
		  Tx_len=strlen(Tx_Buffer);
		  HAL_UART_Transmit(&huart1, (uint8_t *)Tx_Buffer, Tx_len, 10);

		  uSend("\nVdc");
		  itoa((debug_Vdc_filt * (uint16_t) (10*VIN_ADCR)) >> ADC_BITS, Tx_Buffer, 10);
		  Tx_len=strlen(Tx_Buffer);
		  HAL_UART_Transmit(&huart1, (uint8_t *)Tx_Buffer, Tx_len, 10);

		  uSend("\nIrefAmp");
		  itoa((debug_i_ref_amp*10*(uint16_t)IGRID_ADCR) >> ADC_BITS, Tx_Buffer, 10);
		  Tx_len=strlen(Tx_Buffer);
		  HAL_UART_Transmit(&huart1, (uint8_t *)Tx_Buffer, Tx_len, 10);

//		  uSend("\nVdcR");
//		  itoa(Vdc_filt, Tx_Buffer, 10);
//		  Tx_len=strlen(Tx_Buffer);
//		  HAL_UART_Transmit(&huart1, (uint8_t *)Tx_Buffer, Tx_len, 10);
//
//		  uSend("\nVacR");
//		  itoa(ADCxConvertedData[1], Tx_Buffer, 10);
//		  Tx_len=strlen(Tx_Buffer);
//		  HAL_UART_Transmit(&huart1, (uint8_t *)Tx_Buffer, Tx_len, 10);
//
//		  uSend("\nIrefAmpR");
//		  itoa(debug_i_ref_amp, Tx_Buffer, 10);
//		  Tx_len=strlen(Tx_Buffer);
//		  HAL_UART_Transmit(&huart1, (uint8_t *)Tx_Buffer, Tx_len, 10);
//
//		  uSend("\nv_amp_predR");
//		  itoa(debug_v_amp_pred, Tx_Buffer, 10);
//		  Tx_len=strlen(Tx_Buffer);
//		  HAL_UART_Transmit(&huart1, (uint8_t *)Tx_Buffer, Tx_len, 10);

		  uSend("\n\n");
	  }
	  //HAL_Delay(1000);  //ms
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 5;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  uSend("EH\n");

  NVIC_SystemReset();
//  while (1)
//  {
//  }
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
