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
#include "string.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbh_hid.h"
#include "notes.h"
#include "keybd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _NOTE_C0 16.35160
#define _NOTE_CS0 17.32391
#define _NOTE_D0 18.35405
#define _NOTE_DS0 19.44544
#define _NOTE_E0 20.60172
#define _NOTE_F0 21.82676
#define _NOTE_FS0 23.12465
#define _NOTE_G0 24.49971
#define _NOTE_GS0 25.95654
#define _NOTE_A0 27.50000
#define _NOTE_AS0 29.13524
#define _NOTE_B0 30.86771

#define _NOTE_C1 32.70320
#define _NOTE_CS1 34.64783
#define _NOTE_D1 36.70810
#define _NOTE_DS1 38.89087
#define _NOTE_E1 41.20344
#define _NOTE_F1 43.65353
#define _NOTE_FS1 46.24930
#define _NOTE_G1 48.99943
#define _NOTE_GS1 51.91309
#define _NOTE_A1 55.00000
#define _NOTE_AS1 58.27047
#define _NOTE_B1 61.73541

#define _NOTE_C2 65.40639
#define _NOTE_CS2 69.29566
#define _NOTE_D2 73.41619
#define _NOTE_DS2 77.78175
#define _NOTE_E2 82.40689
#define _NOTE_F2 87.30706
#define _NOTE_FS2 92.49861
#define _NOTE_G2 97.99886
#define _NOTE_GS2 103.8262
#define _NOTE_A2 110.0000
#define _NOTE_AS2 116.5409
#define _NOTE_B2 123.4708

#define _NOTE_C3 130.8128
#define _NOTE_CS3 138.5913
#define _NOTE_D3 146.8324
#define _NOTE_DS3 155.5635
#define _NOTE_E3 164.8138
#define _NOTE_F3 174.6141
#define _NOTE_FS3 184.9972
#define _NOTE_G3 195.9977
#define _NOTE_GS3 207.6523
#define _NOTE_A3 220.0000
#define _NOTE_AS3 233.0819
#define _NOTE_B3 246.9417

#define _NOTE_C4 261.6256
#define _NOTE_CS4 277.1826
#define _NOTE_D4 293.6648
#define _NOTE_DS4 311.1270
#define _NOTE_E4 329.6276
#define _NOTE_F4 349.2282
#define _NOTE_FS4 369.9944
#define _NOTE_G4 391.9954
#define _NOTE_GS4 415.3047
#define _NOTE_A4 440.0000
#define _NOTE_AS4 466.1638
#define _NOTE_B4 493.8833

#define _NOTE_C5 523.2511
#define _NOTE_CS5 554.3653
#define _NOTE_D5 587.3295
#define _NOTE_DS5 622.2540
#define _NOTE_E5 659.2551
#define _NOTE_F5 698.4565
#define _NOTE_FS5 739.9888
#define _NOTE_G5 783.9909
#define _NOTE_GS5 830.6094
#define _NOTE_A5 880.0000
#define _NOTE_AS5 932.3275
#define _NOTE_B5 987.7666

#define _NOTE_C6 1046.502
#define _NOTE_CS6 1108.731
#define _NOTE_D6 1174.659
#define _NOTE_DS6 1244.508
#define _NOTE_E6 1318.510
#define _NOTE_F6 1396.913
#define _NOTE_FS6 1479.978
#define _NOTE_G6 1567.982
#define _NOTE_GS6 1661.219
#define _NOTE_A6 1760.000
#define _NOTE_AS6 1864.655
#define _NOTE_B6 1975.533

#define _NOTE_C7 2093.005
#define _NOTE_CS7 2217.461
#define _NOTE_D7 2349.318
#define _NOTE_DS7 2489.016
#define _NOTE_E7 2637.020
#define _NOTE_F7 2793.826
#define _NOTE_FS7 2959.955
#define _NOTE_G7 3135.963
#define _NOTE_GS7 3322.438
#define _NOTE_A7 3520.000
#define _NOTE_AS7 3729.310
#define _NOTE_B7 3951.066

#define _NOTE_C8 4186.009
#define _NOTE_CS8 4434.922
#define _NOTE_D8 4698.636
#define _NOTE_DS8 4978.032
#define _NOTE_E8 5274.041
#define _NOTE_F8 5587.652
#define _NOTE_FS8 5919.911
#define _NOTE_G8 6271.927
#define _NOTE_GS8 6644.875
#define _NOTE_A8 7040.000
#define _NOTE_AS8 7458.620
#define _NOTE_B8 7902.133



#define NS 128
#define DAC_SAMPLES 1024
#define HALF_SAMPLE 512
#define SAMP_RATE 46875
#define WAVETABLE_SIZE 1874
#define WAVETABLE_FREQ 25
#define SAMP_TIME 21.333333 //us
#define NUM_KEYS 16

#define STATUS_WAIT 0
#define STATUS_HALF 1
#define STATUS_DONE 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
struct NOTE {
	float start;
	float end;
	float pitch;
};

extern int wavetable[WAVETABLE_SIZE];
extern int wavetable2[WAVETABLE_SIZE];

int key_pressed = 0;
const char key_names[] = "zsxcfvgbnjmk,l./";
const float key_pitches[] = {220.0, 233.0819, 246.9417, 261.6256, 277.1826, 293.6648, 311.1270, 329.6276, 349.2282, 369.9944, 391.9954,
		415.3047, 440.0000, 466.1638, 493.8833, 523.2511};
struct NOTE note_store[500];
int key_tracker[16] = {0};
uint32_t DAC_buff[DAC_SAMPLES] = {0};

uint32_t Wave_LUT[NS] = {
    2048, 2149, 2250, 2350, 2450, 2549, 2646, 2742, 2837, 2929, 3020, 3108, 3193, 3275, 3355,
    3431, 3504, 3574, 3639, 3701, 3759, 3812, 3861, 3906, 3946, 3982, 4013, 4039, 4060, 4076,
    4087, 4094, 4095, 4091, 4082, 4069, 4050, 4026, 3998, 3965, 3927, 3884, 3837, 3786, 3730,
    3671, 3607, 3539, 3468, 3394, 3316, 3235, 3151, 3064, 2975, 2883, 2790, 2695, 2598, 2500,
    2400, 2300, 2199, 2098, 1997, 1896, 1795, 1695, 1595, 1497, 1400, 1305, 1212, 1120, 1031,
    944, 860, 779, 701, 627, 556, 488, 424, 365, 309, 258, 211, 168, 130, 97,
    69, 45, 26, 13, 4, 0, 1, 8, 19, 35, 56, 82, 113, 149, 189,
    234, 283, 336, 394, 456, 521, 591, 664, 740, 820, 902, 987, 1075, 1166, 1258,
    1353, 1449, 1546, 1645, 1745, 1845, 1946, 2047
};


uint8_t status = STATUS_DONE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void adjust_gain(int gain){
	float amplitude = pow(10,(gain/20));
	for(int i = 0; i < NS; i++){
		wavetable2[i]*=amplitude;
	}
}

void fill_dac(){
	for(int i = 0; i < DAC_SAMPLES; i++){
		DAC_buff[i] = Wave_LUT[i%NS];
		if(key_pressed == 0){
			break;
		}
	}
}

int linear_interpolate(float index){
	int index_floor = (int)floor(index);
	int index_next = (index_floor + 1) % WAVETABLE_SIZE;
	float index_next_weight = index - (float)index_floor;
	float index_floor_weight = 1.0 - index_next_weight;
	int val = (index_floor_weight*(float)wavetable[index_floor] + index_next_weight*(float)wavetable[index_next]);

	return val;
}

void add_note(uint32_t* buffer, float pitch, int start_index, int end_index, float gain, int on){
	if(on == 0){
		return;
	}
	float index = 0;
	float index_increment = pitch * (float)WAVETABLE_SIZE / (float)SAMP_RATE;
	for(int i = start_index; i < end_index; i++){
		buffer[i] += (int)(gain*linear_interpolate(index));
		index += index_increment;
		index = fmodf(index, (float)WAVETABLE_SIZE);
	}
}


void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
	//uint8_t key;
	HID_KEYBD_Info_TypeDef *Keyboard_Info;
	Keyboard_Info = USBH_HID_GetKeybdInfo(phost);  // get the info
	//key = USBH_HID_GetASCIICode(Keyboard_Info);  // get the key pressed
	get_all_keys(Keyboard_Info, key_tracker);
	//printf ("Key Pressed = %c\n", key);
	//printf("Keys Pressed: \n");
//	for(int i = 0; i < 16; i++){
//			if(key_tracker[i] == 1){
//				printf("%c ",key_names[i]);
//			}
//		}
//		printf(" \n");
//	for(int i = 0; i < 16; i++){
//		printf("%i ",key_tracker[i]);
//	}
//	printf(" \n");
//	for(int i = 0; i < 16; i++){
//		if(key_tracker[i] == 1){
//			key_pressed = 1;
//			fill_dac();
//			return;
//		}
//	}
//	key_pressed=0;

}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	status = STATUS_DONE;
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	status = STATUS_HALF;
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  adjust_gain(-20);
  if(HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, DAC_buff, DAC_SAMPLES, DAC_ALIGN_12B_R) != HAL_OK){
		  Error_Handler();
	  }
  uint32_t audio_buffer[DAC_SAMPLES] = {0};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();
    switch(status){
    case STATUS_HALF:
    	//update first half of dac buffer
    	for(int i = 0; i < HALF_SAMPLE; i++){
    		DAC_buff[i] = audio_buffer[i];
    	}
    	memset(audio_buffer,0,sizeof audio_buffer / 2);
    	for(int i = 0; i < NUM_KEYS; i++){
    		add_note(audio_buffer, key_pitches[i], 0, HALF_SAMPLE, 0.05,key_tracker[i]);
    	}
    	break;
    case STATUS_DONE:
    	//update second half of dac buffer
    	for(int i = HALF_SAMPLE; i < DAC_SAMPLES; i++){
    	    		DAC_buff[i] = audio_buffer[i];
    	    }
    	memset(audio_buffer+HALF_SAMPLE,0,sizeof audio_buffer/2);
    	for(int i = 0; i < NUM_KEYS; i++){
    	    		add_note(audio_buffer, key_pitches[i], HALF_SAMPLE, DAC_SAMPLES,0.05, key_tracker[i]);
    	   }
    	break;
    case STATUS_WAIT:
    default:
    	break;
    }

    /* USER CODE BEGIN 3 */
	  // TODO: to be changed later


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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T1_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1600-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
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

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
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
  HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
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
