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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "MdlUARTnModbus.h"
#if SUPPORT_GPRS
#include "MdlGprs.h"
#endif

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
DMA_HandleTypeDef hdma_adc1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId DefaultTaskHandle;
uint32_t DefaultTaskBuffer[ 128 ];
osStaticThreadDef_t DefaultTaskControlBlock;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
SEM_Handle SEM_CtrTskRun, SEM_NvMemAcs;
TSK_Handle CtrTaskHandle, TSK_DataAcs;
osStaticThreadDef_t CtrTskCtlBlk;
osStaticSemaphoreDef_t CtrSemCtlBlk;
#define CTR_TSK_STACK_BLEN		2048
uint8 CtrTaskStack[CTR_TSK_STACK_BLEN];
void CtrTask(const void* argument);
int main(void);
extern void __libc_init_array(void);
TSK_Handle MqttSubTaskHandle;
TSK_Handle MqttPubTaskHandle;
void NetTask(const void* argument);

osStaticThreadDef_t MqttPubTskCtlBlk;
void MqttPubTask(void);

osStaticThreadDef_t MqttSubTskCtlBlk;
void MqttSubTask(void);

/* 閻拷.s缂堟槒鐦ч懛铏劃閿涘本顒濋崙鑺ユ殶闁灝鍘ゆ担璺ㄦ暏閸忋劌鐪崣姗�鍣洪敍灞芥礈娑撳搫鍨垫慨瀣閺冩湹绱伴弨鐟板綁娴犳牜娈戦崐锟� */
void _c_int00(void)
{
	uint32 *pValInitSrc;
	uint32 *pValInitDst;

	extern unsigned int _sdata, _edata, _sidata, _sbss, _ebss;		/* 鏉╂瑤绨洪崣姗�鍣洪惃鍕暰娑斿婀柧鐐复閼存碍婀伴柌锟� */

    /* Call the clock system initialization function. */
    SystemInit();

    /* Copy the data segment initializers from flash to SRAM (LoopCopyDataInit)*/
    pValInitSrc = (uint32*)&_sidata;
    pValInitDst = (uint32*)&_sdata;
    while (pValInitDst < (uint32*)&_edata) {
        *pValInitDst++ = *pValInitSrc++;
    }

    /* Zero fill the bss segment.(LoopFillZerobss) */
    pValInitSrc = (uint32*)&_sbss;		/* _sbss韫囧懘銆忛悽銊︾垽闁夸礁鐡� */
	pValInitDst = (uint32*)&_ebss;
    while (pValInitSrc < pValInitDst) {
        *pValInitSrc++ = 0;
    }

    /* ResetFunc();	TODO:閹跺ィI閻ㄥ嫭灏撻崚鐧塺vTI */

    /* Call static constructors */
    __libc_init_array();

    /* Call the application's entry point. */
    main();
}

int __io_putchar(int ch)
{
	HAL_UART_Transmit(g_UartComm[0].Handle, (uint8_t *)&ch, 1, 1000);
	return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//  /* 閸氬本顒炵�规碍妞傞崳锟� */
//  volatile uint32* pU32 = (uint32*)0xE0042008;
//  *pU32 = 0x000F;   /* debug閺嗗倸浠燙PU閿涘苯浠燭IM2/3/4/5閿涘奔澶嶉弮鑸碉拷褏娈� */
//  pU32 = (uint32*)0xE004200C;
//  *pU32 = 0x0003;   /* debug閺嗗倸浠燙PU閿涘苯浠燭IM1/8閿涘奔澶嶉弮鑸碉拷褏娈� */
//  htim2.Instance->EGR = 1;  /* 閺堬拷娴ｅ簼缍匲G缂冾喕璐�1閿涘矂鍣哥純顔款吀閺佹澘娅掗敍灞间簰娓氬灝鐤勯悳鏉跨暰閺冭泛娅掗崥灞绢劄 */
 	InitDebugDat();
 	g_CodeTest.i32Val[99] = SOFTWARE_VER;
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
  MX_UART4_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 閺囧瓨鏌婇崥顖氬З閻╃鍙ч崣姗�鍣洪敍灞筋洤閺嬫粍妲搁崘宄版儙閸旓拷(RCC_CSR_PORRSTF)閵嗕礁宕岀痪褎鍨ㄦ禍杞拌礋鏉烆垯娆㈤柌宥呮儙(RCC_CSR_PINRSTF|RCC_CSR_SFTRSTF)閵嗕浇鐨熺拠鏇犲閺堫剨绱濋崚娆忕殺闁插秴鎯庣拋鈩冩殶濞撳懘娴� */
  #if (DEVICE_TYPE == YKC)
  if((RCC->CSR & RCC_CSR_PORRSTF) || CheckDebugVersion()) {
	  g_Sys.uRstCount = 0;
  } else if((g_Sys.uRstCount < 0xFFFF) && (g_Sys.bCtrMdlOK)) {
	  g_Sys.uRstCount++;
  }
  #else	/* YYT4S etc */
  if((RCC->CSR & (RCC_CSR_BORRSTF | RCC_CSR_PORRSTF)) || CheckDebugVersion()) {
	  g_Sys.uRstCount = 0;
	  g_Sys.u32UpTimer_s = 0;
	} else if((g_Sys.uRstCount < 0xFFFF)) {
	  if(g_Sys.bCtrMdlOK) {
		  g_Sys.uRstCount++;
	  }
	}
  #endif
  RCC->CSR = RCC_CSR_RMVF;  	/* 閸氼垰濮╅崢鐔锋礈鐎靛嫬鐡ㄩ崳銊ヮ槻娴ｏ拷 */
  g_Sys.u32RstTimer_s = 0;
  g_Sys.uTmr_1Hz = 1000;

  /* 閸掓繂顫愰崠鏍у彠闁款喖褰夐柌锟� */
	g_Sys.bCtrMdlOK = FALSE;
	g_Sys.tReboot_0Null_nUrgent_pWaitIdle = 0;
	g_Sys.u8Tmr_RstExtNet_tick = 0;
	g_Sys.u32Seconds_LastSyncRTC = 0;
	g_Sys.iTimer_nSoftChkWaitTick_pErrCode = -600;    /* 鏉╁洣绔撮崚鍡涙寭閸愬秴浠涢崚銈嗘焽--SysTask鐠嬪啰鏁ゆ０鎴犲芳閺勶拷10Hz閿涘苯姘ㄧ粻妞剧瑝閸戝棴绱濋崗宕囬兇娑旂喍绗夋径锟� */

	if(g_Sys.uRstCount == 0) {
		InitDataWithZero((uint8*)(&g_DataAcsIntf), sizeof(g_DataAcsIntf));
		InitDataWithZero((uint8*)(&g_Ctr), sizeof(g_Ctr));
		g_Sys.uTmr_EraseFlash_DistPwr_ms = 0;
	}
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of DefaultTask */
  osThreadStaticDef(DefaultTask, StartDefaultTask, osPriorityNormal, 0, 128, DefaultTaskBuffer, &DefaultTaskControlBlock);
  DefaultTaskHandle = osThreadCreate(osThread(DefaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  TaskCreate("SysTask", SysTask, NULL, osPriorityLow, 1, (uint8*)g_SysBlock.SysStackBuf, SYS_TASK_STACK_BLEN, &g_SysBlock.SysTaskCtlBlk);

  /* 闁俺顔嗘禒璇插 */
  /* UART */
  InitUartComm();

  g_UartComm[0].Handle = &huart4;
  g_UartComm[0].Sem_TxCplt = SemCreate("Uart0TxCplt", 1, &g_UartBlock[0].TxSemCtlBlk);
  g_UartComm[0].Sem_RxGet = SemCreate("Uart0RxCplt", 1, &g_UartBlock[0].RxSemCtlBlk);
//  TaskCreate("UartTask0", UartCommTask, (uint32*)0, osPriorityHigh, 0, g_UartBlock[0].Stack, MAX_UART_TASK_STACK_LEN, &g_UartBlock[0].TaskCtlBlk);		姝や覆鍙ｄ笉鐢ㄧ嫭绔嬩换鍔＄鐞嗭紝鑰岀敱璋冭瘯闇�姹傝礋璐ｇ鐞�

  g_UartComm[1].Handle = &huart3;
  g_UartComm[1].Sem_TxCplt = SemCreate("Uart1TxCplt", 1, &g_UartBlock[1].TxSemCtlBlk);
  g_UartComm[1].Sem_RxGet = SemCreate("Uart1RxCplt", 1, &g_UartBlock[1].RxSemCtlBlk);
//  TaskCreate("UartTask1", UartCommTask, (uint32*)0, osPriorityHigh, 0, g_UartBlock[1].Stack, MAX_UART_TASK_STACK_LEN, &g_UartBlock[1].TaskCtlBlk);		姝や覆鍙ｄ笉鐢ㄧ嫭绔嬩换鍔＄鐞嗭紝鑰岀敱TTS璇煶妯″潡鍔熻兘璐熻矗绠＄悊

  g_UartComm[2].Handle = &huart1;
  g_UartComm[2].Sem_TxCplt = SemCreate("Uart2TxCplt", 1, &g_UartBlock[2].TxSemCtlBlk);
  g_UartComm[2].Sem_RxGet = SemCreate("Uart2RxCplt", 1, &g_UartBlock[2].RxSemCtlBlk);
  g_UartComm[2].uRxBufPt = 0;
  g_UartComm[2].uRxFrameIndex = 0;
//  TaskCreate("UartTask2", UartCommTask, (uint32*)0, osPriorityHigh, 0, g_UartBlock[2].Stack, MAX_UART_TASK_STACK_LEN, &g_UartBlock[2].TaskCtlBlk);		姝や覆鍙ｄ笉鐢ㄧ嫭绔嬩换鍔＄鐞嗭紝鑰岀敱GPRS鍔熻兘璐熻矗绠＄悊

  /* SPI */
  g_SPIComm[0].Handle = &hspi1;
  //Sem_TxCplt閸︹垼penSPI鐟侊拷
  
  /* 閹貉冨煑娴犺濮� */
  g_SysBlock.SEM_Ctr = SemCreate("CtrRun", 1, &g_SysBlock.RunSemCtlBlk);
  TaskCreate("CtrTask", CtrTask, NULL, osPriorityRealtime, 1, (uint8*)g_SysBlock.CtrStackBuf, CTR_TASK_STACK_BLEN, &g_SysBlock.CtrTaskCtlBlk);
  
  /* 鐎涙ê鍋嶆禒璇插 */
  g_DataAccessBlock.Sem_Op = SemCreate("NvMemOp", 1, &g_DataAccessBlock.OpSemCtlBlk);
  g_DataAccessBlock.DataAcsTSK = TaskCreate("DataAcs", DataAccessTask, NULL, osPriorityRealtime, 1, g_DataAccessBlock.Stack, DATA_ACCESS_TASK_STACK_LEN, &g_DataAccessBlock.TaskCtlBlk);

  /* 缂冩垹绮舵禒璇插 */
#if SUPPORT_GPRS
  /* GPRS req */
  g_GprsComm.Sem_GprsReq = SemCreate("GprsReq", 1, &g_GprsComm.SemCtlBlk);
  Semaphore_post(g_GprsComm.Sem_GprsReq);
#endif

  /* Mqtt pub sem */
  SEM_MqttPubReq = SemCreate("MqttPubReq", 1, &MqttPubReqSemCtlBlk);

//  /* MQTT sub task osPriorityBelowNormal*/
  MqttSubTaskHandle = TaskCreate("MqttSub", MqttSubTask, NULL, osPriorityBelowNormal, 0, (uint8*)g_MqttComm[MQTT_TYPE_SUB].u32TaskStack, MQTT_TASK_STACK/4, &MqttSubTskCtlBlk);

  /* MQTT pub task */
  MqttPubTaskHandle = TaskCreate("MqttPub", MqttPubTask, NULL, osPriorityLow, 0, (uint8*)g_MqttComm[MQTT_TYPE_PUB].u32TaskStack, MQTT_TASK_STACK/4, &MqttPubTskCtlBlk);

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable or disable the remapping of ADC1_ETRGREG:
  * ADC1 External Event regular conversion is connected to TIM8 TRG0
  */
  __HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE();

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_AdcDmaBuf.u32Adc1Res, 5);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */
	/* 閹稿娓跺锟介崥顖滄箙闂傘劎瀚� */
	if(SOFT_RELEASE1_DEBUG0 || (g_Sys.SerialNo.u32Dat >= 10000000UL)) {
  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */
	}
  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* 闂囷拷鐟曚浇顔曠純鐡笴MR.CCxS(閸忎浇顔忛幑鏇熷磸) ICxF(鏉╁洦鎶ら崳锟�) ICxPSC(閸掑棝顣堕崳锟�) */
  htim1.Instance->CCMR1 = 0x0100;
  htim1.Instance->CCMR2 = 0x0101;
  /* 闂囷拷鐟曚浇顔曠純鐡笴ER.CCxE CCxP(0娴狅綀銆冩稉濠傚磳濞岋拷) */
  htim1.Instance->CCER = 0x1110;
  /* 闂囷拷鐟曚浇顔曠純顔煎帒鐠侀晲鑵戦弬锟� DIER.CCxE CCxP */
  htim1.Instance->DIER = 0x1C;
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Period = 3600;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3600;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 71;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  //娑撻缚濡惇浣风娑擃亜褰夐柌蹇ョ礉缂佹瑧瀛╂径鏍ф嫲IMU閻ㄥ嚲IM5閻ㄥ嫭绺哄ú缁樻杹閸︺劋绨oardSupport.c闁诧拷
  /* USER CODE END TIM5_Init 2 */

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
  htim8.Init.Prescaler = 71;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
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
  HAL_TIM_Base_Start(&htim8);
  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_nss_IMU_GPIO_Port, SPI1_nss_IMU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, red_led_Pin|blue_led_Pin|green_led_Pin|test_Pin
                          |testE13_Pin|other_3_3_en_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(right_pump_output_GPIO_Port, right_pump_output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(left_pump_output_GPIO_Port, left_pump_output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, air_sda_Pin|air_scl_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : cliff_btn_left_down_Pin machine_start_or_stop_control_Pin */
  GPIO_InitStruct.Pin = cliff_btn_left_down_Pin|machine_start_or_stop_control_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : cliff_btn_right_down_Pin cliff_btn_left_up_Pin */
  GPIO_InitStruct.Pin = cliff_btn_right_down_Pin|cliff_btn_left_up_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_nss_IMU_Pin */
  GPIO_InitStruct.Pin = SPI1_nss_IMU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_nss_IMU_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : right_bounce_btn_____Pin right_bounce_btn____C5_Pin */
  GPIO_InitStruct.Pin = right_bounce_btn_____Pin|right_bounce_btn____C5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : right_bounce_btn____B0_Pin right_bounce_btn____B1_Pin */
  GPIO_InitStruct.Pin = right_bounce_btn____B0_Pin|right_bounce_btn____B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : cliff_btn_right_up_Pin */
  GPIO_InitStruct.Pin = cliff_btn_right_up_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(cliff_btn_right_up_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : red_led_Pin blue_led_Pin green_led_Pin other_3_3_en_Pin */
  GPIO_InitStruct.Pin = red_led_Pin|blue_led_Pin|green_led_Pin|other_3_3_en_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : test_Pin testE13_Pin */
  GPIO_InitStruct.Pin = test_Pin|testE13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : right_pump_output_Pin */
  GPIO_InitStruct.Pin = right_pump_output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(right_pump_output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : left_pump_output_Pin */
  GPIO_InitStruct.Pin = left_pump_output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(left_pump_output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : left_bounce_btn_____Pin left_bounce_btn____D3_Pin left_bounce_btn____D4_Pin left_bounce_btn____D5_Pin */
  GPIO_InitStruct.Pin = left_bounce_btn_____Pin|left_bounce_btn____D3_Pin|left_bounce_btn____D4_Pin|left_bounce_btn____D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : pump_input_Pin */
  GPIO_InitStruct.Pin = pump_input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(pump_input_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : air_sda_Pin air_scl_Pin */
  GPIO_InitStruct.Pin = air_sda_Pin|air_scl_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the SysTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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
