/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <math.h>
#include <float.h>
#include <string.h>
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

void dac_set_volt(uint16_t volt_mv);
void adc_calc(void);
void dac_feedback(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define htimDAC htim16
#define htimPWM htim1
#define htimTIM htim17
#define htimSTP htim3



volatile uint8_t output_mode = MODE_NORMAL;
volatile uint8_t pulse_group_output = 0;


/**
  * @brief  ����PWM�����ȣ����޸�PSC��ARR��
  * @param  htim: ��ʱ�����ָ��
  * @param  Channel: PWMͨ����TIM_CHANNEL_1�ȣ�
  * @param  pulseWidth_ns: �����������ȣ����룩
  * @retval HAL״̬
  */
HAL_StatusTypeDef TIM_ConfigPulseWidth(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t pulseWidth_ns)
{
    // ��ȡ��ǰ��ʱ������
    uint32_t psc = htim->Instance->PSC;
    uint32_t arr = htim->Instance->ARR; 
    uint32_t timerClockFreq = HAL_RCC_GetHCLKFreq(); // ����ʹ��APB1��ʱ������������ʵ�������
    
    // ���㶨ʱ����������ʱ�䣨���룩
    float timerPeriod_ns = (1.0f / (timerClockFreq / (psc + 1))) * 1e9f;
    
    // ������Ҫ��CCRֵ
    uint32_t ccr = (uint32_t)(pulseWidth_ns / timerPeriod_ns);
    
    // ȷ��CCR1С��ARR/2����������
    if (ccr > (arr / 2)) {
        ccr = arr / 2;
    }
    
    // ȷ��CCRֵ��Ч
    if (ccr == 0) {
        ccr = 1; // ������̫�̻�̫��
    }
    
    // ����CCR�Ĵ���
    switch (Channel) {
        case TIM_CHANNEL_1:
            htim->Instance->CCR1 = ccr;
            break;
        case TIM_CHANNEL_2:
            htim->Instance->CCR2 = ccr;
            break;
        case TIM_CHANNEL_3:
            htim->Instance->CCR3 = ccr;
            break;
        case TIM_CHANNEL_4:
            htim->Instance->CCR4 = ccr;
            break;
        default:
            return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
  * @brief  ���ö�ʱ��������ָ����Ƶ�ʣ�֧��0.1Hz������Ż���
  * @param  htim: ��ʱ�����ָ��
  * @param  desiredFreq: �������ɵ�Ƶ��(Hz) (֧��0.1Hz������)
  * @retval HAL״̬
  */
HAL_StatusTypeDef TIM_ConfigFrequencyOptimized(
                            float desiredFreq, 
                            uint32_t pulseWidth_ns)
{
    // ��������������Ч��
    if (desiredFreq <= 0.0f)
    {
        return HAL_ERROR;
    }

    uint32_t bestPsc = 0;
    uint32_t bestArr = 0;
    float minError = FLT_MAX;  // ��ʼ��Ϊ��󸡵���
    float actualFreq = 0.0f;
    uint32_t timerClockFreq = HAL_RCC_GetHCLKFreq();
    
    // ��������������
    float desiredPeriods = (float)timerClockFreq / desiredFreq;
    
    // �������PSC��ARR���
    for (uint32_t psc = 0; psc <= 0xFFFF; psc++)
    {
        uint32_t arr = (uint32_t)(desiredPeriods / (psc + 1)) - 1;
        
        // ȷ��ARR����Ч��Χ��
        if (arr > 0xFFFF)
            continue;
        
        // ����ʵ��Ƶ�ʺ����
        actualFreq = (float)timerClockFreq / ((psc + 1) * (arr + 1));
        float error = fabsf(actualFreq - desiredFreq);
        
        // ����ҵ�����ȷ����ϣ��������ֵ
        if (error < minError)
        {
            minError = error;
            bestPsc = psc;
            bestArr = arr;
            
            // �������Ѿ���С����ǰ�˳�ѭ��
            if (error < (desiredFreq * 0.0001f)) // 0.01%���
                break;
        }
    }
    
    // ����Ƿ��ҵ���Ч���
    if (bestArr == 0 && bestPsc == 0 && desiredFreq < ((float)timerClockFreq / (0xFFFFFFFF)))
    {
        // ������Ƶ���
        bestPsc = 0xFFFF;
        bestArr = (uint32_t)((float)timerClockFreq / (desiredFreq * (bestPsc + 1))) - 1;
        
        if (bestArr > 0xFFFF)
            return HAL_ERROR;
    }
    
    
    // ���ö�ʱ��
    HAL_TIM_Base_Stop_IT(&htimTIM);
    HAL_Delay(1);
    HAL_TIM_PWM_PWMN_Stop(&htimPWM, TIM_CHANNEL_1);
    HAL_Delay(1);
    HAL_TIM_PWM_PWMN_Stop_IT(&htimPWM, TIM_CHANNEL_1);
    HAL_Delay(1);
    
    HAL_Delay(50);
    
    htimPWM.Instance->CNT = 0;
    htimTIM.Instance->CNT = 0;
    
    HAL_Delay(10);
		
		
		if(output_mode == MODE_SINGEL_BURST)
		{
				if(bestPsc == 0)
				{
						if (HAL_TIM_OnePulse_Init(&htimPWM, TIM_OPMODE_SINGLE) != HAL_OK)
						{
								Error_Handler();
						}
						//ֻ�õ���PWM��ʱ��
						__HAL_TIM_SET_PRESCALER(&htimPWM, 0);
						__HAL_TIM_SET_AUTORELOAD(&htimPWM, bestArr);
						
						TIM_ConfigPulseWidth(&htimPWM, TIM_CHANNEL_1, pulseWidth_ns);
						
						__HAL_TIM_SET_PRESCALER(&htimTIM, 60000-1);
						__HAL_TIM_SET_AUTORELOAD(&htimTIM, 1000-1);
						
						HAL_TIM_Base_Start_IT(&htimTIM);
				}
				else
				{
					return HAL_ERROR;
				}
		}
		else if(output_mode == MODE_REPEAT_BURST)
		{
				if(bestPsc == 0)
				{
						htimPWM.Instance->RCR = 100-1;
						if (HAL_TIM_OnePulse_Init(&htimPWM, TIM_OPMODE_SINGLE) != HAL_OK)
						{
								Error_Handler();
						}
						
						//ֻ�õ���PWM��ʱ��
						__HAL_TIM_SET_PRESCALER(&htimPWM, 0);
						__HAL_TIM_SET_AUTORELOAD(&htimPWM, bestArr);
						
						TIM_ConfigPulseWidth(&htimPWM, TIM_CHANNEL_1, pulseWidth_ns);
						
						__HAL_TIM_SET_PRESCALER(&htimTIM, 60000-1);
						__HAL_TIM_SET_AUTORELOAD(&htimTIM, 1000-1);
						
						HAL_TIM_Base_Start_IT(&htimTIM);
				}
				else
				{
					return HAL_ERROR;
				}
		}
		else
		{
				if (HAL_TIM_OnePulse_Init(&htimPWM, TIM_OPMODE_REPETITIVE) != HAL_OK)
				{
						Error_Handler();
				}
				if(bestPsc == 0)
				{
						//ֻ�õ���PWM��ʱ��
						__HAL_TIM_SET_PRESCALER(&htimPWM, 0);
						__HAL_TIM_SET_AUTORELOAD(&htimPWM, bestArr);
						
						TIM_ConfigPulseWidth(&htimPWM, TIM_CHANNEL_1, pulseWidth_ns);
						
						HAL_TIM_PWM_PWMN_Start(&htimPWM, TIM_CHANNEL_1);
				}
				else
				{
						__HAL_TIM_SET_PRESCALER(&htimTIM, bestPsc);
						__HAL_TIM_SET_AUTORELOAD(&htimTIM, bestArr);
						//ֻ�õ���PWM��ʱ��
						__HAL_TIM_SET_PRESCALER(&htimPWM, 0);
						__HAL_TIM_SET_AUTORELOAD(&htimPWM, 60000-1);
						
						TIM_ConfigPulseWidth(&htimPWM, TIM_CHANNEL_1, pulseWidth_ns);
						
						__HAL_TIM_SET_AUTORELOAD(&htimPWM, htimPWM.Instance->CCR1+10000);
						
						HAL_TIM_Base_Start_IT(&htimTIM);
				}
		}
		

    
    return HAL_OK;
}


void output_off(void){
    dac_set_volt(0);
    HAL_TIM_Base_Stop_IT(&htimTIM);
    HAL_Delay(1);
    HAL_TIM_PWM_PWMN_Stop(&htimPWM, TIM_CHANNEL_1);
    HAL_Delay(1);
    HAL_TIM_PWM_PWMN_Stop_IT(&htimPWM, TIM_CHANNEL_1);
    HAL_Delay(1);
}


uint16_t g_ram[6] = {0xFFFF,0,0xFFFF,0,0xFFFF,0,};
const uint16_t disp_output[6] = {DISP_CHAR_O,DISP_CHAR_U,DISP_CHAR_T,DISP_CHAR_P,DISP_CHAR_U,DISP_CHAR_T};
const uint16_t disp_normal_mode[11] = {DISP_CHAR_N,DISP_CHAR_O,DISP_CHAR_R,DISP_CHAR_M,DISP_CHAR_A,DISP_CHAR_L,0,DISP_CHAR_M,DISP_CHAR_O,DISP_CHAR_D,DISP_CHAR_E};
const uint16_t disp_singel_burst[12] = {DISP_CHAR_S,DISP_CHAR_I,DISP_CHAR_N,DISP_CHAR_G,DISP_CHAR_E,DISP_CHAR_L,0,DISP_CHAR_B,DISP_CHAR_U,DISP_CHAR_R,DISP_CHAR_S,DISP_CHAR_T};
const uint16_t disp_repeat_burst[12] = {DISP_CHAR_R,DISP_CHAR_E,DISP_CHAR_P,DISP_CHAR_E,DISP_CHAR_A,DISP_CHAR_T,0,DISP_CHAR_B,DISP_CHAR_U,DISP_CHAR_R,DISP_CHAR_S,DISP_CHAR_T};
const uint16_t disp_100_CPS[6] ={DISP_NUM_1,DISP_NUM_0,DISP_NUM_0,DISP_CHAR_C,DISP_CHAR_P,DISP_CHAR_S}; 
const uint16_t disp_100_count[6] ={DISP_NUM_1,DISP_NUM_0,DISP_NUM_0,DISP_CHAR_C,DISP_CHAR_N,DISP_CHAR_T}; 

volatile uint8_t step_str = 0;

//如果返回1，则说明已经显示完成
// run in 100ms
uint8_t disp_str_step(const uint16_t* str,uint8_t len){
	if(len == 0){
		memset(g_ram,0,12);
		if(step_str >= 2){
			step_str = 0;
			return 1;
		}
	}
	else if(len<=6){
		memset(g_ram,0,12);
		if(step_str <= 15){
			for(uint8_t i=0;i<len;i++){
				g_ram[i+6-len] = str[i];
			}
		}
		if(step_str >= 17){
			step_str = 0;
			return 1;
		}
	}
	else{
		memset(g_ram,0,12);
		if(step_str < 7){
			for(uint8_t i=0;i<6;i++){
				g_ram[i] = str[i];
			}
		}
		else if(step_str - 7 < len-6){
			for(uint8_t i=0;i<6;i++){
				g_ram[i] = str[step_str - 7+i+1];
			}
		}
		else if(step_str < 7+8+len-6){
			for(uint8_t i=0;i<6;i++){
				g_ram[i] = str[len-6+i];
			}
		}
		else if(step_str < 7+8+len-6+2){
		}
		else{
			step_str = 0;
			return 1;
		}
	}
	step_str++;
	return 0;
}

const uint16_t disp_number_table[10] = {DISP_NUM_0,DISP_NUM_1,DISP_NUM_2,DISP_NUM_3,DISP_NUM_4,DISP_NUM_5,DISP_NUM_6,DISP_NUM_7,DISP_NUM_8,DISP_NUM_9};

void disp_num(uint32_t num,uint8_t len,uint8_t pos,uint8_t e10){
	uint16_t temp[6];
	
	if(pos >= 6 || e10 >= 6 || len > 6)
		return;
	
	temp[0] = disp_number_table[num/100000%10];
	temp[1] = disp_number_table[num/10000%10];
	temp[2] = disp_number_table[num/1000%10];
	temp[3] = disp_number_table[num/100%10];
	temp[4] = disp_number_table[num/10%10];
	temp[5] = disp_number_table[num%10];
	
	if(e10)
		temp[5-e10] |= 0x4000;
	
	for(uint8_t i=0;i<len;i++){
		g_ram[pos+i] = temp[i+6-len];
	}
}

void disp_on(void){
	while(!disp_str_step(disp_normal_mode,sizeof(disp_normal_mode)/2))
		HAL_Delay(100);
	
	g_ram[0] = DISP_CHAR_B;
	g_ram[1] = DISP_CHAR_A;
	g_ram[2] = DISP_CHAR_T;
	g_ram[5] = DISP_CHAR_V;
	
	for(uint8_t i=0;i<10;i++){
		disp_num((uint32_t)((adc_value[BAT_CH]+0.05)*10),2,3,1);
		HAL_Delay(100);
	}
	
	memset(g_ram,0,12);
	HAL_Delay(200);
}

uint8_t need_disp_str = 0;
uint8_t need_disp_str_changed = 0;
const uint16_t  *p_disp_str1;
const uint16_t  *p_disp_str2;
uint8_t  len_disp_str1;
uint8_t  len_disp_str2;


void disp_task(void){
	static uint16_t str_timer = 0;
	if(need_disp_str > 2)need_disp_str = 0;
	
	//切换模式时候显示的字符串提示
	if(need_disp_str){
		if(need_disp_str_changed){
			need_disp_str_changed = 0;
			str_timer = 0;
			step_str = 0;
		}
		if((str_timer % 10) == 0){
			//显示字符串
			if(need_disp_str == 2){
				if(disp_str_step(p_disp_str2,len_disp_str2)){
					need_disp_str--;
				}
			}
			else if(need_disp_str == 1){
				if(disp_str_step(p_disp_str1,len_disp_str1)){
					need_disp_str--;
				}
			}
			else{
				need_disp_str = 0;
				str_timer = 0;
			}
		}
		str_timer++;
		return;
	}
	str_timer = 0;
	
	
	
}


void Change_Mode(void){
    switch(output_mode){
        case MODE_NORMAL:
            output_mode = MODE_REPEAT_BURST;
		
			need_disp_str_changed = 1;
			need_disp_str = 2;
			p_disp_str2 = disp_repeat_burst;
			len_disp_str2 = sizeof(disp_repeat_burst)/2;
		
			p_disp_str1 = disp_100_CPS;
			len_disp_str1 = sizeof(disp_100_CPS)/2;
            break;
		
        case MODE_REPEAT_BURST:
            output_mode = MODE_SINGEL_BURST;
		
			need_disp_str_changed = 1;
			need_disp_str = 2;
			p_disp_str2 = disp_singel_burst;
			len_disp_str2 = sizeof(disp_singel_burst)/2;
		
			p_disp_str1 = disp_100_count;
			len_disp_str1 = sizeof(disp_100_count)/2;
            break;
		
        case MODE_SINGEL_BURST:
        default:
            output_mode = MODE_NORMAL;
		
		
			need_disp_str_changed = 1;
			need_disp_str = 1;
		
			p_disp_str1 = disp_normal_mode;
			len_disp_str1 = sizeof(disp_normal_mode)/2;
            break;
    }
}


void HID_Task(void){
    
}





uint32_t freq = 50,pulse = 100; 
uint8_t need_fresh = 0;

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM14_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	
	//开启显示中断
	HAL_TIM_Base_Start_IT(&htim14);
    
	//ADC
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_buffer,ADC_BUFFER_SIZE*ADC_CHANNEL_NUM);
    
    disp_on();
	
	//DAC 输出
	HAL_TIM_PWM_Start(&htimDAC, TIM_CHANNEL_1);
	
	//自动关闭输出中断
	HAL_TIM_Base_Start_IT(&htimSTP);
	
	//
	output_off();
	
	HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if(need_fresh){
          need_fresh = 0;
//          TIM_ConfigFrequencyOptimized(freq / 10.0f, pulse);
		  Change_Mode();
      }
	  disp_task();
	  HAL_Delay(10);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 15;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_4;
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
  htim1.Init.Period = 6000;
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
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 60000-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DISP_F_Pin|DISP_D_Pin|DISP_C_Pin|DISP_J_Pin
                          |DISP_H_Pin|DISP_K_Pin|DISP_G1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DISP_DIG2_Pin|DISP_DIG1_Pin|DISP_DIG6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DISP_DP_Pin|DISP_N_Pin|DISP_B_Pin|DISP_M_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DISP_A_Pin|DISP_E_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DISP_DIG3_GPIO_Port, DISP_DIG3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DISP_DIG4_Pin|DISP_DIG5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DISP_L_Pin|DISP_G2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : key_output_Pin key_pulse_Pin key_add_Pin */
  GPIO_InitStruct.Pin = key_output_Pin|key_pulse_Pin|key_add_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DISP_F_Pin DISP_D_Pin DISP_DIG2_Pin DISP_C_Pin
                           DISP_J_Pin DISP_H_Pin DISP_DIG1_Pin DISP_K_Pin
                           DISP_G1_Pin DISP_DIG6_Pin */
  GPIO_InitStruct.Pin = DISP_F_Pin|DISP_D_Pin|DISP_DIG2_Pin|DISP_C_Pin
                          |DISP_J_Pin|DISP_H_Pin|DISP_DIG1_Pin|DISP_K_Pin
                          |DISP_G1_Pin|DISP_DIG6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DISP_DP_Pin DISP_DIG3_Pin DISP_N_Pin DISP_B_Pin
                           DISP_M_Pin */
  GPIO_InitStruct.Pin = DISP_DP_Pin|DISP_DIG3_Pin|DISP_N_Pin|DISP_B_Pin
                          |DISP_M_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DISP_A_Pin DISP_E_Pin */
  GPIO_InitStruct.Pin = DISP_A_Pin|DISP_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DISP_DIG4_Pin DISP_L_Pin DISP_DIG5_Pin DISP_G2_Pin */
  GPIO_InitStruct.Pin = DISP_DIG4_Pin|DISP_L_Pin|DISP_DIG5_Pin|DISP_G2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : key_sub_Pin key_volt_Pin key_move_Pin key_freq_Pin */
  GPIO_InitStruct.Pin = key_sub_Pin|key_volt_Pin|key_move_Pin|key_freq_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/**
  * @brief  Enables or disables the TIM Capture Compare Channel xN.
  * @param  TIMx to select the TIM peripheral
  * @param  Channel specifies the TIM Channel
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1
  *            @arg TIM_CHANNEL_2: TIM Channel 2
  *            @arg TIM_CHANNEL_3: TIM Channel 3
  * @param  ChannelNState specifies the TIM Channel CCxNE bit new state.
  *          This parameter can be: TIM_CCxN_ENABLE or TIM_CCxN_Disable.
  * @retval None
  */
static void TIM_CCxNChannelCmd_mine(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState)
{
  uint32_t tmp;

  tmp = TIM_CCER_CC1NE << (Channel & 0x1FU); /* 0x1FU = 31 bits max shift */

  /* Reset the CCxNE Bit */
  TIMx->CCER &=  ~tmp;

  /* Set or reset the CCxNE Bit */
  TIMx->CCER |= (uint32_t)(ChannelNState << (Channel & 0x1FU)); /* 0x1FU = 31 bits max shift */
}


/**
  * @brief  Starts the PWM signal generation.
  * @param  htim TIM handle
  * @param  Channel TIM Channels to be enabled
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  *            @arg TIM_CHANNEL_5: TIM Channel 5 selected
  *            @arg TIM_CHANNEL_6: TIM Channel 6 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIM_PWM_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  uint32_t tmpsmcr;

  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

  /* Check the TIM channel state */
  if (TIM_CHANNEL_STATE_GET(htim, Channel) != HAL_TIM_CHANNEL_STATE_READY)
  {
    return HAL_ERROR;
  }
	
  /* Check the TIM complementary channel state */
  if (TIM_CHANNEL_N_STATE_GET(htim, Channel) != HAL_TIM_CHANNEL_STATE_READY)
  {
    return HAL_ERROR;
  }

  /* Set the TIM channel state */
  TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);
	
	/* Set the TIM complementary channel state */
  TIM_CHANNEL_N_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);

  /* Enable the Capture compare channel */
  TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);
	TIM_CCxNChannelCmd_mine(htim->Instance, Channel, TIM_CCxN_ENABLE);
	
	__HAL_TIM_MOE_ENABLE(htim);

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TIM_SLAVE_INSTANCE(htim->Instance))
  {
    tmpsmcr = htim->Instance->SMCR & TIM_SMCR_SMS;
    if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __HAL_TIM_ENABLE(htim);
    }
  }
  else
  {
    __HAL_TIM_ENABLE(htim);
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Stops the PWM signal generation.
  * @param  htim TIM PWM handle
  * @param  Channel TIM Channels to be disabled
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  *            @arg TIM_CHANNEL_5: TIM Channel 5 selected
  *            @arg TIM_CHANNEL_6: TIM Channel 6 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIM_PWM_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

  /* Disable the Capture compare channel */
	TIM_CCxNChannelCmd_mine(htim->Instance, Channel, TIM_CCxN_DISABLE);
	
  TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_DISABLE);
	
  __HAL_TIM_MOE_DISABLE(htim);

  /* Disable the Peripheral */
  __HAL_TIM_DISABLE(htim);

  /* Set the TIM complementary channel state */
  TIM_CHANNEL_N_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_READY);

  /* Set the TIM channel state */
  TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_READY);

  /* Return function status */
  return HAL_OK;
}


/**
  * @brief  Starts the PWM signal generation in interrupt mode.
  * @param  htim TIM PWM handle
  * @param  Channel TIM Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIM_PWM_PWMN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t tmpsmcr;

  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

  /* Check the TIM channel state */
  if (TIM_CHANNEL_STATE_GET(htim, Channel) != HAL_TIM_CHANNEL_STATE_READY)
  {
    return HAL_ERROR;
  }
	
  /* Check the TIM complementary channel state */
  if (TIM_CHANNEL_N_STATE_GET(htim, Channel) != HAL_TIM_CHANNEL_STATE_READY)
  {
    return HAL_ERROR;
  }
	
  /* Set the TIM channel state */
  TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);
	
  /* Set the TIM complementary channel state */
  TIM_CHANNEL_N_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);

  switch (Channel)
  {
    case TIM_CHANNEL_1:
    {
      /* Enable the TIM Capture/Compare 1 interrupt */
      __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC1);
      break;
    }

    case TIM_CHANNEL_2:
    {
      /* Enable the TIM Capture/Compare 2 interrupt */
      __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC2);
      break;
    }

    case TIM_CHANNEL_3:
    {
      /* Enable the TIM Capture/Compare 3 interrupt */
      __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC3);
      break;
    }

    case TIM_CHANNEL_4:
    {
      /* Enable the TIM Capture/Compare 4 interrupt */
      __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC4);
      break;
    }

    default:
      status = HAL_ERROR;
      break;
  }

  if (status == HAL_OK)
  {
    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);

    /* Enable the complementary PWM output  */
    TIM_CCxNChannelCmd_mine(htim->Instance, Channel, TIM_CCxN_ENABLE);
    
		__HAL_TIM_MOE_ENABLE(htim);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_TIM_SLAVE_INSTANCE(htim->Instance))
    {
      tmpsmcr = htim->Instance->SMCR & TIM_SMCR_SMS;
      if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
      {
        __HAL_TIM_ENABLE(htim);
      }
    }
    else
    {
      __HAL_TIM_ENABLE(htim);
    }
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Stops the PWM signal generation in interrupt mode.
  * @param  htim TIM PWM handle
  * @param  Channel TIM Channels to be disabled
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIM_PWM_PWMN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));
	
  switch (Channel)
  {
    case TIM_CHANNEL_1:
    {
      /* Disable the TIM Capture/Compare 1 interrupt */
      __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
      break;
    }

    case TIM_CHANNEL_2:
    {
      /* Disable the TIM Capture/Compare 2 interrupt */
      __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);
      break;
    }

    case TIM_CHANNEL_3:
    {
      /* Disable the TIM Capture/Compare 3 interrupt */
      __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC3);
      break;
    }

    case TIM_CHANNEL_4:
    {
      /* Disable the TIM Capture/Compare 4 interrupt */
      __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC4);
      break;
    }

    default:
      status = HAL_ERROR;
      break;
  }

  if (status == HAL_OK)
  {
    /* Disable the complementary PWM output  */
    TIM_CCxNChannelCmd_mine(htim->Instance, Channel, TIM_CCxN_DISABLE);
		
    /* Disable the Capture compare channel */
    TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_DISABLE);
		
		/* Disable the Main Output */
		__HAL_TIM_MOE_DISABLE(htim);

    /* Disable the Peripheral */
    __HAL_TIM_DISABLE(htim);

    /* Set the TIM complementary channel state */
    TIM_CHANNEL_N_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_READY);

    /* Set the TIM channel state */
    TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_READY);
  }

  /* Return function status */
  return status;
}



/*LED DRIVE*/

void led_output(uint16_t data){
	
	if(data & 0x0001)DISP_A_GPIO_Port->BRR = (uint32_t)DISP_A_Pin;
	else DISP_A_GPIO_Port->BSRR = (uint32_t)DISP_A_Pin;
	
	if(data & 0x0002)DISP_B_GPIO_Port->BRR = (uint32_t)DISP_B_Pin;
	else DISP_B_GPIO_Port->BSRR = (uint32_t)DISP_B_Pin;
	
	if(data & 0x0004)DISP_C_GPIO_Port->BRR = (uint32_t)DISP_C_Pin;
	else DISP_C_GPIO_Port->BSRR = (uint32_t)DISP_C_Pin;
	
	if(data & 0x0008)DISP_D_GPIO_Port->BRR = (uint32_t)DISP_D_Pin;
	else DISP_D_GPIO_Port->BSRR = (uint32_t)DISP_D_Pin;
	
	if(data & 0x0010)DISP_E_GPIO_Port->BRR = (uint32_t)DISP_E_Pin;
	else DISP_E_GPIO_Port->BSRR = (uint32_t)DISP_E_Pin;
	
	if(data & 0x0020)DISP_F_GPIO_Port->BRR = (uint32_t)DISP_F_Pin;
	else DISP_F_GPIO_Port->BSRR = (uint32_t)DISP_F_Pin;
	
	if(data & 0x0040)DISP_G1_GPIO_Port->BRR = (uint32_t)DISP_G1_Pin;
	else DISP_G1_GPIO_Port->BSRR = (uint32_t)DISP_G1_Pin;
	
	if(data & 0x0080)DISP_G2_GPIO_Port->BRR = (uint32_t)DISP_G2_Pin;
	else DISP_G2_GPIO_Port->BSRR = (uint32_t)DISP_G2_Pin;
	
	if(data & 0x0100)DISP_H_GPIO_Port->BRR = (uint32_t)DISP_H_Pin;
	else DISP_H_GPIO_Port->BSRR = (uint32_t)DISP_H_Pin;
	
	if(data & 0x0200)DISP_J_GPIO_Port->BRR = (uint32_t)DISP_J_Pin;
	else DISP_J_GPIO_Port->BSRR = (uint32_t)DISP_J_Pin;
	
	if(data & 0x0400)DISP_K_GPIO_Port->BRR = (uint32_t)DISP_K_Pin;
	else DISP_K_GPIO_Port->BSRR = (uint32_t)DISP_K_Pin;
	
	if(data & 0x0800)DISP_L_GPIO_Port->BRR = (uint32_t)DISP_L_Pin;
	else DISP_L_GPIO_Port->BSRR = (uint32_t)DISP_L_Pin;
	
	if(data & 0x1000)DISP_M_GPIO_Port->BRR = (uint32_t)DISP_M_Pin;
	else DISP_M_GPIO_Port->BSRR = (uint32_t)DISP_M_Pin;
	
	if(data & 0x2000)DISP_N_GPIO_Port->BRR = (uint32_t)DISP_N_Pin;
	else DISP_N_GPIO_Port->BSRR = (uint32_t)DISP_N_Pin;
	
	if(data & 0x4000)DISP_DP_GPIO_Port->BRR = (uint32_t)DISP_DP_Pin;
	else DISP_DP_GPIO_Port->BSRR = (uint32_t)DISP_DP_Pin;
	
}


void led_task(void){
	static uint8_t step = 0;
	
	switch(step){
		case 0:
			DISP_DIG6_GPIO_Port->BRR = (uint32_t)DISP_DIG6_Pin;
			led_output(g_ram[0]);
			DISP_DIG1_GPIO_Port->BSRR = (uint32_t)DISP_DIG1_Pin;
			step = 1;
			break;
		case 1:
			DISP_DIG1_GPIO_Port->BRR = (uint32_t)DISP_DIG1_Pin;
			led_output(g_ram[1]);
			DISP_DIG2_GPIO_Port->BSRR = (uint32_t)DISP_DIG2_Pin;
			step = 2;
			break;
		case 2:
			DISP_DIG2_GPIO_Port->BRR = (uint32_t)DISP_DIG2_Pin;
			led_output(g_ram[2]);
			DISP_DIG3_GPIO_Port->BSRR = (uint32_t)DISP_DIG3_Pin;
			step = 3;
			break;
		case 3:
			DISP_DIG3_GPIO_Port->BRR = (uint32_t)DISP_DIG3_Pin;
			led_output(g_ram[3]);
			DISP_DIG4_GPIO_Port->BSRR = (uint32_t)DISP_DIG4_Pin;
			step = 4;
			break;
		case 4:
			DISP_DIG4_GPIO_Port->BRR = (uint32_t)DISP_DIG4_Pin;
			led_output(g_ram[4]);
			DISP_DIG5_GPIO_Port->BSRR = (uint32_t)DISP_DIG5_Pin;
			step = 5;
			break;
		case 5:
			DISP_DIG5_GPIO_Port->BRR = (uint32_t)DISP_DIG5_Pin;
			led_output(g_ram[5]);
			DISP_DIG6_GPIO_Port->BSRR = (uint32_t)DISP_DIG6_Pin;
			step = 0;
			break;
		default:
			step = 0;
	}
}

volatile uint16_t adc_buffer[ADC_BUFFER_SIZE][ADC_CHANNEL_NUM] = {0};

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_calc();
}


void HAL_IncTick(void)
{
  uwTick += (uint32_t)uwTickFreq;
}

volatile double adc_value[ADC_CHANNEL_NUM] = {0};

volatile uint8_t dac_sta = DISABLE; 
volatile uint16_t dac_fb = 0;

void adc_calc(void){
    uint32_t temp0;
    uint32_t temp1;
    uint32_t temp2;
    uint32_t temp3;
    
    temp0 = 0;
    temp1 = 0;
    temp2 = 0;
    temp3 = 0;
    
    for(uint8_t i=0;i<ADC_CHANNEL_NUM;i++){
        temp0 += adc_buffer[i][0];
        temp1 += adc_buffer[i][1];
        temp2 += adc_buffer[i][2];
        temp3 += adc_buffer[i][3];
    }
		
    adc_value[0] = (double)(temp0 * 2.414) / (double)temp3;
    adc_value[1] = (double)(temp1 * 1212) / (double)temp3;
    adc_value[2] = (double)(temp2 * 1.212) / (double)temp3;
    
	if(dac_sta)
		dac_feedback();
}


void dac_set_volt(uint16_t volt_mv){
    uint32_t temp;
    if(volt_mv == 0){
        htimDAC.Instance->CCR1 = 0;
        dac_sta = DISABLE;
        return;
    }
    temp = (uint32_t)volt_mv*10000/1031;
    htimDAC.Instance->CCR1 = temp;
    
    dac_fb = volt_mv;
    dac_sta = ENABLE;
}

void dac_feedback(void){
    uint16_t temp;
	
	static uint8_t count = 0;
	static double volt_buffer = 0;
	
	count++;
	volt_buffer += adc_value[DAC_CH];
	
	if(count >= 50){
		temp = volt_buffer*20;
        if(dac_fb > temp + 1 && htimDAC.Instance->CCR1 < 10000){
			htimDAC.Instance->CCR1++;
        }
        else if(dac_fb + 1 < temp && htimDAC.Instance->CCR1 > 0){
			htimDAC.Instance->CCR1--;
        }
		count = 0;
		volt_buffer = 0;
	}
}

void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
	  if(0)break;
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
