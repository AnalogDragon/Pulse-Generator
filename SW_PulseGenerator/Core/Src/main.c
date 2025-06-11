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
#include <stdlib.h> 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define htimDAC htim16
#define htimPWM htim1
#define htimTIM htim17
#define htimSTP htim3

/*
发现的bug：
低速率下burst模式会有时不置位输出状态：用凑合的方式先解决了
*/

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


void disp_on(void);
void disp_task(void);
void dac_set_volt(uint16_t volt_mv);
void adc_calc(void);
void dac_feedback(void);
void disp_output_sta_task(void);
void key_output(void);
void set_output(uint8_t sta);
HAL_StatusTypeDef TIM_ConfigFrequencyOptimized(float desiredFreq, uint32_t pulseWidth_ns);
void KeyStaIn(uint8_t num, uint8_t sta);
HAL_StatusTypeDef TIM1_ReversePolarity(uint8_t reverse);
uint8_t disp_str_step(const uint16_t* str,uint8_t len);
void disp_num_hid(uint32_t num,uint8_t len,uint8_t pos,uint8_t e10,uint32_t hidd);
void disp_num(uint32_t num,uint8_t len,uint8_t pos,uint8_t e10);

void key_do_output(void);
void key_do_pulse(uint8_t press_long);
void key_do_volt(uint8_t press_long);
void key_do_freq(uint8_t press_long);
void key_do_move(uint8_t press_long);
void key_do_add(void);
void key_do_sub(void);
void key_do_negt(void);


extern uint8_t set_FREQ_unit;
extern uint8_t set_PULSE_unit;
extern uint8_t set_VOLT_unit;

extern int16_t set_FREQ_num;	//50.0Hz
extern int16_t set_PULSE_num;	//5.0uS
extern int16_t set_VOLT_num;	//500mV

extern uint8_t disp_set_unit;

extern uint8_t need_disp_str;
extern uint8_t need_disp_str_changed;
extern const uint16_t  *p_disp_str1;
extern const uint16_t  *p_disp_str2;
extern uint8_t  len_disp_str1;
extern uint8_t  len_disp_str2;

volatile uint8_t pwm_timer_lock = 0;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint16_t g_ram[6] = {0};
const uint16_t disp_output[6] = {DISP_CHAR_O,DISP_CHAR_U,DISP_CHAR_T,DISP_CHAR_P,DISP_CHAR_U,DISP_CHAR_T};
const uint16_t disp_normal_mode[11] = {DISP_CHAR_N,DISP_CHAR_O,DISP_CHAR_R,DISP_CHAR_M,DISP_CHAR_A,DISP_CHAR_L,0,DISP_CHAR_M,DISP_CHAR_O,DISP_CHAR_D,DISP_CHAR_E};
const uint16_t disp_singel_burst[12] = {DISP_CHAR_S,DISP_CHAR_I,DISP_CHAR_N,DISP_CHAR_G,DISP_CHAR_E,DISP_CHAR_L,0,DISP_CHAR_B,DISP_CHAR_U,DISP_CHAR_R,DISP_CHAR_S,DISP_CHAR_T};
const uint16_t disp_repeat_burst[12] = {DISP_CHAR_R,DISP_CHAR_E,DISP_CHAR_P,DISP_CHAR_E,DISP_CHAR_A,DISP_CHAR_T,0,DISP_CHAR_B,DISP_CHAR_U,DISP_CHAR_R,DISP_CHAR_S,DISP_CHAR_T};
const uint16_t disp_100_CPS[6] ={DISP_NUM_1,DISP_NUM_0,DISP_NUM_0,DISP_CHAR_C,DISP_CHAR_P,DISP_CHAR_S}; 
const uint16_t disp_100_count[6] ={DISP_NUM_1,DISP_NUM_0,DISP_NUM_0,DISP_CHAR_C,DISP_CHAR_N,DISP_CHAR_T}; 

const uint16_t disp_negative[6] ={DISP_CHAR_N,DISP_CHAR_E,DISP_CHAR_G,0x0001,0x003E,0x0001}; 
const uint16_t disp_positive[6] ={DISP_CHAR_P,DISP_CHAR_O,DISP_CHAR_S,0x0008,0x0037,0x0008}; 

const uint16_t disp_HZ[6] ={0,0,0,0,DISP_CHAR_H,DISP_CHAR_Z}; 
const uint16_t disp_KHZ[6] ={0,0,0,DISP_CHAR_K,DISP_CHAR_H,DISP_CHAR_Z}; 
const uint16_t disp_MHZ[6] ={0,0,0,DISP_CHAR_M,DISP_CHAR_H,DISP_CHAR_Z}; 
const uint16_t disp_NS[6] ={0,0,0,0,DISP_CHAR_n,DISP_CHAR_S}; 
const uint16_t disp_US[6] ={0,0,0,0,DISP_CHAR_u,DISP_CHAR_S}; 
const uint16_t disp_MV[6] ={0,0,0,0,DISP_CHAR_m,DISP_CHAR_V}; 
const uint16_t disp_V[6] ={0,0,0,0,0,DISP_CHAR_V}; 


const uint16_t disp_calib_HV[8] ={DISP_CHAR_C,DISP_CHAR_A,DISP_CHAR_L,DISP_CHAR_I,DISP_CHAR_B|0x4000,0,DISP_CHAR_H,DISP_CHAR_V};
const uint16_t disp_done[6] ={0,0,DISP_CHAR_D,DISP_CHAR_O,DISP_CHAR_N,DISP_CHAR_E}; 
const uint16_t disp_failed[12] ={DISP_CHAR_F,DISP_CHAR_A,DISP_CHAR_I,DISP_CHAR_L,DISP_CHAR_E,DISP_CHAR_D,0,DISP_CHAR_R,DISP_CHAR_E,DISP_CHAR_T,DISP_CHAR_R,DISP_CHAR_Y}; 


double hv_calib_gain = 1.0;

uint32_t hv_calib_num = 9000;


uint8_t cal_task(void){
	
	static uint8_t set_pos_count = 0;
	uint16_t set_pos_buf = 0;
	static uint8_t done[KEY_SIZE];
	static double volt_filter = 0;
	
	/*闪烁位*/
	if(set_pos_count < 25){
		set_pos_count++;
	}
	else{
		set_pos_count = 0;
	}
	if(set_pos_count > 20)
		set_pos_buf = set_pos;
	/*key*/
	
	/**/
	if(key_sta[KEY_ADD]){
		if(done[KEY_ADD] == 0 || done[KEY_ADD] == 50){
			if(hv_calib_num + set_pos < 40000)hv_calib_num += set_pos;
			else hv_calib_num = 40000;
		}
		done[KEY_ADD] ++;
		if(done[KEY_ADD] == 60)done[KEY_ADD] = 50;
	}
	else{
		done[KEY_ADD] = 0;
	}

	/**/
	if(key_sta[KEY_SUB]){
		if(done[KEY_SUB] == 0 || done[KEY_SUB] == 50){
			if(hv_calib_num - set_pos > 1000)hv_calib_num -= set_pos;
			else hv_calib_num = 1000;
		}
		done[KEY_SUB] ++;
		if(done[KEY_SUB] == 60)done[KEY_SUB] = 50;
	}
	else{
		done[KEY_SUB] = 0;
	}

	/**/
	if(key_sta[KEY_MOVE]){
		if(done[KEY_MOVE] == 0){
			switch (set_pos){
				case 10000:
					set_pos = 1000;
					break;
				case 1000:
					set_pos = 100;
					break;
				case 100:
					set_pos = 10;
					break;
				case 10:
					set_pos = 1;
					break;
				case 1:
				default:
					set_pos = 10000;
			}
		}
		if(done[KEY_MOVE] < 128)
			done[KEY_MOVE] ++;
	}
	else{
		done[KEY_MOVE] = 0;
	}
	
	memcpy(g_ram,disp_V,12);
	disp_num_hid(hv_calib_num,5,0,1,set_pos_buf);
	
	/**/
	if(key_sta[KEY_OUTPUT]){
		
		volt_filter = volt_filter * 0.99 + adc_value[HV_CH]*0.01;
		
		if(done[KEY_OUTPUT] == 250){
			hv_calib_gain = hv_calib_num / volt_filter / 10;
			return 1;
		}
		if(done[KEY_OUTPUT] < 250)
			done[KEY_OUTPUT] ++;
		for(uint8_t i=0;i<6;i++){
			if(done[KEY_OUTPUT] > i*40){
				g_ram[i] = rand();
			}
		}
	}
	else{
		done[KEY_OUTPUT] = 0;
		volt_filter = adc_value[HV_CH];
	}
	return 0;
}

uint8_t  save_cal(void){
  uint32_t PageError = 0; 
	
  FLASH_EraseInitTypeDef FLASH_EraseInitType = {
	FLASH_TYPEERASE_PAGES,0,FLASH_PAGE_SAVE,1
	};
  
  HAL_FLASH_Unlock();
  
  HAL_FLASHEx_Erase(&FLASH_EraseInitType, &PageError);
  
  if(0xFFFFFFFF != PageError){
    //Erase Failed!
	  HAL_FLASH_Lock();
	  return 1;
  }
	// 写入 Flash
	uint64_t rawData;
	memcpy(&rawData, &hv_calib_gain, sizeof(double));
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_ADDR_SAVE, rawData) != HAL_OK) {
		HAL_FLASH_Lock();
		return 2;  // 写入失败
	}
	
  HAL_FLASH_Lock();
  return 0;
}

void load_cal(void) {
    uint64_t *pData = (uint64_t*)FLASH_ADDR_SAVE;
    if (*pData == 0xFFFFFFFF) {
        hv_calib_gain = 1.0;  // 默认值（Flash 未写入）
    } else {
        memcpy(&hv_calib_gain, pData, sizeof(double));
    }
}

void voltage_cal(void){
	while(!disp_str_step(disp_calib_HV,sizeof(disp_calib_HV)/2))
		HAL_Delay(100);
	
	set_pos = 10000;
	hv_calib_gain = 1.0;
	
	while(!cal_task()){
		HAL_Delay(10);
	}
	
	if(save_cal()){
		while(!disp_str_step(disp_failed,sizeof(disp_failed)/2))
			HAL_Delay(100);
	}
	else{
		while(!disp_str_step(disp_done,sizeof(disp_done)/2))
			HAL_Delay(100);
	}
	
	HAL_NVIC_SystemReset();
}




/*
频率范围：
normal：000.1Hz~999.9Hz，001KHZ~999KHZ，0.00MHZ~5.00MHZ
burst：001KHZ~999KHZ，0.01MHZ~5.00MHZ
电压范围：
0005mV~1000mV
脉宽范围：
000.1uS~500.0uS,0050nS~9999nS
除此之外还需要计算频率范围锁
*/



void set_limit(void){
	
	//电压限制：0005mV~1000mV
	if(set_VOLT_unit == SET_VOLT_MV){
		if(set_VOLT_num < 5){
			set_VOLT_num = 5;
		}
		else if(set_VOLT_num > 1000){
			set_VOLT_num = 1000;
		}
	}
	else if(set_VOLT_unit == SET_VOLT_V){
	}
	else{
		set_VOLT_unit = SET_VOLT_MV;
		set_VOLT_num = 500;
	}
	
	if(output_mode == MODE_NORMAL){
		//频率限制：
		//normal：000.1Hz~999.9Hz，001KHZ~999KHZ，0.01MHZ~5.00MHZ
		if(set_FREQ_unit == SET_FREQ_HZ){
			if(set_FREQ_num < 1){
				set_FREQ_num = 1;
			}
			else if(set_FREQ_num > 9999){
				set_FREQ_num = 9999;
			}
		}
		else if(set_FREQ_unit == SET_FREQ_KHZ0 
			|| set_FREQ_unit == SET_FREQ_KHZ1 
			|| set_FREQ_unit == SET_FREQ_KHZ2){
			if(set_FREQ_num < 1){
				set_FREQ_num = 1;
			}
			else if(set_FREQ_num > 999){
				set_FREQ_num = 999;
			}
		}
		else if(set_FREQ_unit == SET_FREQ_MHZ){
			if(set_FREQ_num < 1){
				set_FREQ_num = 1;
			}
			else if(set_FREQ_num > 500){
				set_FREQ_num = 500;
			}
		}
		else{
			set_FREQ_unit = SET_FREQ_HZ;
			set_FREQ_num = 50;
		}
		//脉宽限制 000.1uS~500.0uS,0050nS~9999nS
		if(set_PULSE_unit == SET_PULSE_NS){
			if(set_PULSE_num < 50){
				set_PULSE_num = 50;
			}
			else if(set_PULSE_num > 9950){
				set_PULSE_num = 9950;
			}
		}
		else if(set_PULSE_unit == SET_PULSE_US){
			if(set_PULSE_num < 1){
				set_PULSE_num = 1;
			}
			else if(set_PULSE_num > 5000){
				set_PULSE_num = 5000;
			}
		}
		else{
			set_PULSE_unit = SET_PULSE_US;
			set_PULSE_num = 50;
		}
	}
	else if(output_mode == MODE_REPEAT_BURST || output_mode == MODE_SINGEL_BURST){
		//频率限制：
		//burst：001KHZ~999KHZ，0.01MHZ~5.00MHZ
		if(set_FREQ_unit == SET_FREQ_HZ){
			set_FREQ_unit = SET_FREQ_KHZ2;
			set_FREQ_num = 10;
			if(disp_set_unit == SET_FREQ_HZ){
				disp_set_unit = SET_FREQ_KHZ2;
			}
		}
		else if(set_FREQ_unit == SET_FREQ_KHZ0){
			if(set_FREQ_num < 100){
				set_FREQ_num = 100;
			}
			else if(set_FREQ_num > 999){
				set_FREQ_num = 999;
			}
		}
		else if(set_FREQ_unit == SET_FREQ_KHZ1){
			if(set_FREQ_num < 10){
				set_FREQ_num = 10;
			}
			else if(set_FREQ_num > 999){
				set_FREQ_num = 999;
			}
		}
		else if(set_FREQ_unit == SET_FREQ_KHZ2){
			if(set_FREQ_num < 1){
				set_FREQ_num = 1;
			}
			else if(set_FREQ_num > 999){
				set_FREQ_num = 999;
			}
		}
		else if(set_FREQ_unit == SET_FREQ_MHZ){
			if(set_FREQ_num < 1){
				set_FREQ_num = 1;
			}
			else if(set_FREQ_num > 500){
				set_FREQ_num = 500;
			}
		}
		else{
			set_FREQ_unit = SET_FREQ_KHZ2;
			set_FREQ_num = 10;
		}
		//脉宽限制 000.1uS~500.0uS,0050nS~9999nS
		if(set_PULSE_unit == SET_PULSE_NS){
			if(set_PULSE_num < 50){
				set_PULSE_num = 50;
			}
			else if(set_PULSE_num > 9950){
				set_PULSE_num = 9950;
			}
		}
		else if(set_PULSE_unit == SET_PULSE_US){
			if(set_PULSE_num < 1){
				set_PULSE_num = 1;
			}
			else if(set_PULSE_num > 5000){
				set_PULSE_num = 5000;
			}
		}
		else{
			set_PULSE_unit = SET_PULSE_US;
			set_PULSE_num = 50;
		}
	}
	else{
		output_mode = MODE_NORMAL;
		
		need_disp_str_changed = 1;
		need_disp_str = 1;
	
		p_disp_str1 = disp_normal_mode;
		len_disp_str1 = sizeof(disp_normal_mode)/2;
		
		set_FREQ_unit = SET_FREQ_HZ;
		set_PULSE_unit = SET_PULSE_US;
		set_VOLT_unit = SET_VOLT_MV;

		set_FREQ_num = 500;	//50.0Hz
		set_PULSE_num = 50;	//5.0uS
		set_VOLT_num = 500;	//500mV
	}
	
	//计算脉宽不能低于1/2周期
	uint32_t pulse;
	uint32_t freq;
	if(set_FREQ_unit == SET_FREQ_HZ){//000.1Hz~999.9Hz
		pulse = 500000;
	}
	else if(set_FREQ_unit == SET_FREQ_KHZ0){//0.01KHZ~9.99KHZ
		freq = set_FREQ_num;
		pulse = 100000000 / freq / 2;	//ns
	}
	else if(set_FREQ_unit == SET_FREQ_KHZ1){//00.1KHZ~99.9KHZ
		freq = set_FREQ_num;
		pulse = 10000000 / freq / 2;	//ns
	}
	else if(set_FREQ_unit == SET_FREQ_KHZ2){//001KHZ~999KHZ
		freq = set_FREQ_num;
		pulse = 1000000 / freq / 2;	//ns
	}
	else if(set_FREQ_unit == SET_FREQ_MHZ){//0.01MHZ~5.00MHZ
		freq = set_FREQ_num*10;
		pulse = 1000000 / freq / 2;	//ns
	}
    
    if(set_PULSE_unit == SET_PULSE_US){
        pulse /= 100;
    }
    if(set_PULSE_num > pulse){
        set_PULSE_num = pulse;
    }
	
}

/*
调用后循环切换mode
*/
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
	//切换mode之后需要限制一次limit
	set_limit();
}

#define KEY_SIZE 7
#define KEY_TIME_TRIG	10

uint8_t press_time[KEY_SIZE] = {0};
uint8_t release_time[KEY_SIZE] = {0};
uint8_t key_sta[KEY_SIZE] = {0};


void KeyStaIn(uint8_t num, uint8_t sta){
	if(num >= KEY_SIZE)return;
	if(sta != DISABLE){
		if(press_time[num] > KEY_TIME_TRIG){
			release_time[num] = 0;
			key_sta[num] = 1;
		}
		else{
			press_time[num] ++;
		}
	}
	else{
		if(release_time[num] > KEY_TIME_TRIG){
			press_time[num] = 0;
			key_sta[num] = 0;
		}
		else{
			release_time[num] ++;
		}
	}
}

uint8_t output_negative = DISABLE;

void out_put_negitve(uint8_t sta){
	
	if(sta == ENABLE && output_negative == DISABLE){
		output_negative = ENABLE;
	
		need_disp_str_changed = 1;
		need_disp_str = 1;

		p_disp_str1 = disp_negative;
		len_disp_str1 = sizeof(disp_negative)/2;
		
		TIM1_ReversePolarity(output_negative);
	}
	else if(sta == DISABLE && output_negative == ENABLE){
		output_negative = DISABLE;
	
		need_disp_str_changed = 1;
		need_disp_str = 1;

		p_disp_str1 = disp_positive;
		len_disp_str1 = sizeof(disp_positive)/2;
		
		TIM1_ReversePolarity(output_negative);
	}
	
}


//key_output();
//Change_Mode();
//

void Key_Task(void){
	static uint8_t done[7] = {0};
	static uint16_t count_set = 0;
	
	if(set_pos)
		count_set ++;
	else
		count_set = 0;
	
	if(count_set > 1000){
		count_set = 0;
		set_pos = 0;
	}
	
	/**/
	if(key_sta[KEY_OUTPUT]){
		count_set = 0;
		if(done[KEY_OUTPUT] == 0){
			key_do_output();
			done[KEY_OUTPUT] = 1;
		}
	}
	else{
		done[KEY_OUTPUT] = 0;
	}
	
	/**/
	if(key_sta[KEY_PULSE]){
		count_set = 0;
		if(done[KEY_PULSE] == 0){
			key_do_pulse(0);
		}
		if(done[KEY_PULSE] == 60){
			key_do_pulse(1);
		}
		if(done[KEY_PULSE] < 128)
			done[KEY_PULSE] ++;
	}
	else{
		done[KEY_PULSE] = 0;
	}
	
	/**/
	if(key_sta[KEY_FREQ]){
		count_set = 0;
		if(done[KEY_FREQ] == 0){
			key_do_freq(0);
		}
		if(done[KEY_FREQ] == 60){
			key_do_freq(1);
		}
		if(done[KEY_FREQ] < 128)
			done[KEY_FREQ] ++;
	}
	else{
		done[KEY_FREQ] = 0;
	}
	
	/**/
	if(key_sta[KEY_VOLT]){
		count_set = 0;
		if(done[KEY_VOLT] == 0){
			key_do_volt(0);
		}
		if(done[KEY_VOLT] == 60){
			key_do_volt(1);
		}
		if(done[KEY_VOLT] < 128)
			done[KEY_VOLT] ++;
	}
	else{
		done[KEY_VOLT] = 0;
	}
	
	/**/
	if(done[KEY_ADD] == 0xFF || done[KEY_SUB] == 0xFF || done[KEY_MOVE] == 0xFF){
		if(key_sta[KEY_SUB] == 0 && key_sta[KEY_ADD] == 0 && key_sta[KEY_MOVE] == 0){
			done[KEY_ADD] = 0;
			done[KEY_SUB] = 0;
			done[KEY_MOVE] = 0;
		}
	}
	else{
		
		/**/
		if(key_sta[KEY_ADD]){
			count_set = 0;
			if(done[KEY_ADD] == 0 || done[KEY_ADD] == 50)
				key_do_add();
			done[KEY_ADD] ++;
			if(done[KEY_ADD] == 60)done[KEY_ADD] = 50;
		}
		else{
			done[KEY_ADD] = 0;
		}
	
		/**/
		if(key_sta[KEY_SUB]){
			count_set = 0;
			if(done[KEY_SUB] == 0 || done[KEY_SUB] == 50)
				key_do_sub();
			done[KEY_SUB] ++;
			if(done[KEY_SUB] == 60)done[KEY_SUB] = 50;
		}
		else{
			done[KEY_SUB] = 0;
		}
	
		/**/
		if(key_sta[KEY_MOVE]){
			count_set = 0;
			if(done[KEY_MOVE] == 0){
				key_do_move(0);
			}
			if(done[KEY_MOVE] == 60){
				key_do_move(1);
			}
			if(done[KEY_MOVE] < 128)
				done[KEY_MOVE] ++;
		}
		else{
			done[KEY_MOVE] = 0;
		}
		
		if(key_sta[KEY_SUB] && key_sta[KEY_MOVE]){
			//负脉冲输出切换
			count_set = 0;
			done[KEY_MOVE] = 0xFF;
			done[KEY_SUB] = 0xFF;
			set_output(DISABLE);
			out_put_negitve(ENABLE);
		}
		
		if(key_sta[KEY_ADD] && key_sta[KEY_MOVE]){
			//负脉冲输出切换
			count_set = 0;
			done[KEY_MOVE] = 0xFF;
			done[KEY_ADD] = 0xFF;
			set_output(DISABLE);
			out_put_negitve(DISABLE);
		}
	}
	
	//约束数值：
	set_limit();
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
	
	//DAC 输出
	HAL_TIM_PWM_Start(&htimDAC, TIM_CHANNEL_1);
	
	//自动关闭输出中断
	HAL_TIM_Base_Start_IT(&htimSTP);
	
	//
	set_output(DISABLE);
	
    disp_on();
	
	if(key_sta[KEY_VOLT] && key_sta[KEY_MOVE]){
		voltage_cal();
	}
	else{
		load_cal();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Key_Task();
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
  htim14.Init.Period = 3000;
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




/**
 * @brief  重新配置 TIM1 输出极性（主通道和互补通道）
 * @param  htim: TIM1 的句柄（TIM_HandleTypeDef*）
 * @param  reverse: true=反向（低有效），false=正常（高有效）
 * @retval HAL 状态（HAL_OK / HAL_ERROR）
 */
HAL_StatusTypeDef TIM1_ReversePolarity(uint8_t reverse) {
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};


	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 100;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    // 根据 reverse 参数设置极性
    if (reverse == ENABLE) {
        sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;    // 主输出低有效
        sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;  // 互补输出低有效
        sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
        sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
        GPIO_InitStruct.Pin = GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
    }
    else {
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;    // 主输出高有效
        sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;  // 互补输出高有效
        sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
        sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
        GPIO_InitStruct.Pin = GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
    }

    // 重新配置 PWM 通道（仅更新极性）
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}



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
    
  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

//  /* Check the TIM channel state */
//  if (TIM_CHANNEL_STATE_GET(htim, Channel) != HAL_TIM_CHANNEL_STATE_READY)
//  {
//    return HAL_ERROR;
//  }
//	
//  /* Check the TIM complementary channel state */
//  if (TIM_CHANNEL_N_STATE_GET(htim, Channel) != HAL_TIM_CHANNEL_STATE_READY)
//  {
//    return HAL_ERROR;
//  }

//  /* Set the TIM channel state */
//  TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);
//	
//	/* Set the TIM complementary channel state */
//  TIM_CHANNEL_N_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);

  /* Enable the Capture compare channel */
  TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);
	TIM_CCxNChannelCmd_mine(htim->Instance, Channel, TIM_CCxN_ENABLE);
	
//	__HAL_TIM_MOE_ENABLE(htim);
    htim->Instance->BDTR |= TIM_BDTR_MOE;
    htim->Instance->CR1 |= (TIM_CR1_CEN);

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
//  if (IS_TIM_SLAVE_INSTANCE(htim->Instance))
//  {
//    tmpsmcr = htim->Instance->SMCR & TIM_SMCR_SMS;
//    if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
//    {
//      __HAL_TIM_ENABLE(htim);
//    }
//  }
//  else
//  {
//    __HAL_TIM_ENABLE(htim);
//  }

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
	
//  __HAL_TIM_MOE_DISABLE(htim);
	  htim->Instance->BDTR &= ~(TIM_BDTR_MOE);
	  htim->Instance->CR1 &= ~(TIM_CR1_CEN);

  /* Disable the Peripheral */
//  __HAL_TIM_DISABLE(htim);
	
//  /* Set the TIM complementary channel state */
//  TIM_CHANNEL_N_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_READY);

//  /* Set the TIM channel state */
//  TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_READY);

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

  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

//  /* Check the TIM channel state */
//  if (TIM_CHANNEL_STATE_GET(htim, Channel) != HAL_TIM_CHANNEL_STATE_READY)
//  {
//    return HAL_ERROR;
//  }
//	
//  /* Check the TIM complementary channel state */
//  if (TIM_CHANNEL_N_STATE_GET(htim, Channel) != HAL_TIM_CHANNEL_STATE_READY)
//  {
//    return HAL_ERROR;
//  }
//	
//  /* Set the TIM channel state */
//  TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);
//	
//  /* Set the TIM complementary channel state */
//  TIM_CHANNEL_N_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);

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
    
//		__HAL_TIM_MOE_ENABLE(htim);
	htim->Instance->BDTR |= TIM_BDTR_MOE;
	htim->Instance->CR1 |= (TIM_CR1_CEN);

//    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
//    if (IS_TIM_SLAVE_INSTANCE(htim->Instance))
//    {
//      tmpsmcr = htim->Instance->SMCR & TIM_SMCR_SMS;
//      if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
//      {
//        __HAL_TIM_ENABLE(htim);
//      }
//    }
//    else
//    {
//      __HAL_TIM_ENABLE(htim);
//    }
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
	  htim->Instance->BDTR &= ~(TIM_BDTR_MOE);
	  htim->Instance->CR1 &= ~(TIM_CR1_CEN);
//		__HAL_TIM_MOE_DISABLE(htim);

    /* Disable the Peripheral */
//    __HAL_TIM_DISABLE(htim);

//    /* Set the TIM complementary channel state */
//    TIM_CHANNEL_N_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_READY);

//    /* Set the TIM channel state */
//    TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_READY);
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

void disp_num_hid(uint32_t num,uint8_t len,uint8_t pos,uint8_t e10,uint32_t hidd){
	uint16_t temp[6];
	
	if(pos >= 6 || e10 >= 6 || len > 6)
		return;
	
	temp[0] = disp_number_table[num/100000%10];
	temp[1] = disp_number_table[num/10000%10];
	temp[2] = disp_number_table[num/1000%10];
	temp[3] = disp_number_table[num/100%10];
	temp[4] = disp_number_table[num/10%10];
	temp[5] = disp_number_table[num%10];
	if(hidd == 100000)temp[0] = 0;
	if(hidd == 10000)temp[1] = 0;
	if(hidd == 1000)temp[2] = 0;
	if(hidd == 100)temp[3] = 0;
	if(hidd == 10)temp[4] = 0;
	if(hidd == 1)temp[5] = 0;
	
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



uint8_t set_FREQ_unit = SET_FREQ_HZ;
uint8_t set_PULSE_unit = SET_PULSE_US;
uint8_t set_VOLT_unit = SET_VOLT_MV;

int16_t set_FREQ_num = 500;	//50.0Hz
int16_t set_PULSE_num = 50;	//5.0uS
int16_t set_VOLT_num = 500;	//500mV

uint8_t disp_set_unit = SET_FREQ_HZ;

uint8_t need_disp_str = 0;
uint8_t need_disp_str_changed = 0;
const uint16_t  *p_disp_str1;
const uint16_t  *p_disp_str2;
uint8_t  len_disp_str1;
uint8_t  len_disp_str2;

uint16_t set_pos = 0;

void disp_task(void){
	static uint16_t str_timer = 0;
	static uint8_t high_voltage_counter = 0;
	static double high_voltage_filter = 0;
	static double high_voltage_out = 0;
	static uint8_t set_pos_count = 0;
	uint16_t set_pos_buf = 0;
	
	
	/*显示电压频率降低*/
	if(high_voltage_counter < 20){
		high_voltage_filter += adc_value[HV_CH];
		high_voltage_counter++;
	}
	else{
		high_voltage_out = high_voltage_filter/20+0.5;
		high_voltage_filter = 0;
		high_voltage_counter = 0;
	}
	
	/*闪烁位*/
	if(set_pos_count < 25){
		set_pos_count++;
	}
	else{
		set_pos_count = 0;
	}
	if(set_pos_count > 20)
		set_pos_buf = set_pos;
	
	
	/*显示提示，最多支持*/
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
	
	/*正常显示参数*/
    switch(disp_set_unit){
        case SET_FREQ_HZ:
			memcpy(g_ram,disp_HZ,12);
			disp_num_hid(set_FREQ_num,4,0,1,set_pos_buf);
            break;
		
        case SET_FREQ_KHZ0:
			memcpy(g_ram,disp_KHZ,12);
			disp_num_hid(set_FREQ_num,3,0,2,set_pos_buf);
            break;
		
        case SET_FREQ_KHZ1:
			memcpy(g_ram,disp_KHZ,12);
			disp_num_hid(set_FREQ_num,3,0,1,set_pos_buf);
            break;
		
        case SET_FREQ_KHZ2:
			memcpy(g_ram,disp_KHZ,12);
			disp_num_hid(set_FREQ_num,3,0,0,set_pos_buf);
            break;
		
        case SET_FREQ_MHZ:
			memcpy(g_ram,disp_MHZ,12);
			disp_num_hid(set_FREQ_num,3,0,2,set_pos_buf);
            break;
		
        case SET_PULSE_NS:
			memcpy(g_ram,disp_NS,12);
			disp_num_hid(set_PULSE_num,4,0,0,set_pos_buf);
            break;
		
        case SET_PULSE_US:
			memcpy(g_ram,disp_US,12);
			disp_num_hid(set_PULSE_num,4,0,1,set_pos_buf);
            break;
		
        case SET_VOLT_MV:
			memcpy(g_ram,disp_MV,12);
			disp_num_hid(set_VOLT_num,4,0,0,set_pos_buf);
            break;
		
        case SET_VOLT_V:
			memcpy(g_ram,disp_V,12);
			if(high_voltage_out < 10)
				disp_num(high_voltage_out,1,3,0);
			else if(high_voltage_out < 100)
				disp_num(high_voltage_out,2,2,0);
			else if(high_voltage_out < 1000)
				disp_num(high_voltage_out,3,1,0);
			else
				disp_num(high_voltage_out,4,0,0);
            break;
		
        default:
			;
    }
	
	
	disp_output_sta_task();
}

/*DAC 驱动*/

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
    adc_value[1] = (double)(temp1 * 1212) * hv_calib_gain / (double)temp3;
    adc_value[2] = (double)(temp2 * 1.212) / (double)temp3;
    
	if(dac_sta){
		//adc闭环函数
		dac_feedback();
	}
}


void dac_set_volt(uint16_t volt_mv){
	static uint16_t volt_mv_bak = 0;
    uint32_t temp;
	
	//记录变化
	if(volt_mv == volt_mv_bak){
		return;
	}
	volt_mv_bak = volt_mv;
	
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


/*开启输出*/
uint8_t output_sta = DISABLE;

void set_output(uint8_t sta){
	if(sta == DISABLE){
//		dac_set_volt(0);
		HAL_TIM_Base_Stop_IT(&htimTIM);
		HAL_TIM_PWM_PWMN_Stop(&htimPWM, TIM_CHANNEL_1);
		HAL_TIM_PWM_PWMN_Stop_IT(&htimPWM, TIM_CHANNEL_1);
		output_sta = DISABLE;
	}
	else if(sta == ENABLE){
		//设置输出mV数字
		dac_set_volt(set_VOLT_num);
        HAL_Delay(10);
		//设置频率输出
		uint32_t freq;
		uint32_t pulse;
		
		if(set_FREQ_unit == SET_FREQ_KHZ0){
			freq = set_FREQ_num*100;
		}
		else if(set_FREQ_unit == SET_FREQ_KHZ1){
			freq = set_FREQ_num*1000;
		}
		else if(set_FREQ_unit == SET_FREQ_KHZ2){
			freq = set_FREQ_num*10000;
		}
		else if(set_FREQ_unit == SET_FREQ_MHZ){
			freq = set_FREQ_num*100000;
		}
		else{
			freq = set_FREQ_num;
		}
		
		if(set_PULSE_unit == SET_PULSE_US){//us
			pulse = set_PULSE_num*100;
		}
		else{//ns
			pulse = set_PULSE_num;
		}
		
		TIM_ConfigFrequencyOptimized((float)freq / 10.0f, pulse);
        if(output_mode != MODE_SINGEL_BURST)
            output_sta = ENABLE;
	}
}


//10ms
void disp_output_sta_task(void){
	static uint8_t counter = 0;
	if(output_sta == ENABLE){
		counter++;
		if(counter < 5)g_ram[5] |= 0x4000;
		else g_ram[5] &= ~0x4000;
		if(counter >= 10)
			counter = 0;
	}
	else{
		g_ram[5] &= ~0x4000;
		counter = 0;
	}
}

void key_do_output(void){
	if(output_sta == DISABLE){
		set_pos = 0;
		set_output(ENABLE);
	}
	else
		set_output(DISABLE);
}

void key_do_pulse(uint8_t press_long){
	
	if(press_long){
		//如果是长按，切换单位
		if(set_PULSE_unit == SET_PULSE_NS){
			 set_PULSE_unit = SET_PULSE_US;
		}
		else if(set_PULSE_unit == SET_PULSE_US){
			 set_PULSE_unit = SET_PULSE_NS;
		}
		if(output_sta == ENABLE)
			set_output(DISABLE);
	}
	
	if(disp_set_unit != set_PULSE_unit){
		disp_set_unit = set_PULSE_unit;
		set_pos = 0;
	}
}

void key_do_volt(uint8_t press_long){
	
	if(press_long){
		if(set_VOLT_unit == SET_VOLT_MV){
			 set_VOLT_unit = SET_VOLT_V;
		}
		else if(set_VOLT_unit == SET_VOLT_V){
			 set_VOLT_unit = SET_VOLT_MV;
		}
//		if(output_sta == ENABLE)
//			set_output(DISABLE);
	}
	
	if(disp_set_unit != set_VOLT_unit){
		disp_set_unit = set_VOLT_unit;
		set_pos = 0;
	}
}

void key_do_freq(uint8_t press_long){
	if(press_long){
		if(set_FREQ_unit == SET_FREQ_HZ){
			 set_FREQ_unit = SET_FREQ_KHZ0;
		}
		else if(set_FREQ_unit == SET_FREQ_KHZ0){
			 set_FREQ_unit = SET_FREQ_KHZ1;
		}
		else if(set_FREQ_unit == SET_FREQ_KHZ1){
			 set_FREQ_unit = SET_FREQ_KHZ2;
		}
		else if(set_FREQ_unit == SET_FREQ_KHZ2){
			 set_FREQ_unit = SET_FREQ_MHZ;
		}
		else if(set_FREQ_unit == SET_FREQ_MHZ){
			 set_FREQ_unit = SET_FREQ_HZ;
		}
		if(output_sta == ENABLE)
			set_output(DISABLE);
	}
	
	if(disp_set_unit != set_FREQ_unit){
		disp_set_unit = set_FREQ_unit;
		set_pos = 0;
	}
}

void key_do_move(uint8_t press_long){
	if(press_long){
		Change_Mode();
		set_output(DISABLE);
		set_pos = 0;
	}
	else{
		switch (set_pos){
			case 1000:
				set_pos = 100;
				break;
			case 100:
				set_pos = 10;
				break;
			case 10:
				set_pos = 1;
				break;
			case 1:
			default:
				set_pos = 1000;
				if(disp_set_unit == SET_FREQ_KHZ0
					||disp_set_unit == SET_FREQ_KHZ1
					||disp_set_unit == SET_FREQ_KHZ2
					|| disp_set_unit == SET_FREQ_MHZ)
					set_pos = 100;
		}
	}
}

void key_do_add(void){
	switch(disp_set_unit){
		
		case SET_FREQ_HZ:
			if(set_FREQ_num + set_pos <= 9999)
				set_FREQ_num += set_pos;
			else
				set_FREQ_num = 9999;
			break;
			
		case SET_FREQ_KHZ0:
		case SET_FREQ_KHZ1:
		case SET_FREQ_KHZ2:
		case SET_FREQ_MHZ:
			if(set_FREQ_num + set_pos <= 999)
				set_FREQ_num += set_pos;
			else
				set_FREQ_num = 999;
			break;
			
		case SET_PULSE_NS:
			if(set_pos && set_pos <= 10){
				if(set_PULSE_num + 50 <= 9950)
					set_PULSE_num += 50;
				else
					set_PULSE_num = 9950;
			}
			else{
				if(set_PULSE_num + set_pos <= 9950)
					set_PULSE_num += set_pos;
				else
					set_PULSE_num = 9950;
			}
			break;
			
		case SET_PULSE_US:
			if(set_PULSE_num + set_pos <= 5000)
				set_PULSE_num += set_pos;
			else
				set_PULSE_num = 5000;
			break;
			
		case SET_VOLT_MV:
			if(set_VOLT_num + set_pos <= 1000)
				set_VOLT_num += set_pos;
			else set_VOLT_num = 1000;
			break;
			
		default:
			;
	}
	if(set_pos){
        if(disp_set_unit == SET_VOLT_MV){
            dac_set_volt(set_VOLT_num);
        }
        else if(output_sta == ENABLE){
            set_output(ENABLE);
        }
    }
}

void key_do_sub(void){
	switch(disp_set_unit){
		
		case SET_FREQ_HZ:
		case SET_FREQ_KHZ0:
		case SET_FREQ_KHZ1:
		case SET_FREQ_KHZ2:
		case SET_FREQ_MHZ:
			if(set_FREQ_num - 1 > set_pos)
				set_FREQ_num -= set_pos;
			else
				set_FREQ_num = 1;

			break;
			
		case SET_PULSE_NS:
			if(set_pos && set_pos <= 10){
				if(set_PULSE_num - 50 > 50)
					set_PULSE_num -= 50;
				else
					set_PULSE_num = 50;
			}
			else{
				if(set_PULSE_num - 50 > set_pos)
					set_PULSE_num -= set_pos;
				else
					set_PULSE_num = 50;
			}
			
			break;
		case SET_PULSE_US:
			if(set_PULSE_num - 1 > set_pos)
				set_PULSE_num -= set_pos;
			else
				set_PULSE_num = 1;
			break;
			
		case SET_VOLT_MV:
			if(set_VOLT_num - 5 > set_pos)
				set_VOLT_num -= set_pos;
			else 
				set_VOLT_num = 5;
			break;
		default:
			;
	}
	if(set_pos){
        if(disp_set_unit == SET_VOLT_MV){
            dac_set_volt(set_VOLT_num);
        }
        else if(output_sta == ENABLE){
            set_output(ENABLE);
        }
    }
}

void key_do_negt(void){
}

volatile uint8_t output_mode = MODE_NORMAL;

/**
  * @brief  配置PWM脉冲宽度，无需修改PSC和ARR
  * @param  pulseWidth_ns: 期望脉冲宽度（纳秒）
  * @retval HAL
  */
HAL_StatusTypeDef TIM_ConfigPulseWidth(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t pulseWidth_ns)
{
    // 获取当前定时器配置
    uint32_t psc = htim->Instance->PSC;
    uint32_t arr = htim->Instance->ARR; 
    uint32_t timerClockFreq = HAL_RCC_GetHCLKFreq(); // 注意使用APB1定时器时钟频率时需根据实际情况调整
    
    // 计算定时器计数周期时间（纳秒）
    float timerPeriod_ns = (1.0f / (timerClockFreq / (psc + 1))) * 1e9f;
    
    // 计算需要的CCR值
    uint32_t ccr = (uint32_t)(pulseWidth_ns / timerPeriod_ns);
    
    // 确保CCR1小于ARR/2，保证占空比不超过50%
    if (ccr > (arr / 2)) {
        ccr = arr / 2;
    }
    
    // 确保CCR值有效
    if (ccr == 0) {
        ccr = 1;  // 防止脉宽太短或太长
    }
    
    // 设置CCR寄存器
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
  * @brief  配置定时器输出指定频率，支持0.1Hz级精度优化
  * @param  htim: 定时器句柄指针
  * @param  desiredFreq: 期望生成的频率(Hz) (支持0.1Hz级精度)
  * @retval HAL״̬
  */
HAL_StatusTypeDef TIM_ConfigFrequencyOptimized(
                            float desiredFreq, 
                            uint32_t pulseWidth_ns)
{
    // 检查输入参数有效性
    if (desiredFreq <= 0.0f)
    {
        return HAL_ERROR;
    }

    uint32_t bestPsc = 0;
    uint32_t bestArr = 0;
    float minError = FLT_MAX;  // 初始化为最大浮点数
    float actualFreq = 0.0f;
    uint32_t timerClockFreq = HAL_RCC_GetHCLKFreq();
    
    // 计算期望的周期计数
    float desiredPeriods = (float)timerClockFreq / desiredFreq;
    
    // 遍历寻找最优PSC和ARR组合
    for (uint32_t psc = 0; psc <= 66000; psc+=6)
    {
        uint32_t arr = (uint32_t)(desiredPeriods / (psc + 1)) - 1;
        
        // 确保ARR在有效范围内
        if (arr > 0xFFFF)
            continue;
        
        // 计算实际频率和误差
        actualFreq = (float)timerClockFreq / ((psc + 1) * (arr + 1));
        float error = fabsf(actualFreq - desiredFreq);
        
        // 如果找到更精确的组合，更新最优值
        if (error < minError)
        {
            minError = error;
            bestPsc = psc;
            bestArr = arr;
            
            // 如果误差已经足够小，提前退出循环
            if (error < (desiredFreq * 0.0001f)) // 0.01%精度
                break;
        }
    }
    
    // 检查是否找到有效解
    if (bestArr == 0 && bestPsc == 0 && desiredFreq < ((float)timerClockFreq / (0xFFFFFFFF)))
    {
        // 处理低频情况
        bestPsc = 0xFFFF;
        bestArr = (uint32_t)((float)timerClockFreq / (desiredFreq * (bestPsc + 1))) - 1;
        
        if (bestArr > 0xFFFF)
            return HAL_ERROR;
    }
    
    
    // 配置定时器
    HAL_TIM_Base_Stop_IT(&htimTIM);
    HAL_TIM_PWM_PWMN_Stop(&htimPWM, TIM_CHANNEL_1);
    HAL_TIM_PWM_PWMN_Stop_IT(&htimPWM, TIM_CHANNEL_1);
	
	HAL_TIM_Base_Init(&htimTIM);
    
	TIM1_ReversePolarity(output_negative);
	
    HAL_Delay(1);
	
		
		
		if(output_mode == MODE_SINGEL_BURST)
		{
			if(bestPsc == 0)
			{
				if (HAL_TIM_OnePulse_Init(&htimPWM, TIM_OPMODE_SINGLE) != HAL_OK)
				{
						Error_Handler();
				}
                
                htimPWM.Init.RepetitionCounter = 100-1;
                htimPWM.Init.Prescaler = 0;
                htimPWM.Init.Period = bestArr;
                
				TIM_ConfigPulseWidth(&htimPWM, TIM_CHANNEL_1, pulseWidth_ns);
                
                HAL_TIM_Base_Init(&htimPWM);
//				htimPWM.Instance->RCR = 100-1;
//				__HAL_TIM_SET_PRESCALER(&htimPWM, 0);
//				__HAL_TIM_SET_AUTORELOAD(&htimPWM, bestArr);
				
				HAL_TIM_PWM_PWMN_Start(&htim1, TIM_CHANNEL_1);
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
				if (HAL_TIM_OnePulse_Init(&htimPWM, TIM_OPMODE_SINGLE) != HAL_OK)
				{
						Error_Handler();
				}
                
                htimPWM.Init.RepetitionCounter = 100-1;
                htimPWM.Init.Prescaler = 0;
                htimPWM.Init.Period = bestArr;
                
				TIM_ConfigPulseWidth(&htimPWM, TIM_CHANNEL_1, pulseWidth_ns);
                
                HAL_TIM_Base_Init(&htimPWM);
//				htimPWM.Instance->RCR = 100-1;
//				__HAL_TIM_SET_PRESCALER(&htimPWM, 0);
//				__HAL_TIM_SET_AUTORELOAD(&htimPWM, bestArr);
				
                
				
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
            htimPWM.Init.RepetitionCounter = 0;
			if(bestPsc == 0)
			{
                    htimPWM.Init.Prescaler = 0;
                    htimPWM.Init.Period = bestArr;
                    
                    TIM_ConfigPulseWidth(&htimPWM, TIM_CHANNEL_1, pulseWidth_ns);
                    
                    HAL_TIM_Base_Init(&htimPWM);
					
					HAL_TIM_PWM_PWMN_Start(&htimPWM, TIM_CHANNEL_1);
			}
			else
			{
					__HAL_TIM_SET_PRESCALER(&htimTIM, bestPsc);
					__HAL_TIM_SET_AUTORELOAD(&htimTIM, bestArr);
				
					__HAL_TIM_SET_PRESCALER(&htimPWM, 0);
					__HAL_TIM_SET_AUTORELOAD(&htimPWM, 60000-1);
					
					TIM_ConfigPulseWidth(&htimPWM, TIM_CHANNEL_1, pulseWidth_ns);
					
					__HAL_TIM_SET_AUTORELOAD(&htimPWM, htimPWM.Instance->CCR1+10000);
                
                    htimPWM.Init.Prescaler = 0;
                    
                    HAL_TIM_Base_Init(&htimPWM);
					
					HAL_TIM_Base_Start_IT(&htimTIM);
			}
		}
		

    
    return HAL_OK;
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
