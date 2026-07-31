///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 本程序作者为西北工业大学郭、陈、曹，参与2025年全国大学生电子设计竞赛，并获得国家级一等奖
// Copyright (c) 2025 郭、陈、曹.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////////////////////////////////
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "multi_button.h"
#include "SEGGER_RTT.h"
#include "sin_wave.h"
#include "sin_wave_float.h"
#include "arm_math.h"
#include "oled.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float error;
    float prev_error;
    float prev_prev_error;
    float setpoint;
} PIDController;

static Button btn0, btn1, btn2, btn3, btn4, btn5, btn6, btn7, btn8, btn9, btn10, btn11, btn12, btn13, btn14, btn15;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define printf(...) SEGGER_RTT_printf(0, __VA_ARGS__)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Modulation Parameters */
#define THREE_PHASE_INVERTER_FREQ_HZ 50
#define FREQUENCY_CHANGE_1HZ 209370
#define FREQUENCY_COMPENSATE
uint32_t cnt_add = (4294967296 / 170000000 * 4096 * 2 * THREE_PHASE_INVERTER_FREQ_HZ + 228400);
#define FREQUENCY_50HZ (4294967296 / 170000000 * 4096 * 2 * THREE_PHASE_INVERTER_FREQ_HZ + 228400)
#ifdef CONFIG_INVERTER_USE_SVPWM
#define CARRIER_FREQ_HZ sin_wave_svpwm
#elif CONFIG_INVERTER_USE_SPWM
#define CARRIER_FREQ_HZ sin_wave_spwm
#elif CONFIG_INVERTER_USE_SVPWM_FLOAT
#define CARRIER_FREQ_HZ pwm_wave_svpwm_float
#elif CONFIG_INVERTER_USE_SPWM_FLOAT
#define CARRIER_FREQ_HZ pwm_wave_spwm_float
#endif
uint32_t inv_phi_u = 0; // u相位
uint32_t inv_phi_v = 1431655765; // v相位
uint32_t inv_phi_w = 2863311530; // w相位
uint32_t inv_pwm_index_u = 0; // u相位索引
uint32_t inv_pwm_index_v = 0; // v相位索引
uint32_t inv_pwm_index_w = 0; // w相位索引
uint32_t con_phi_u = 0; // u相位
uint32_t con_phi_v = 1431655765; // v相位
uint32_t con_phi_w = 2863311530; // w相位
uint32_t con_pwm_index_u = 0; // u相位索引
uint32_t con_pwm_index_v = 0; // v相位索引
uint32_t con_pwm_index_w = 0; // w相位索引
uint32_t delta_phi = 2147483648;
float inv_depth = 0.99f; // 调制深度 (0.f~1.f)
float con_depth = 0.99f; // 调制深度 (0.f~1.f)
uint32_t dc_i_compare = 3480;
/* ADC Sampling Parameters */
#define ADC_BUFFER_SIZE 100
uint16_t adc_buffer[ADC_BUFFER_SIZE];
volatile uint8_t adc_data_ready_flag = 0;
#define MEDIAN_FILTER_SIZE 5
uint16_t filtered_adc_buffer[ADC_BUFFER_SIZE];
float filtered_vpp_buffer[ADC_BUFFER_SIZE];
/* PID Controller Parameters */
float output_V = 0;
float Vpp_set = 45.255f - 0.5f;
float Vpp_now = 0.f;
volatile float g_latest_vpp = 0.f;
/* FREQ&PHASE Capture Parameters */
volatile uint32_t g_period_ticks = 0;
volatile int32_t g_time_diff_ticks = 0;
volatile float g_frequency = 0.0f;
volatile float g_phase_diff_degree = 0.0f;

#define AVG_COUNT 100
float vpp_buffer[AVG_COUNT];
int vpp_index = 0;
bool vpp_ready = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// PID
void initPIDController(PIDController* pid, float Kp, float Ki, float Kd);
void incrementalPDControl_V(PIDController* pid, float Vnow, float Vset);
void PID_Ctrl();
// Button
uint8_t read_button_gpio(uint8_t button_id);
void btn0_single_click_handler(Button *btn);
void btn1_single_click_handler(Button *btn);
void btn2_single_click_handler(Button *btn);
void btn3_single_click_handler(Button *btn);
void btn4_single_click_handler(Button *btn);
void btn5_single_click_handler(Button *btn);
void btn6_single_click_handler(Button *btn);
void btn7_single_click_handler(Button *btn);
void btn8_single_click_handler(Button *btn);
void btn9_single_click_handler(Button *btn);
void btn10_single_click_handler(Button *btn);
void btn11_single_click_handler(Button *btn);
void btn12_single_click_handler(Button *btn);
void btn13_single_click_handler(Button *btn);
void btn14_single_click_handler(Button *btn);
void btn15_single_click_handler(Button *btn);
//ADC
float get_vpp_voltage(void);
static int compare_uint16(const void* a, const void* b);
uint16_t median_filter(uint16_t new_sample);
float calc_vpp(uint16_t* buffer, uint32_t size);
// Start
void Inverter_Start(void);
void Converter_Start(void);
void ADC_TIM_Start(void);
void Button_Start();
void PID_Control_Start();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Start Functions */
void Inverter_Start(void)
{
    HAL_TIM_Base_Start_IT(&htim1);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void Converter_Start(void)
{
    HAL_TIM_Base_Start_IT(&htim8);

    // HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    // HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    // HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
}

void ADC_TIM_Start(void)
{
    HAL_TIM_Base_Start(&htim2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);
}

void Button_Start()
{
    // Button 0
    button_init(&btn0, read_button_gpio, 0, 0);
    button_attach(&btn0, BTN_SINGLE_CLICK, btn0_single_click_handler);
    button_start(&btn0);

    // Button 1
    button_init(&btn1, read_button_gpio, 0, 1);
    button_attach(&btn1, BTN_SINGLE_CLICK, btn1_single_click_handler);
    button_start(&btn1);

    // Button 2
    button_init(&btn2, read_button_gpio, 0, 2);
    button_attach(&btn2, BTN_SINGLE_CLICK, btn2_single_click_handler);
    button_start(&btn2);

    // Button 3
    button_init(&btn3, read_button_gpio, 0, 3);
    button_attach(&btn3, BTN_SINGLE_CLICK, btn3_single_click_handler);
    button_start(&btn3);

    // Button 4
    button_init(&btn4, read_button_gpio, 0, 4);
    button_attach(&btn4, BTN_SINGLE_CLICK, btn4_single_click_handler);
    button_start(&btn4);

    // Button 5
    button_init(&btn5, read_button_gpio, 0, 5);
    button_attach(&btn5, BTN_SINGLE_CLICK, btn5_single_click_handler);
    button_start(&btn5);

    // Button 6
    button_init(&btn6, read_button_gpio, 0, 6);
    button_attach(&btn6, BTN_SINGLE_CLICK, btn6_single_click_handler);
    button_start(&btn6);

    // Button 7
    button_init(&btn7, read_button_gpio, 0, 7);
    button_attach(&btn7, BTN_SINGLE_CLICK, btn7_single_click_handler);
    button_start(&btn7);

    // Button 8
    button_init(&btn8, read_button_gpio, 0, 8);
    button_attach(&btn8, BTN_SINGLE_CLICK, btn8_single_click_handler);
    button_start(&btn8);

    // Button 9
    button_init(&btn9, read_button_gpio, 0, 9);
    button_attach(&btn9, BTN_SINGLE_CLICK, btn9_single_click_handler);
    button_start(&btn9);

    // Button 10
    button_init(&btn10, read_button_gpio, 0, 10);
    button_attach(&btn10, BTN_SINGLE_CLICK, btn10_single_click_handler);
    button_start(&btn10);

    // Button 11
    button_init(&btn11, read_button_gpio, 0, 11);
    button_attach(&btn11, BTN_SINGLE_CLICK, btn11_single_click_handler);
    button_start(&btn11);

    // Button 12
    button_init(&btn12, read_button_gpio, 0, 12);
    button_attach(&btn12, BTN_SINGLE_CLICK, btn12_single_click_handler);
    button_start(&btn12);

    // Button 13
    button_init(&btn13, read_button_gpio, 0, 13);
    button_attach(&btn13, BTN_SINGLE_CLICK, btn13_single_click_handler);
    button_start(&btn13);

    // Button 14
    button_init(&btn14, read_button_gpio, 0, 14);
    button_attach(&btn14, BTN_SINGLE_CLICK, btn14_single_click_handler);
    button_start(&btn14);

    // Button 15
    button_init(&btn15, read_button_gpio, 0, 15);
    button_attach(&btn15, BTN_SINGLE_CLICK, btn15_single_click_handler);
    button_start(&btn15);
}

void PID_Control_Start()
{
    HAL_TIM_Base_Start_IT(&htim14);
}

/* Public Functions */
// PID
void initPIDController(PIDController* pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->error = 0;
    pid->prev_error = 0;
    pid->prev_prev_error = 0;
}

void incrementalPDControl_V(PIDController* pid, float Vnow, float Vset)
{
    pid->error = Vset - Vnow;

    float add = pid->Kp * (pid->error - pid->prev_error)
        + pid->Ki * pid->error
        + pid->Kd * (pid->error - 2 * pid->prev_error + pid->prev_prev_error);

    output_V += add;

    if (output_V < 0.0f)
        output_V = 0.0f;
    if (output_V > 0.99f)
        output_V = 0.99f;

    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = pid->error;
}

void PID_Ctrl()
{
    static PIDController pid_V;
    static int initialized = 0;
    if (!initialized)
    {
        initPIDController(&pid_V, 0.002f, 0.0001f, 0.f);
        initialized = 1;
    }
    Vpp_now = get_vpp_voltage();
    incrementalPDControl_V(&pid_V, Vpp_now, Vpp_set);
    inv_depth = output_V;
}

// Button
uint8_t read_button_gpio(uint8_t button_id)
{
    switch (button_id)
    {
    case 0: return HAL_GPIO_ReadPin(K0_GPIO_Port, K0_Pin);
    case 1: return HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin);
    case 2: return HAL_GPIO_ReadPin(K2_GPIO_Port, K2_Pin);
    case 3: return HAL_GPIO_ReadPin(K3_GPIO_Port, K3_Pin);
    case 4: return HAL_GPIO_ReadPin(K4_GPIO_Port, K4_Pin);
    case 5: return HAL_GPIO_ReadPin(K5_GPIO_Port, K5_Pin);
    case 6: return HAL_GPIO_ReadPin(K6_GPIO_Port, K6_Pin);
    case 7: return HAL_GPIO_ReadPin(K7_GPIO_Port, K7_Pin);
    case 8: return HAL_GPIO_ReadPin(K8_GPIO_Port, K8_Pin);
    case 9: return HAL_GPIO_ReadPin(K9_GPIO_Port, K9_Pin);
    case 10: return HAL_GPIO_ReadPin(K10_GPIO_Port, K10_Pin);
    case 11: return HAL_GPIO_ReadPin(K11_GPIO_Port, K11_Pin);
    case 12: return HAL_GPIO_ReadPin(K12_GPIO_Port, K12_Pin);
    case 13: return HAL_GPIO_ReadPin(K13_GPIO_Port, K13_Pin);
    case 14: return HAL_GPIO_ReadPin(K14_GPIO_Port, K14_Pin);
    case 15: return HAL_GPIO_ReadPin(K15_GPIO_Port, K15_Pin);
    default: return 0;
    }
}

void btn0_single_click_handler(Button *btn)
{
    printf("btn0 single click\r\n");
    // for software reset
    SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) |
        (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
        SCB_AIRCR_SYSRESETREQ_Msk |
        SCB_AIRCR_VECTCLRACTIVE_Msk | SCB_AIRCR_VECTRESET_Msk);
}

void btn1_single_click_handler(Button *btn)
{
    printf("btn1 single click\r\n");
    HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_3);
}

void btn2_single_click_handler(Button *btn)
{
    printf("btn2 single click\r\n");
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
}

void btn3_single_click_handler(Button *btn)
{
    printf("btn3 single click\r\n");
    Vpp_set -= 1.f;
}

void btn4_single_click_handler(Button *btn)
{
    printf("btn4 single click\r\n");
    Vpp_set -= 0.1f;
}

void btn5_single_click_handler(Button *btn)
{
    printf("btn5 single click\r\n");
    Vpp_set = 45.255f;
}

void btn6_single_click_handler(Button *btn)
{
    printf("btn6 single click\r\n");
    Vpp_set += 0.1f;
}

void btn7_single_click_handler(Button *btn)
{
    printf("btn7 single click\r\n");
    Vpp_set += 1.f;
}

void btn8_single_click_handler(Button *btn)
{
    printf("btn8 single click\r\n");
    cnt_add += 10 * FREQUENCY_CHANGE_1HZ;
}

void btn9_single_click_handler(Button *btn)
{
    printf("btn9 single click\r\n");
    cnt_add += 1 * FREQUENCY_CHANGE_1HZ;
}

void btn10_single_click_handler(Button *btn)
{
    printf("btn10 single click\r\n");
    cnt_add = FREQUENCY_50HZ;
}

void btn11_single_click_handler(Button *btn)
{
    printf("btn11 single click\r\n");
    cnt_add -= 1 * FREQUENCY_CHANGE_1HZ;
}

void btn12_single_click_handler(Button *btn)
{
    printf("btn12 single click\r\n");
    cnt_add -= 10 * FREQUENCY_CHANGE_1HZ;
}

void btn13_single_click_handler(Button *btn)
{
    printf("btn13 single click\r\n");
    dc_i_compare -= 4;
}

void btn14_single_click_handler(Button *btn)
{
    printf("btn14 single click\r\n");
    dc_i_compare = 3600;
}

void btn15_single_click_handler(Button *btn)
{
    printf("btn15 single click\r\n");
    dc_i_compare += 4;
}

// ADC
float get_vpp_voltage(void)
{
    return g_latest_vpp;
}

static int compare_uint16(const void* a, const void* b)
{
    uint16_t val1 = *(const uint16_t*)a;
    uint16_t val2 = *(const uint16_t*)b;
    if (val1 < val2) return -1;
    if (val1 > val2) return 1;
    return 0;
}

uint16_t median_filter(uint16_t new_sample)
{
    static uint16_t window[MEDIAN_FILTER_SIZE];
    static uint8_t index = 0;
    static bool window_full = false;

    window[index] = new_sample;
    index++;

    if (index >= MEDIAN_FILTER_SIZE)
    {
        index = 0;
        window_full = true;
    }

    if (!window_full)
    {
        return new_sample;
    }

    uint16_t temp_sorted_window[MEDIAN_FILTER_SIZE];
    for (int i = 0; i < MEDIAN_FILTER_SIZE; i++)
    {
        temp_sorted_window[i] = window[i];
    }

    qsort(temp_sorted_window, MEDIAN_FILTER_SIZE, sizeof(uint16_t), compare_uint16);

    return temp_sorted_window[MEDIAN_FILTER_SIZE / 2];
}

float calc_vpp(uint16_t* buffer, uint32_t size)
{
    if (buffer == NULL || size == 0)
    {
        return 0.0f;
    }

    uint16_t adc_max = buffer[0];
    uint16_t adc_min = buffer[0];

    for (uint32_t i = 1; i < size; i++)
    {
        if (buffer[i] > adc_max)
        {
            adc_max = buffer[i];
        }
        if (buffer[i] < adc_min)
        {
            adc_min = buffer[i];
        }
    }

    uint16_t adc_pp = adc_max - adc_min;

    float vpp = 15.3570f * ((float)adc_pp / 4096.0f * 3.3f) + 2.1906f;
    float true_vpp = 1.0615 * vpp - 2.3372 + 0.28f;

    return true_vpp;
}

/* Interrupt Callback Functions */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1 && __HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
    {
        inv_phi_u += cnt_add;
        inv_phi_v += cnt_add;
        inv_phi_w += cnt_add;
        inv_pwm_index_u = inv_phi_u >> 20;
        inv_pwm_index_v = inv_phi_v >> 20;
        inv_pwm_index_w = inv_phi_w >> 20;
        __HAL_TIM_SetCompare(htim, TIM_CHANNEL_1, CARRIER_FREQ_HZ[inv_pwm_index_u] * inv_depth + 2048);
        __HAL_TIM_SetCompare(htim, TIM_CHANNEL_2, CARRIER_FREQ_HZ[inv_pwm_index_v] * inv_depth + 2048);
        __HAL_TIM_SetCompare(htim, TIM_CHANNEL_3, CARRIER_FREQ_HZ[inv_pwm_index_w] * inv_depth + 2048);
    }

    if (htim->Instance == TIM8 && __HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
    {
        con_phi_u = inv_phi_u + delta_phi;
        con_phi_v = inv_phi_v + delta_phi;
        con_phi_w = inv_phi_w + delta_phi;
        con_pwm_index_u = con_phi_u >> 20;
        con_pwm_index_v = con_phi_v >> 20;
        con_pwm_index_w = con_phi_w >> 20;
        // __HAL_TIM_SetCompare(htim, TIM_CHANNEL_1, dc_i_compare);
        // __HAL_TIM_SetCompare(htim, TIM_CHANNEL_2, dc_i_compare);
        __HAL_TIM_SetCompare(htim, TIM_CHANNEL_3, dc_i_compare);
    }

    if (htim->Instance == TIM14)
    {
        PID_Ctrl();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    adc_data_ready_flag = 1;
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
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM14_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
    SEGGER_RTT_Init();
    printf("Hello Three Phase Inverter\r\n");
    printf("System Clock: %ld Hz\r\n", HAL_RCC_GetSysClockFreq());
    // HAL_Delay(1000);
    // OLED_Init();
    Button_Start();
    Inverter_Start();
    Converter_Start();
    ADC_TIM_Start();
    PID_Control_Start();
    char str[32];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (adc_data_ready_flag)
        {
            adc_data_ready_flag = 0;

            for (int i = 0; i < ADC_BUFFER_SIZE; i++)
            {
                filtered_adc_buffer[i] = median_filter(adc_buffer[i]);
                filtered_vpp_buffer[i] = calc_vpp(filtered_adc_buffer, ADC_BUFFER_SIZE);
            }
            float current_vpp = calc_vpp(filtered_adc_buffer, ADC_BUFFER_SIZE);
            g_latest_vpp = current_vpp;
            // sprintf(str, "%.6f\r\n", current_vpp);
            // printf(str);
        }
        // sprintf(str, "now vpp: %.2f\r\n", get_vpp_voltage());
        // printf(str);
        // char str[64];


        // 在每次调用处更新
        vpp_buffer[vpp_index++] = Vpp_now;
        if (vpp_index >= AVG_COUNT) {
            vpp_index = 0;
            vpp_ready = true;
        }

        if (vpp_ready) {
            float sum = 0.0f;
            for (int i = 0; i < AVG_COUNT; i++) {
                sum += vpp_buffer[i];
            }
            float vpp_avg = sum / AVG_COUNT;


            // OLED_NewFrame();
            // sprintf(str, "Vpnow: %.2f\r\n", vpp_avg);
            // OLED_PrintString(2, 0, str, &font16x16, OLED_COLOR_NORMAL);
            // sprintf(str, "Vpset: %.2f", Vpp_set);
            // OLED_PrintString(2, 16, str, &font16x16, OLED_COLOR_NORMAL);
            // sprintf(str, "M: %.2f", output_V);
            // OLED_PrintString(2, 32, str, &font16x16, OLED_COLOR_NORMAL);
            // sprintf(str, "i: %d", dc_i_compare);
            // OLED_PrintString(2, 48, str, &font16x16, OLED_COLOR_NORMAL);
            // OLED_ShowFrame();

            vpp_ready = false;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
