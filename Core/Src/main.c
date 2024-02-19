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
#define ADC_PORT_0 GPIOA
#define ADC_PORT_1 GPIOA
#define ADC_PORT_2 GPIOA
#define ADC_PORT_3 GPIOA
#define ADC_PORT_4 GPIOA
#define ADC_PIN_0 GPIO_PIN_0
#define ADC_PIN_1 GPIO_PIN_1
#define ADC_PIN_2 GPIO_PIN_2
#define ADC_PIN_3 GPIO_PIN_3
#define ADC_PIN_4 GPIO_PIN_4

#define OUT_PORT_0 GPIOA
#define OUT_PORT_1 GPIOB
#define OUT_PORT_2 GPIOA
#define OUT_PORT_3 GPIOA
#define OUT_PORT_4 GPIOA
#define OUT_PIN_0 GPIO_PIN_9
#define OUT_PIN_1 GPIO_PIN_1
#define OUT_PIN_2 GPIO_PIN_7
#define OUT_PIN_3 GPIO_PIN_6
#define OUT_PIN_4 GPIO_PIN_5

#define OUT_LINE_PORT GPIOA
#define OUT_LINE_PIN GPIO_PIN_12

#define LED_PORT GPIOF
#define LED_PIN GPIO_PIN_0

#define KEY_PORT GPIOF
#define KEY_PIN GPIO_PIN_1

uint16_t adc_value[5] = {0};
uint8_t TIM17_flag = 0;
uint32_t out_time_dely = 0;


int adc_list[5][50] = {0};
uint8_t adc_list_the_po = 0;

int po_least_at = 4; // 检查到的变化的最少次数
int po_most_at = 18; // 检查到的变化的最多次数
int wave_range = 50; // 波动范围

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
ADC_HandleTypeDef hadc;

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
static void MX_ADC_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

// �????????????启adc事件
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_value[0] = HAL_ADC_GetValue(hadc);
    adc_value[1] = HAL_ADC_GetValue(hadc);
    adc_value[2] = HAL_ADC_GetValue(hadc);
    adc_value[3] = HAL_ADC_GetValue(hadc);
    adc_value[4] = HAL_ADC_GetValue(hadc);
  }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int adc_average[5] = {0};
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
  MX_ADC_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  MX_TIM14_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // 打开tim1 pwm输出
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_GPIO_WritePin(OUT_PORT_4, OUT_PIN_4, GPIO_PIN_SET);

  // 打开tim17中断
  HAL_TIM_Base_Start_IT(&htim17);

  HAL_ADC_Start(&hadc);

  uint8_t i = 0;
  uint32_t summation[5] = {0};
  uint8_t pw[5] = {0};
  uint8_t scanstate = 0; // 0 校准状态 1 等待变化状态
  uint8_t steady[5] = {0};
  uint8_t steady_add = 0;
  uint16_t the_aValue[5] = {0};
  int bx_U[5]={0};
  int bx_D[5]={0};
  int bx_P[5]={0};
  int bx_U_up[5]={0};



  uint8_t out_flag=0;
  uint8_t out_state =0;
  uint16_t out_time =0;
  uint16_t on_time =200;




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (TIM17_flag == 1)
    {

      if(on_time>0){  // 刚开机 或 刚有物体掉落 都不进行输出
        on_time--;
        out_time=0;
      }

      if (out_time>0){
        out_time--;
        HAL_GPIO_WritePin(LED_PORT,LED_PIN,GPIO_PIN_SET);
        HAL_GPIO_WritePin(OUT_LINE_PORT,OUT_LINE_PIN,GPIO_PIN_SET);
        if (out_time==0){
          HAL_GPIO_WritePin(LED_PORT,LED_PIN,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(OUT_LINE_PORT,OUT_LINE_PIN,GPIO_PIN_RESET);
        }

      }


      TIM17_flag = 0;

      adc_list_the_po++;
      if (adc_list_the_po >= 50)
      {
        adc_list_the_po = 0;
      }

      // 打开红外发射io3 为高电平
      // HAL_GPIO_WritePin(OUT_PORT_3, OUT_PIN_3, GPIO_PIN_SET);

      // 读取adc�????????????
      HAL_ADC_Start(&hadc);
      HAL_ADC_PollForConversion(&hadc, 100);
      adc_list[0][adc_list_the_po] = HAL_ADC_GetValue(&hadc);
      HAL_ADC_PollForConversion(&hadc, 100);
      adc_list[1][adc_list_the_po] = HAL_ADC_GetValue(&hadc);
      HAL_ADC_PollForConversion(&hadc, 100);
      adc_list[2][adc_list_the_po] = HAL_ADC_GetValue(&hadc);
      HAL_ADC_PollForConversion(&hadc, 100);
      adc_list[3][adc_list_the_po] = HAL_ADC_GetValue(&hadc);
      HAL_ADC_PollForConversion(&hadc, 100);
      adc_list[4][adc_list_the_po] = HAL_ADC_GetValue(&hadc);

      // 建立一个10次的循环,将adc_list中的数据进行累加,是从adc_list_the_po开始，向后累加
      uint8_t staddpo = 0;
      int forpo = 0;
      staddpo = adc_list_the_po - 10;
      if (adc_list_the_po < 10)
      {
        staddpo = 50 - abs(adc_list_the_po - 10);
      }

      forpo = staddpo;
      summation[0] = 0;
      summation[1] = 0;
      summation[2] = 0;
      summation[3] = 0;
      summation[4] = 0;
      for (i = 0; i < 10; i++)
      {
        forpo++;
        if (forpo >= 50)
        {
          forpo = 0;
        }
        summation[0] = summation[0] + adc_list[0][forpo]; // 累加每一位
        summation[1] = summation[1] + adc_list[1][forpo]; // 累加每一位
        summation[2] = summation[2] + adc_list[2][forpo]; // 累加每一位
        summation[3] = summation[3] + adc_list[3][forpo]; // 累加每一位
        summation[4] = summation[4] + adc_list[4][forpo]; // 累加每一位

      }

      adc_average[0] = summation[0] / 10; // 求出平均值
      adc_average[1] = summation[1] / 10; // 求出平均值
      adc_average[2] = summation[2] / 10; // 求出平均值
      adc_average[3] = summation[3] / 10; // 求出平均值
      adc_average[4] = summation[4] / 10; // 求出平均值

      forpo = staddpo;
      steady[0] = 0;
      steady[1] = 0;
      steady[2] = 0;
      steady[3] = 0;
      steady[4] = 0;
      for (i = 0; i < 10; i++) // 判读就近10个数据是否稳定
      {
        forpo++;
        if (forpo >= 50)
        {
          forpo = 0;
        }

        if (adc_list[0][forpo] > adc_average[0] + wave_range || adc_list[0][forpo] < adc_average[0] - wave_range)
        {
          steady[0]++;
        }
        if (adc_list[1][forpo] > adc_average[1] + wave_range || adc_list[1][forpo] < adc_average[1] - wave_range)
        {
          steady[1]++;
        }
        if (adc_list[2][forpo] > adc_average[2] + wave_range || adc_list[2][forpo] < adc_average[2] - wave_range)
        {
          steady[2]++;
        }
        if (adc_list[3][forpo] > adc_average[3] + wave_range || adc_list[3][forpo] < adc_average[3] - wave_range)
        {
          steady[3]++;
        }
        if (adc_list[4][forpo] > adc_average[4] + wave_range || adc_list[4][forpo] < adc_average[4] - wave_range)
        {
          steady[4]++;
        }


      }

      steady_add = steady[0] + steady[1] + steady[2] + steady[3] + steady[4];

      if (steady_add == 0) // 所有的都有10个稳定数据
      { // 稳定就继续倒数30次 看是否稳定

        forpo = staddpo - 30;
        if (forpo < 0)
        {
          forpo = 50 - abs(forpo);
        }

        steady[0] = 0;

        bx_D[0]=0;bx_D[1]=0;bx_D[2]=0;bx_D[3]=0;bx_D[4]=0;
        bx_U[0]=0;bx_U[1]=0;bx_U[2]=0;bx_U[3]=0;bx_U[4]=0;
        bx_P[0]=0;bx_P[1]=0;bx_P[2]=0;bx_P[3]=0;bx_P[4]=0;

        

        for (i = 0; i < 40; i++)
        { // 倒数30次 累计不稳定的次数
          forpo++;
          if (forpo >= 50)
          {
            forpo = 0;
          }
                   if (adc_list[0][forpo] > adc_average[0] + wave_range )
          {
            steady[0]++;
            bx_U[0]++;
            bx_D[0]--;
          }else if (adc_list[0][forpo] < adc_average[0] - wave_range) {
            steady[0]++;
            bx_D[0]++;
            bx_U[0]--;
          }else{
            if (bx_U_up[0]==i-1 ){
              bx_P[0]++;
            }else{
              bx_P[0]=0;
            }

            bx_U_up[0]=i;
          }

          if (adc_list[1][forpo] > adc_average[1] + wave_range )
          {
            steady[1]++;
            bx_U[1]++;
            bx_D[1]--;
          }else if (adc_list[1][forpo] < adc_average[1] - wave_range) {
            steady[1]++;
            bx_D[1]++;
            bx_U[1]--;
          }else{
            if (bx_U_up[1]==i-1 ){
              bx_P[1]++;
            } else{
              bx_P[1]=0;
            }

            bx_U_up[1]=i;
          }

          if (adc_list[2][forpo] > adc_average[2] + wave_range )
          {
            steady[2]++;
            bx_U[2]++;
            bx_D[2]--;
          }else if (adc_list[2][forpo] < adc_average[2] - wave_range) {
            steady[2]++;
            bx_D[2]++;
            bx_U[2]--;
          }else{
            if (bx_U_up[2]==i-1 ){
              bx_P[2]++;
            } else{
              bx_P[2]=0;
            }

            bx_U_up[2]=i;
          }

          if (adc_list[3][forpo] > adc_average[3] + wave_range )
          {
            steady[3]++;
            bx_U[3]++;
            bx_D[3]--;
          }else if (adc_list[3][forpo] < adc_average[3] - wave_range) {
            steady[3]++;
            bx_D[3]++;
            bx_U[3]--;
          }else{
            if (bx_U_up[3]==i-1 ){
              bx_P[3]++;
            } else{
              bx_P[3]=0;
            }

            bx_U_up[3]=i;
          }

          if (adc_list[4][forpo] > adc_average[4] + wave_range )
          {
            steady[4]++;
            bx_U[4]++;
            bx_D[4]--;
          }else if (adc_list[4][forpo] < adc_average[4] - wave_range) {
            steady[4]++;
            bx_D[4]++;
            bx_U[4]--;
          }else{
            if (bx_U_up[4]==i-1 ){
              bx_P[4]++;
            } else{
              bx_P[4]=0;
            }
            bx_U_up[4]=i;
          }





        }
        out_flag=0;
        steady[0] = 0;
        steady[1] = 0;
        steady[2] = 0;
        steady[3] = 0;
        steady[4] = 0;


        if (bx_U[0] > po_least_at)
        { // 就相当于有物体掉落
          if (bx_U[0] < po_most_at){
            out_flag=1;
            steady[0]=1;
          }
        }
        if (bx_D[0] > po_least_at){
          if (bx_D[0] < po_most_at){
            out_flag=1;
            steady[0]=1;
          }
        }
        if (bx_U[1] > po_least_at)
        { // 就相当于有物体掉落
          if (bx_U[1] < po_most_at){
            out_flag=1;
            steady[1]=1;
          }
        }
        if (bx_D[1] > po_least_at){
          if (bx_D[1] < po_most_at){
            out_flag=1;
            steady[1]=1;
          }
        }

        if (bx_U[2] > po_least_at)
        { // 就相当于有物体掉落
          if (bx_U[2] < po_most_at){
            out_flag=1;
            steady[2]=1;
          }
        }
        if (bx_D[2] > po_least_at){
          if (bx_D[2] < po_most_at){
            out_flag=1;
            steady[2]=1;
          }
        }

        if (bx_U[3] > po_least_at)
        { // 就相当于有物体掉落
          if (bx_U[3] < po_most_at){
            out_flag=1;
            steady[3]=1;
          }
        }
        if (bx_D[3] > po_least_at){
          if (bx_D[3] < po_most_at){
            out_flag=1;
            steady[3]=1;
          }
        }

        if (bx_U[4] > po_least_at)
        { // 就相当于有物体掉落
          if (bx_U[4] < po_most_at){
            out_flag=1;
            steady[4]=1;
          }
        }
        if (bx_D[4] > po_least_at){
          if (bx_D[4] < po_most_at){
            out_flag=1;
            steady[4]=1;
          }
        }


        if (out_flag==1){
           if (steady[0]==1){ // 再检查最早的数据是否稳定
             for (i=0;i<5;i++){
               if (adc_list[0][adc_list_the_po+i+1] < adc_average[0] + wave_range && adc_list[0][adc_list_the_po+i+1] > adc_average[0] - wave_range){
                 steady[0]++;
               }
             }
           }

            if (steady[1]==1){ // 再检查最早的数据是否稳定
              for (i=0;i<5;i++){
                if (adc_list[1][adc_list_the_po+i+1] < adc_average[1] + wave_range && adc_list[1][adc_list_the_po+i+1] > adc_average[1] - wave_range){
                  steady[0]++;
                }
              }
            }

            if (steady[2]==1){ // 再检查最早的数据是否稳定
              for (i=0;i<5;i++){
                if (adc_list[2][adc_list_the_po+i+1] < adc_average[2] + wave_range && adc_list[2][adc_list_the_po+i+1] > adc_average[2] - wave_range){
                  steady[0]++;
                }
              }
            }

            if (steady[3]==1){ // 再检查最早的数据是否稳定
              for (i=0;i<5;i++){
                if (adc_list[3][adc_list_the_po+i+1] < adc_average[3] + wave_range && adc_list[3][adc_list_the_po+i+1] > adc_average[3] - wave_range){
                  steady[0]++;
                }
              }
            }

            if (steady[4]==1){ // 再检查最早的数据是否稳定
              for (i=0;i<5;i++){
                if (adc_list[4][adc_list_the_po+i+1] < adc_average[4] + wave_range && adc_list[4][adc_list_the_po+i+1] > adc_average[4] - wave_range){
                  steady[0]++;
                }
              }
            }

            if (steady[0]>4 || steady[1]>4 || steady[2]>4 || steady[3]>4 || steady[4]>4){
              out_time = 20;

            }





            for (i = 0; i < 5; i++)
            {
              summation[i] = 0;
              steady[i] = 0;
            }

            for (i = 0; i < 50; i++)
            {
              adc_list[0][i] = adc_average[0];
              adc_list[1][i] = adc_average[1];
              adc_list[2][i] = adc_average[2];
              adc_list[3][i] = adc_average[3];
              adc_list[4][i] = adc_average[4];
            }



        }


      }

      // if (scanstate==0){
      //   summation[0] = summation[0] + adc_list[0][adc_list_the_po]; //累加每一位
      //   if (adc_list_the_po==19){  //减去最早的一位数据
      //     summation[0]= summation[0]- adc_list[0][0]; //在最后的位置，最早的数就是第一位
      //   }else{
      //     summation[0]= summation[0]- adc_list[0][adc_list_the_po+1];
      //   }

      //   adc_average[0] = summation[0]/19; // 求出平均值
      //   uint8_t steady[5] ={0};
      //   for (i=0;i<19;i++){
      //     if (adc_list[0][i] > adc_average[0]+50 || adc_list[0][i] < adc_average[0]-50 ){
      //       steady[0]++;
      //     }
      //   }
      //   if (steady[0]==0){ // 没问题就将状态至于1
      //     scanstate=1;
      //   }
      // }else{
      //   __NOP();
      // }
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
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

  /** Configure for the selected ADC regular channel to be converted.
   */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
   */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
   */
  sConfig.Channel = ADC_CHANNEL_4;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1262;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 900;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1262;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 900;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
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
  htim14.Init.Prescaler = 63;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 63;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
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
  htim17.Init.Prescaler = 47;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 9999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

#ifdef USE_FULL_ASSERT
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
