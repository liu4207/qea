/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI 3.141592653589793

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/*
 * 默认机器人的前方相对坐标系x轴，左方为相对坐标系y�?
 * 启动时，默认前方为绝对坐标系x轴，左方为绝对坐标系y�?
 */

struct WheelPWM
{
    double speed; //当前速度
    double target; //目标速度
    double minPWM; //限制pwm输出范围
    double maxPWM;
    double kp;
    double ki;
    double kd;
    double error;
    double lastError;
    double errorI;
    double pwm; //pwm输出�?
    double pulse; //脉冲�?
}wheelPWM1, wheelPWM2;

struct RobotInfo
{
    double wheel1Speed; //左轮速度
    double wheel2Speed; //右轮速度
    double XPos; //绝对坐标
    double YPos;
    double radian; //方向, 逆时�?, 起点x�?
}robotInfo;

void ClearSpeed()
{
    robotInfo.wheel1Speed = 0;
    robotInfo.wheel2Speed = 0;
}

void MoveForward(double speed)
{
    robotInfo.wheel1Speed += speed * 1.14;
    robotInfo.wheel2Speed += -speed;
}

void MoveBackward(double speed)
{
    robotInfo.wheel1Speed += -speed;
    robotInfo.wheel2Speed += speed;
}

void SpinLeft(double speed)
{
    robotInfo.wheel1Speed += -speed;
    robotInfo.wheel2Speed += -speed;
}

void SpinRight(double speed)
{
    robotInfo.wheel1Speed += speed;
    robotInfo.wheel2Speed += speed;
}

void CommitSpeed()
{
    wheelPWM1.target = robotInfo.wheel1Speed;
    wheelPWM2.target = robotInfo.wheel2Speed;
}

void Stop()
{
    ClearSpeed();
    CommitSpeed();
}

//将距离转换成脉冲 100cm -> 11500
double DisConvertToPulse(double dis)
{
    return 115 * dis;
}
//将弧度转换成脉冲 pi/2 -> 1700
double RadianConvertToPulse(double radian)
{
    return 1055.253613024887954 * radian;
}

//向前移动，�?�度，距离（cm�?
void MoveForwardV2(double speed, double dis)
{
    double pulse = DisConvertToPulse(dis);
    wheelPWM1.pulse = 0;
    ClearSpeed();
    MoveForward(speed);
    CommitSpeed();
    while (fabs(wheelPWM1.pulse) < pulse) {}
    Stop();
}

//向后移动，�?�度，距离（cm�?
void MoveBackwardV2(double speed, double dis)
{
    double pulse = DisConvertToPulse(dis);
    wheelPWM1.pulse = 0;
    ClearSpeed();
    MoveBackward(speed);
    CommitSpeed();
    while (fabs(wheelPWM1.pulse) < pulse) {}
    Stop();
}

//向左自转，�?�度，角�?(rad)
void SpinLeftV2(double speed, double radian)
{
    double pulse = RadianConvertToPulse(radian);
    wheelPWM1.pulse = 0;
    ClearSpeed();
    SpinLeft(speed);
    CommitSpeed();
    while (fabs(wheelPWM1.pulse) < pulse) {}
    Stop();
}

//向右自转，�?�度，角�?(rad)
void SpinRightV2(double speed, double radian)
{
    double pulse = RadianConvertToPulse(radian);
    wheelPWM1.pulse = 0;
    ClearSpeed();
    SpinRight(speed);
    CommitSpeed();
    while (fabs(wheelPWM1.pulse) < pulse) {}
    Stop();
}

void MoveTo(double speed, double x, double y)
{
    double dx = x - robotInfo.XPos;
    double dy = y - robotInfo.YPos;
    double dis = sqrt(pow(dx, 2) + pow(dy, 2));
    double theta = atan(dy / dx);
    if (dx < 0 && dy > 0) theta += PI;
    if (dx < 0 && dy < 0) theta += PI;
    if (dx > 0 && dy < 0) theta += 2 * PI;
    double det = theta - robotInfo.radian;
    if (det > 0) {
        SpinLeftV2(speed, det);
    } else if (det < 0) {
        SpinRightV2(speed, -det);
    }
    MoveForwardV2(speed, dis);
    //Update Pos Info
    robotInfo.XPos = x;
    robotInfo.YPos = y;
    robotInfo.radian = theta;
}

void PID_Init()
{
    wheelPWM1.target = 0;
    wheelPWM1.minPWM = -1000;
    wheelPWM1.maxPWM = 1000;
    wheelPWM1.kp = 500;
    wheelPWM1.ki = 6;
    wheelPWM1.kd = 0;

    wheelPWM2.target = 0;
    wheelPWM2.minPWM = -1000;
    wheelPWM2.maxPWM = 1000;
    wheelPWM2.kp = 500;
    wheelPWM2.ki = 6;
    wheelPWM2.kd = 0;
}

void Do_PID(struct WheelPWM* wheelPWM)
{
    wheelPWM->lastError = wheelPWM->error;
    wheelPWM->error = wheelPWM->target - wheelPWM->speed;
    wheelPWM->errorI += wheelPWM->error;
    double pwm = wheelPWM->kp * wheelPWM->error + wheelPWM->ki * wheelPWM->errorI + wheelPWM->kd * (wheelPWM->error - wheelPWM->lastError);
    if (pwm > wheelPWM->maxPWM) pwm = wheelPWM->maxPWM;
    if (pwm < wheelPWM->minPWM) pwm = wheelPWM->minPWM;
    wheelPWM->pwm = pwm;
}

void PID_Tick()
{
    wheelPWM1.speed = (short) __HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    wheelPWM1.pulse += wheelPWM1.speed;

    wheelPWM2.speed = (short) __HAL_TIM_GET_COUNTER(&htim8);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    wheelPWM2.pulse += wheelPWM2.speed;

    Do_PID(&wheelPWM1);
    if (wheelPWM1.pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, wheelPWM1.pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    } else if (wheelPWM1.pwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -wheelPWM1.pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }


    Do_PID(&wheelPWM2);
    if (wheelPWM2.pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, wheelPWM2.pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    } else if (wheelPWM2.pwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -wheelPWM2.pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM6) {
        PID_Tick();
    }
}



/*
 * 实际坐标�?
 * 8 8.5
 * 9.5 18
 * 8.5 27.5
 * 4 37
 *
 * -6 44.5
 * -20.5 49
 * -34 49
 * -45 44.5
 *
 * 修正坐标�?
 *
 *
 */

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1 | TIM_CHANNEL_2);

    HAL_TIM_Base_Start_IT(&htim6);

    PID_Init();



    Stop();

    double points[] = {7.35478, 1.56377, 13.6229, 6.07313, 17.833, 13.0008, 19.2331, 21.5279, 17.3726, 30.6274, 12.1537, 39.1698, 3.84847, 46.0391, -6.9224, 50.2492, -19.2453, 51.0482, -32, 48,
                       -43.9667, 41.0364, -53.9452, 30.4721, -60.8729, 16.9837, -63.9308, 1.55155, -62.6274, -14.6274, -56.8514, -30.2492, -46.8884, -44.0236, -33.4, -54.7945, -17.3666, -61.6483,
                       -7.3666, -63.6483, };
    int pointsCnt = 20;

    //初始化机器人的位�?
    robotInfo.XPos = points[0];
    robotInfo.YPos = points[1];
    robotInfo.radian = 0;


    //修正位置
    for (int i = 1; i < pointsCnt; ++i) {
        points[i * 2] *= 0.7;
        points[i * 2 + 1] *= 0.7;
    }
    for (int i = 3; i < pointsCnt; ++i) {
        points[i * 2] *= 1.1;
        points[i * 2 + 1] *= 1.1;
    }
    for (int i = 5; i < pointsCnt; ++i) {
        points[i * 2] *= 1.1;
        points[i * 2 + 1] *= 1.1;
    }
    for (int i = 8; i < pointsCnt; ++i) {
        points[i * 2] *= 1.1;
        points[i * 2 + 1] *= 1.1;
    }
    for (int i = 13; i < pointsCnt; ++i) {
        points[i * 2] *= 1.05;
        points[i * 2 + 1] *= 1.05;
    }
    for (int i = 16; i < pointsCnt; ++i) {
        points[i * 2] *= 0.99;
        points[i * 2 + 1] *= 0.99;
    }


    for (int i = 1; i < pointsCnt; ++i) {
        MoveTo(1, points[i * 2], points[i * 2 + 1]);
        //HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
        HAL_Delay(20);
        //HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    }


    Stop();

    while (1) {}





  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
