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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

#include "AK8963Utils.h"
#include "MPU9250Utils.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU9250_ADDR  (0x68 << 1)  // 7-bit 地址左移一位
#define AK8963_ADDR   (0x0C << 1)  // AK8963 地址

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#include <math.h>
#include <stdbool.h>

#include "AttitudeTask.h"
#include "Madgwick.h"
#include "SensorUtils.h"
#include "stdio.h"
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

struct __FILE
{
    int handle; /* Add whatever you need here */
};

FILE __stdout;
FILE __stdin;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* USER CODE BEGIN PV */
osThreadId_t imuTaskHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void* argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE* f)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
    return ch;
}


float mag_sensitivity_adjust[3] = {1.0f, 1.0f, 1.0f}; // 你的磁力计校准比例因子，示例1.0f
float mag_offset[3] = {-10.33f, -236.35f, 147.34f}; // 你的硬铁偏移量

// 简单归一化磁力计数据，计算方位角（弧度）
float calculate_heading(float mx, float my)
{
    float heading = atan2f(my, mx); // atan2(y, x)，返回[-pi, pi]
    if (heading < 0) heading += 2 * M_PI; // 转换到[0, 2pi]
    return heading;
}

float compute_yaw(float mx, float my, float mz, float roll, float pitch)
{
    float sin_roll = sinf(roll);
    float cos_roll = cosf(roll);
    float sin_pitch = sinf(pitch);
    float cos_pitch = cosf(pitch);

    // 将磁力计读数从机体坐标系转换到水平坐标系
    float mag_x_horizontal = mx * cos_pitch + my * sin_pitch * sin_roll - mz * sin_pitch * cos_roll;
    float mag_y_horizontal = my * cos_roll + mz * sin_roll;

    // 计算偏航角
    // atan2f(y, x) -> atan2f(East, North)
    float yaw = atan2f(mag_y_horizontal, mag_x_horizontal);

    // 将偏航角从 [-π, π] 转换到 [0, 2π)
    if (yaw < 0)
    {
        yaw += 2 * M_PI;
    }
    return yaw;
}

void get_roll_pitch(float ax, float ay, float az, float* roll, float* pitch)
{
    *roll = atan2f(ay, az);
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
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
    MX_CRC_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();
    MX_TIM11_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */


    uint8_t data;

    // === MPU9250 初始化 ===


    // 1. 检查 WHO_AM_I
    data = 0;
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x75, 1, &data, 1, HAL_MAX_DELAY);
    if (data != 0x71)
    {
        Error_Handler();
    }

    // 2. 退出休眠，设置时钟源为陀螺仪X轴（0x01）
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x6B, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(100);

    // 3. 设置加速度计量程 ±2g（0x00）
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x1C, 1, &data, 1, HAL_MAX_DELAY);

    // 4. 设置陀螺仪量程 ±250°/s（0x00）
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x1B, 1, &data, 1, HAL_MAX_DELAY);

    // 5. 打开 I2C 主模式 bypass，使主控可直接访问 AK8963
    data = 0x02; // BIT1 = I2C_BYPASS_EN
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x37, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // 设置加速度计低通滤波器（如44.8Hz截止频率）
    data = 0x06; // 配置ACCEL_CONFIG2寄存器，DLPF_CFG=0x03
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x1D, 1, &data, 1, HAL_MAX_DELAY);

    // 设置陀螺仪低通滤波器（如41Hz截止频率）
    data = 0x06; // 配置CONFIG寄存器，DLPF_CFG=0x03
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x1A, 1, &data, 1, HAL_MAX_DELAY);

    // === AK8963 初始化 ===

    uint8_t asa[3];

    // 1. 检查 AK8963 WHO_AM_I
    data = 0;
    HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, 0x00, 1, &data, 1, HAL_MAX_DELAY);
    if (data != 0x48)
    {
        Error_Handler();
    }
    data = 0x06;
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, 0x0A, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // 进入 Fuse ROM 读取模式
    data = 0x0F;
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, 0x0A, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // 读取 ASA 寄存器（0x10~0x12）
    HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, 0x10, 1, asa, 3, HAL_MAX_DELAY);

    // 退出 Fuse ROM 模式（进入 Power-down）
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, 0x0A, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    data = 0x16; // 0b00010110 -> 16-bit | Continuous Mode 1
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, 0x0A, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // 保存校准因子
    float mag_sensitivity_adjust[3];
    mag_sensitivity_adjust[0] = ((asa[0] - 128) / 256.0f + 1.0f);
    mag_sensitivity_adjust[1] = ((asa[1] - 128) / 256.0f + 1.0f);
    mag_sensitivity_adjust[2] = ((asa[2] - 128) / 256.0f + 1.0f);

    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    int16_t mx_raw, my_raw, mz_raw;

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    float roll = 0, pitch = 0, yaw = 0;
    uint32_t last_send_time = 0; // 记录上次发送时间，单位ms
    while (1)
    {
        // 读取加速度计原始值
        HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x3B, 1, (uint8_t*)&ax_raw, 2, HAL_MAX_DELAY);
        HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x3D, 1, (uint8_t*)&ay_raw, 2, HAL_MAX_DELAY);
        HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x3F, 1, (uint8_t*)&az_raw, 2, HAL_MAX_DELAY);

        // 读取陀螺仪原始值
        HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x43, 1, (uint8_t*)&gx_raw, 2, HAL_MAX_DELAY);
        HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x45, 1, (uint8_t*)&gy_raw, 2, HAL_MAX_DELAY);
        HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x47, 1, (uint8_t*)&gz_raw, 2, HAL_MAX_DELAY);

        uint8_t mag_status;

        // 1. 检查 ST1 寄存器 (0x02)，看数据是否就绪 (DRDY bit is 1)
        HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, 0x02, 1, &mag_status, 1, HAL_MAX_DELAY);
        uint8_t mag_buf[7];
        // 2. 如果数据就绪 (mag_status 的第0位为1)
        if (mag_status & 0x01)
        {
            // 高效读取方法：一次性读取从 HXL (0x03) 到 ST2 (0x09) 的所有7个字节
            // 这样既拿到了数据，也完成了读取 ST2 的操作

            HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, 0x03, 1, mag_buf, 7, HAL_MAX_DELAY);

            // mag_buf[6] 现在是 ST2 寄存器的值，可以用来检查数据是否溢出
            // 如果 ST2 的 HOFL 位 (bit3) 为0，则数据有效
            if (!(mag_buf[6] & 0x08))
            {
                // AK8963 是小端序 (Little Endian)，低字节在前
                mx_raw = (int16_t)(mag_buf[1] << 8 | mag_buf[0]);
                my_raw = (int16_t)(mag_buf[3] << 8 | mag_buf[2]);
                mz_raw = (int16_t)(mag_buf[5] << 8 | mag_buf[4]);
            }
            // 读取 mag_buf[6] 的这个动作本身就已经完成了“解锁”传感器的任务
        }


        // 原始数据是高低字节连续，需要根据字节顺序转换（假设小端）
        ax_raw = (int16_t)((((uint8_t*)&ax_raw)[0] << 8) | ((uint8_t*)&ax_raw)[1]);
        ay_raw = (int16_t)((((uint8_t*)&ay_raw)[0] << 8) | ((uint8_t*)&ay_raw)[1]);
        az_raw = (int16_t)((((uint8_t*)&az_raw)[0] << 8) | ((uint8_t*)&az_raw)[1]);

        gx_raw = (int16_t)((((uint8_t*)&gx_raw)[0] << 8) | ((uint8_t*)&gx_raw)[1]);
        gy_raw = (int16_t)((((uint8_t*)&gy_raw)[0] << 8) | ((uint8_t*)&gy_raw)[1]);
        gz_raw = (int16_t)((((uint8_t*)&gz_raw)[0] << 8) | ((uint8_t*)&gz_raw)[1]);

        // 单位换算
        ax = (float)ax_raw / 16384.0f; // ±2g量程
        ay = (float)ay_raw / 16384.0f;
        az = (float)az_raw / 16384.0f;

        gx = (float)gx_raw / 131.0f; // ±250°/s量程
        gy = (float)gy_raw / 131.0f;
        gz = (float)gz_raw / 131.0f;

        mx = (float)mx_raw * mag_sensitivity_adjust[0] - 10.33f;
        my = (float)my_raw * mag_sensitivity_adjust[1] - 236.35f;
        mz = (float)mz_raw * mag_sensitivity_adjust[2] + 147.34f;


        get_roll_pitch(ax, ay, az, &roll, &pitch);
        // 计算方位角（单位度）
        yaw = compute_yaw(mx, my, mz, roll, pitch);
        // 输出结果
        uint32_t current_time = HAL_GetTick(); // 获取当前系统运行时间，单位ms

        if (current_time - last_send_time >= 100) // 距离上次发送超过100ms
        {
            last_send_time = current_time;

            char uart_buf[100];
            char buf[100];
            int len = snprintf(buf, sizeof(buf),
                               "%.2f, %.2f, %.2f\n",
                               roll * 180.0f / M_PI,
                               pitch * 180.0f / M_PI,
                               yaw * 180.0f / M_PI);
            HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);
        }
    }
    for (;;);
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

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
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    const osThreadAttr_t imuTask_attributes = {
        .name = "IMU_Task",
        .stack_size = 256 * 4, // 256字，4字节/字
        .priority = (osPriority_t)osPriorityNormal,
    };

    imuTaskHandle = osThreadNew(AttitudeTask, NULL, &imuTask_attributes);
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

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

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 100;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{
    /* USER CODE BEGIN CRC_Init 0 */

    /* USER CODE END CRC_Init 0 */

    /* USER CODE BEGIN CRC_Init 1 */

    /* USER CODE END CRC_Init 1 */
    hcrc.Instance = CRC;
    if (HAL_CRC_Init(&hcrc) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CRC_Init 2 */

    /* USER CODE END CRC_Init 2 */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
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
    htim2.Init.Prescaler = 99;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 19999;
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
    sConfigOC.Pulse = 150;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 1500;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
    HAL_TIM_MspPostInit(&htim2);
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
    htim11.Init.Prescaler = 99;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = 65535;
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
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PA4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void* argument)
{
    /* USER CODE BEGIN 5 */
    UNUSED(argument);
    /* Infinite loop */
    for (;;)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        osDelay(1);
    }
    /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM1)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
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
