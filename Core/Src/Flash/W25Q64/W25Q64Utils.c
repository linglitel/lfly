//
// Created by Administrator on 25-6-19.
//

#include "W25Q64Utils.h"

#include <stdbool.h>

#include "cmsis_os2.h"
#include "freertos_os2.h"
#include "LogUtils.h"
#include "main.h"

extern UART_HandleTypeDef huart1;

void W25Q64_Init()
{
    uint32_t id = W25Q64_Read_ID();
    if (id != 0xEF4017)
    {
        //TODO ,we need find a batter way to solve this question
        //if logger disable or broken,we can`t save any data
        //when it broken make it shutdown and turn LED
        //if all broken,i love you
        //:)
        //todo turn on led
        Error_Handler();
        //todo fuck this simple error handle,i can`t enough to accept **NO LOG ,NO INFO**,i will kill it and use my own error handler
    }
    else
    {
        if (W25Q64_Find_Next_Log_Address() == 0xFFFFFFFF)
        {
            const char buf[] = "ERROR: Storage space is full\n";
            HAL_UART_Transmit(&huart1, buf, strlen(buf), HAL_MAX_DELAY);
        }
        //todo need more handler
    }
}

uint32_t W25Q64_Read_ID(void)
{
    uint8_t cmd = W25Q64_CMD_JEDEC_ID;
    uint8_t id[3] = {0};

    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, id, 3, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();

    return (id[0] << 16) | (id[1] << 8) | id[2];
}

void W25Q64_Enable_Write(void)
{
    uint8_t cmd = W25Q64_CMD_WRITE_ENABLE;
    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();
}

static uint8_t W25Q64_Read_Status(void)
{
    uint8_t cmd = W25Q64_CMD_READ_STATUS;
    uint8_t status = 0;

    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();

    return status;
}

static void W25Q64_Wait_Busy(void)
{
    while (W25Q64_Read_Status() & 0x01)
    {
        //osDelay(1); // FreeRTOS 非阻塞延时，1个Tick
        //mast use it  in tasks
        HAL_Delay(1);
    }
}

void W25Q64_Write_Data(uint32_t addr, uint8_t* buf, uint16_t len)
{
    if (len == 0) return;

    if (len > W25Q64_PAGE_SIZE) len = W25Q64_PAGE_SIZE; // 每页最大256字节

    W25Q64_Enable_Write();

    uint8_t cmd[4] = {
        W25Q64_CMD_PAGE_PROGRAM, // 页写入命令
        (addr >> 16) & 0xFF,
        (addr >> 8) & 0xFF,
        addr & 0xFF
    };

    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, buf, len, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();

    W25Q64_Wait_Busy();
}

void W25Q64_Read_Data(uint32_t addr, uint8_t* buf, uint16_t len)
{
    if (len == 0) return;

    uint8_t cmd[4] = {
        W25Q64_CMD_READ_DATA, // 读数据命令
        (addr >> 16) & 0xFF,
        (addr >> 8) & 0xFF,
        addr & 0xFF
    };

    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();
}

void W25Q64_Erase_Data(uint32_t addr)
{
    W25Q64_Enable_Write();

    uint8_t cmd[4] = {
        W25Q64_CMD_SECTOR_ERASE, // 扇区擦除命令 (4KB)
        (addr >> 16) & 0xFF,
        (addr >> 8) & 0xFF,
        addr & 0xFF
    };

    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();

    W25Q64_Wait_Busy();
}

uint32_t W25Q64_Find_Next_Log_Address(void)
{
    uint8_t buffer[LOG_ENTRY_SIZE];
    for (uint32_t addr = LOG_START_ADDR; addr < LOG_TOTAL_SIZE; addr += LOG_ENTRY_SIZE)
    {
        W25Q64_Read_Data(addr, buffer, LOG_ENTRY_SIZE);
        uint8_t is_empty = 1;
        for (int i = 0; i < LOG_ENTRY_SIZE; i++)
        {
            if (buffer[i] != 0xFF)
            {
                is_empty = 0;
                break;
            }
        }
        if (is_empty)
            return addr;
    }
    return 0xFFFFFFFF; // 没有空间了
}

bool W25Q64_Is_Sector_Erased(uint32_t sector_addr)
{
    uint8_t buffer[W25Q64_SECTOR_SIZE];

    // 读取扇区数据
    W25Q64_Read_Data(sector_addr, buffer, W25Q64_SECTOR_SIZE);

    // 检查是否全为0xFF
    for (uint32_t i = 0; i < W25Q64_SECTOR_SIZE; i++)
    {
        if (buffer[i] != 0xFF)
        {
            return false;
        }
    }
    return true;
}

void W25Q64_Write_Log(uint32_t addr, uint8_t* data, uint16_t len)
{
    // 计算所在扇区地址（4KB对齐）
    uint32_t sector_addr = addr & ~(W25Q64_SECTOR_SIZE - 1);

    // 检查是否需要擦除
    if (!W25Q64_Is_Sector_Erased(sector_addr))
    {
        W25Q64_Erase_Data(sector_addr);
    }

    // 直接写数据
    W25Q64_Write_Data(addr, data, len);
}


//no use it , but i need it
void W25Q64_Erase_All()
{
    uint8_t cmd;

    // 写使能（0x06）
    cmd = 0x06;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // CS HIGH

    // 发送整片擦除指令（0xC7）
    cmd = 0xC7;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    // 等待擦除完成（读状态寄存器 0x05）
    cmd = 0x05;
    uint8_t status = 0x01;
    while (status & 0x01)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_Delay(10); // 防止轮询太快
    }
}
