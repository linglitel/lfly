//
// Created by Administrator on 25-6-19.
//

#ifndef W25Q64UTILS_H
#define W25Q64UTILS_H
#include <stdbool.h>

#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

#define W25Q64_CS_GPIO_PORT   GPIOA
#define W25Q64_CS_PIN         GPIO_PIN_4

// --- 指令宏 ---
#define W25Q64_CMD_WRITE_ENABLE   0x06
#define W25Q64_CMD_READ_STATUS    0x05
#define W25Q64_CMD_PAGE_PROGRAM   0x02
#define W25Q64_CMD_READ_DATA      0x03
#define W25Q64_CMD_SECTOR_ERASE   0x20  // 4KB
#define W25Q64_CMD_JEDEC_ID       0x9F

// --- 常量宏 ---
#define W25Q64_PAGE_SIZE          256
#define W25Q64_SECTOR_SIZE        4096

#define W25Q64_CS_LOW()      do {HAL_GPIO_WritePin(W25Q64_CS_GPIO_PORT, W25Q64_CS_PIN, GPIO_PIN_RESET);} while (0)
#define W25Q64_CS_HIGH()     do {HAL_GPIO_WritePin(W25Q64_CS_GPIO_PORT, W25Q64_CS_PIN, GPIO_PIN_SET);} while (0)


void W25Q64_Init(void);
uint32_t W25Q64_Read_ID(void);
void W25Q64_Enable_Write(void);
void W25Q64_Write_Data(uint32_t addr, uint8_t* buf, uint16_t len);
void W25Q64_Read_Data(uint32_t addr, uint8_t* buf, uint16_t len);
void W25Q64_Erase_Data(uint32_t addr); // 擦除一个4K扇区
uint32_t W25Q64_Find_Next_Log_Address(void);

void W25Q64_Write_Log(uint32_t addr, uint8_t* data, uint16_t len);
bool W25Q64_Is_Sector_Erased(uint32_t sector_addr);
void W25Q64_Erase_All();

#endif //W25Q64UTILS_H
