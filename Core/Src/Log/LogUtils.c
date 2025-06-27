//
// Created by Administrator on 25-6-19.
//

#include "LogUtils.h"

#include <string.h>

#include "W25Q64Utils.h"

static uint32_t s_log_start_addr = LOG_START_ADDR;
static uint32_t s_log_total_size = LOG_TOTAL_SIZE;
static uint32_t s_log_entry_size = LOG_ENTRY_SIZE;
static uint32_t s_log_max_entries = LOG_MAX_ENTRIES;

void LOG_Write(LogEntry* log_entry)
{
    uint32_t addr = W25Q64_Find_Next_Log_Address();
    if (addr == 0xFFFFFFFF)
    {
        // 日志满了，处理逻辑，比如删除旧日志或报警
        return;
    }

    uint32_t sector_addr = addr & ~(W25Q64_SECTOR_SIZE - 1);
    if (!W25Q64_Is_Sector_Erased(sector_addr))
    {
        W25Q64_Erase_Data(sector_addr);
    }

    W25Q64_Write_Data(addr, (uint8_t*)log_entry, LOG_ENTRY_SIZE);
}

uint8_t LOG_Read(uint32_t index, LogEntry* log_entry)
{
    if (!log_entry || index >= s_log_max_entries)
        return false;

    uint32_t addr = s_log_start_addr + index * s_log_entry_size;
    W25Q64_Read_Data(addr, (uint8_t*)log_entry, s_log_entry_size);

    if (log_entry->timestamp == 0xFFFFFFFF || log_entry->timestamp == 0)
        return false; // 无有效日志

    return true;
}

void LOG_Delete(uint32_t index)
{
    if (index >= s_log_max_entries)
        return;

    uint32_t addr = s_log_start_addr + index * s_log_entry_size;
    LogEntry empty_entry;
    memset(&empty_entry, 0xFF, sizeof(LogEntry)); // 填满0xFF，模拟擦除状态

    W25Q64_Write_Data(addr, (uint8_t*)&empty_entry, sizeof(LogEntry));
}

void LOG_Traverse(void (*process)(const LogEntry*))
{
    if (!process) return;

    LogEntry entry;
    for (uint32_t i = 0; i < s_log_max_entries; i++)
    {
        if (LOG_Read(i, &entry))
        {
            process(&entry);
        }
    }
}
