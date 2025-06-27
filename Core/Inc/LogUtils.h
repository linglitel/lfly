//
// Created by Administrator on 25-6-19.
//

#ifndef LOGUTILS_H
#define LOGUTILS_H

#include <stdint.h>

#define LOG_ENTRY_SIZE     64               // 每条日志占用64字节
#define LOG_START_ADDR     0x000000         // 日志区起始地址
#define LOG_TOTAL_SIZE     (8 * 1024 * 1024) // 8MB Flash
#define LOG_MAX_ENTRIES    (LOG_TOTAL_SIZE / LOG_ENTRY_SIZE)

// 日志类型枚举（LogType），定义日志的类别
typedef enum
{
    LOG_TYPE_INFO = 0, // 普通信息
    LOG_TYPE_WARNING = 1, // 警告信息
    LOG_TYPE_ERROR = 2, // 错误信息
    LOG_TYPE_DEBUG = 3, // 调试信息
    LOG_TYPE_EVENT = 4, // 事件日志
    LOG_TYPE_CRITICAL = 5, // 严重错误
    LOG_TYPE_MAX // 用于统计类型数量或边界判断
} LogType;


typedef struct
{
    uint32_t timestamp; // 4 bytes
    uint8_t type; // 1 byte，日志类型
    uint8_t module_id; // 1 byte，模块ID
    uint8_t error_code; // 1 byte，错误码
    char message[57]; // 56 bytes，日志内容
} LogEntry;

uint8_t LOG_Read(uint32_t index, LogEntry* log_entry);
void LOG_Write(LogEntry* log_entry);
void LOG_Traverse(void (*process)(const LogEntry*)); //todo need modify

#endif //LOGUTILS_H
