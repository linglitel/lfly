//
// Created by Administrator on 25-6-26.
//

#include "JY901PUtils.h"

#include "LogUtils.h"
#include "SensorUtils.h"
#include "stm32f4xx_hal.h"
#include "wit_c_sdk.h"



extern UART_HandleTypeDef huart6;

static void JY901P_USART_SEND(uint8_t* pData, uint16_t len);
static void JY901P_Callback(uint32_t reg, uint32_t len);

bool JY901P_Init()
{
    int32_t ret;
    LogEntry log;
    log.module_id = JY901P_MOUDLE_ID;

    log.timestamp = HAL_GetTick();

    // 初始化SDK，设备地址0x01
    ret = WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    if (ret != WIT_HAL_OK)
    {
        log.type = LOG_TYPE_ERROR;
        log.error_code = ret;
        snprintf(log.message, sizeof(log.message), "WitInit failed");
        LOG_Write(&log);
        return false;
    };

    // 注册UART写函数
    ret = WitSerialWriteRegister(JY901P_USART_SEND);
    if (ret != WIT_HAL_OK)
    {
        log.type = LOG_TYPE_ERROR;
        log.error_code = ret;
        snprintf(log.message, sizeof(log.message), "WitSerialWriteRegister failed");
        LOG_Write(&log);
        return false;
    };

    // 注册延时函数
    ret = WitDelayMsRegister(Delay_ms);
    if (ret != WIT_HAL_OK)
    {
        log.type = LOG_TYPE_ERROR;
        log.error_code = ret;
        snprintf(log.message, sizeof(log.message), "WitDelayMsRegister failed");
        LOG_Write(&log);
        return false;
    };

    // 注册数据更新回调（这里演示只打印寄存器变更，实际你改）
    ret = WitRegisterCallBack(JY901P_Callback);
    if (ret != WIT_HAL_OK)
    {
        log.type = LOG_TYPE_ERROR;
        log.error_code = ret;
        snprintf(log.message, sizeof(log.message), "WitRegisterCallBack failed");
        LOG_Write(&log);
        return false;
    }
    ret = WitSetContent(RSW_Q);
    if (ret != WIT_HAL_OK)
    {
        log.type = LOG_TYPE_ERROR;
        log.error_code = ret;
        snprintf(log.message, sizeof(log.message), "WitSetContent failed");
        LOG_Write(&log);
        return false;
    }
    ret = WitSetOutputRate(RRATE_200HZ);
    if (ret != WIT_HAL_OK)
    {
        log.type = LOG_TYPE_ERROR;
        log.error_code = ret;
        snprintf(log.message, sizeof(log.message), "WitSetOutputRate failed");
        LOG_Write(&log);
        return false;
    }
    log.type = LOG_TYPE_INFO;
    log.error_code = 0;
    snprintf(log.message, sizeof(log.message), "JY901P init success");
    LOG_Write(&log);
    return true;
}

static void JY901P_USART_SEND(uint8_t* pData, uint16_t len)
{
    int ret;
    if ((ret = HAL_UART_Transmit(&huart6, pData, len, 100)) != HAL_OK)
    {
        //LOG
        LogEntry log;
        log.module_id = 0;
        log.error_code = ret;
        log.type = LOG_TYPE_ERROR;
        log.timestamp = HAL_GetTick();
        snprintf(log.message, sizeof(log.message), "HAL_UART_Transmit failed");
        LOG_Write(&log);
    }
}

static void JY901P_Callback(uint32_t reg, uint32_t len)
{
    LogEntry log;
    log.type = LOG_TYPE_INFO;
    log.timestamp = HAL_GetTick();
    log.module_id = JY901P_MOUDLE_ID;
    log.error_code = 0;
    snprintf(log.message, sizeof(log.message), "Reg %lu updated, len %lu", reg, len);
    LOG_Write(&log);
}

