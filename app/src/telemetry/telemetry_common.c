
#include <stdint.h>

#include "telemetry_common.h"

#include "cmsis_os.h"

#include "stm32f1xx_hal.h"

UART_HandleTypeDef huart3;

static USARTIRQFUNC m_pfUSART3IRQHandle = NULL;

void telemetry_uart3_init(uint32_t baudRate);

int32_t telemetry_init(USARTINITFUNC pfInit, USARTIRQFUNC pfIRQ, uint32_t baudRate)
{
    int32_t ret;
    
    telemetry_uart3_init(baudRate);

    if (pfInit != NULL){
        ret = pfInit();
        if (ret < 0){
            return ret;
        }
    }

    m_pfUSART3IRQHandle = pfIRQ;

    HAL_NVIC_SetPriority(USART3_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    return 0;
}

/* USART3 init function */
void telemetry_uart3_init(uint32_t baudRate)
{

    huart3.Instance = USART3;
    huart3.Init.BaudRate = baudRate;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);

    return;
}

void USART3_IRQHandler(void)
{
    uint8_t c;
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)
    {
        c = huart3.Instance->DR;
        //printf("0x%x \r\n", c);
        if (m_pfUSART3IRQHandle != NULL){
            m_pfUSART3IRQHandle(c);
        }       
    }
}


