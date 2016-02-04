
#include <stdint.h>

#include "telemetry_common.h"

#include "cmsis_os.h"

#include "stm32f1xx_hal.h"



UART_HandleTypeDef huart3;

static uint8_t m_uart3_buf; 

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

    HAL_UART_Receive_IT(&huart3, &m_uart3_buf, 1);

    return 0;
}

void telemetry_disable_it(void){
    __HAL_USART_DISABLE_IT(&huart3, UART_IT_RXNE);
}

void telemetry_enable_it(void){
    __HAL_USART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

void telemetry_transmit(uint8_t *pTxData, uint16_t len, uint32_t timeout){
    HAL_UART_Transmit(&huart3, pTxData, len, timeout);
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3){
        if (m_pfUSART3IRQHandle != NULL){
            m_pfUSART3IRQHandle(huart->pRxBuffPtr[0]);
        }  
    }
}

