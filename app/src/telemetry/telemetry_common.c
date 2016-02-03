
#include <stdint.h>

#include "telemetry_common.h"

#include "cmsis_os.h"

#include "stm32f1xx_hal.h"



USART_HandleTypeDef hsuart3;

static uint8_t m_usart3_buf; 

static USARTIRQFUNC m_pfUSART3IRQHandle = NULL;

void telemetry_usart3_init(uint32_t baudRate);

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

    HAL_USART_Receive_IT(&hsuart3, &m_usart3_buf, 1);

    return 0;
}

/* USART3 init function */
void telemetry_uart3_init(uint32_t baudRate)
{

    hsuart3.Instance = USART3;
    hsuart3.Init.BaudRate = baudRate;
    hsuart3.Init.WordLength = UART_WORDLENGTH_8B;
    hsuart3.Init.StopBits = UART_STOPBITS_1;
    hsuart3.Init.Parity = UART_PARITY_NONE;
    hsuart3.Init.Mode = UART_MODE_TX_RX;
    //hsuart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    //hsuart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_USART_Init(&hsuart3);

    return;
}

void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart)
{
    if (husart->Instance == USART3){
        if (m_pfUSART3IRQHandle != NULL){
            m_pfUSART3IRQHandle(husart->pRxBuffPtr[0]);
        }  
    }
}


