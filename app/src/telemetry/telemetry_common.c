
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

	printf("uart3 init ok!\r\n");

    if (pfInit != NULL){
        ret = pfInit();
        if (ret < 0){
            return ret;
        }
    }

    m_pfUSART3IRQHandle = pfIRQ;

    HAL_NVIC_SetPriority(USART3_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

//    HAL_UART_Receive_IT(&huart3, &m_uart3_buf, 1);
	telemetry_enable_it();

	printf("uart3 init ok!\r\n");
    return 0;
}

void telemetry_disable_it(void){
    __HAL_USART_DISABLE_IT(&huart3, UART_IT_RXNE);
	__HAL_USART_DISABLE_IT(&huart3, USART_IT_IDLE);
}

void telemetry_enable_it(void){
    __HAL_USART_ENABLE_IT(&huart3, UART_IT_RXNE);
	__HAL_USART_ENABLE_IT(&huart3, USART_IT_IDLE);
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

#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	printf("enter %d\r\n", __LINE__);
	
    if (huart->Instance == USART3){
//        if (m_pfUSART3IRQHandle != NULL){
//            m_pfUSART3IRQHandle(huart->pRxBuffPtr[0]);
//        }  
		printf("recv %x\r\n", huart->pRxBuffPtr[0]);
//		UART_Receive_IT(&huart);
    }
}
#endif

uint8_t g_uart3_buf[100];
uint16_t g_uart3_len;
void usart3_irq_handler_callback(void)
{
	uint8_t c;
	uint16_t temp;
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)
    {
    	c = huart3.Instance->DR;
//    	printf("0x%x \r\n", c);
		g_uart3_buf[g_uart3_len++] = c;

		if(g_uart3_len >= 100)
			g_uart3_len = 0;
//		c = huart3.Instance->DR;
//        printf("0x%x \r\n", c);
    }

	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)
	{

		temp = huart3.Instance->DR;
		temp = huart3.Instance->SR;

		if (m_pfUSART3IRQHandle != NULL){
			for(uint16_t i=0; i<g_uart3_len; i++)
			{
            	m_pfUSART3IRQHandle(g_uart3_buf[i]);
			}
		}

		g_uart3_len = 0;
	}
}

