
#include <stdint.h>

#include "telemetry_common.h"

#include "cmsis_os.h"

#include "stm32f1xx_hal.h"




UART_HandleTypeDef huart3;

static uint8_t m_uart3_buf; 

static USARTIRQFUNC m_pfUSART3IRQHandle = NULL;

osSemaphoreId uart_sema;
osThreadId uartTaskHandle;


void telemetry_uart3_init(uint32_t baudRate);
void uart_receive_task(void const *argument);

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

	osSemaphoreDef(UART_SEM);
    uart_sema = osSemaphoreEmptyCreate(osSemaphore(UART_SEM));
    if (NULL == uart_sema)
    {
        printf("[%s, L%d] create semaphore failed ret 0x%x.\r\n",
            __FILE__, __LINE__, (unsigned int)uart_sema);
        return -1;
    }

    osThreadDef(uartTask, uart_receive_task, osPriorityAboveNormal, 0, 512);
    uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);
    if (NULL == uartTaskHandle)
    {
        printf("[%s, L%d] create thread failed.\r\n",
            __FILE__, __LINE__);
        return -1;
    }    

	printf("uart3 init ok!\r\n");
    return 0;
}

void telemetry_disable_it(void){
    __HAL_USART_DISABLE_IT(&huart3, UART_IT_RXNE);
	//__HAL_USART_DISABLE_IT(&huart3, USART_IT_IDLE);
}

void telemetry_enable_it(void){
    __HAL_USART_ENABLE_IT(&huart3, UART_IT_RXNE);
	//__HAL_USART_ENABLE_IT(&huart3, USART_IT_IDLE);
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

#define UART3_BUF_LEN	100
uint8_t g_uart3_buf[UART3_BUF_LEN];
uint8_t *p_uart3_in = &g_uart3_buf[0];
uint8_t *p_uart3_out = &g_uart3_buf[0];
uint8_t *p_uart3_head = &g_uart3_buf[0];
uint8_t *p_uart3_tail = &g_uart3_buf[UART3_BUF_LEN -1];

void usart3_irq_handler_callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)
    {
    	*p_uart3_in++ = huart3.Instance->DR;
		if(p_uart3_in > p_uart3_tail){
			p_uart3_in = p_uart3_head;
		}

		(void)xSemaphoreGiveFromISR(uart_sema, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


void uart_receive_task(void const *argument)
{	
	uint8_t len, i;
	uint8_t uart_buf[UART3_BUF_LEN];
	uint32_t flag;
	
    argument = argument;	

    HAL_NVIC_SetPriority(USART3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+3, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

	telemetry_enable_it();
    
    for (;;)
    {
        /* waitting for data update notify */
        (void)osSemaphoreWait(uart_sema, osWaitForever);

		MCU_INTERRUPTS_DISABLE(flag);
		if(p_uart3_out <= p_uart3_in)
		{
			len = p_uart3_in - p_uart3_out;
			memcpy(uart_buf, p_uart3_out, len);
		}
		else
		{
			i = p_uart3_tail - p_uart3_out + 1;
			memcpy(uart_buf, p_uart3_out, i);
			len = p_uart3_in - p_uart3_head;
			memcpy(&uart_buf[i], p_uart3_head, len);
			len += i;			
		}	
		p_uart3_out = p_uart3_in;
		MCU_INTERRUPTS_ENABLE(flag);

		printf("recv in %s, %s, L:%d\r\n", __FILE__, __func__, __LINE__);

		for(i=0; i<len; i++)
		{
			m_pfUSART3IRQHandle(uart_buf[i]);
			printf("%#x ", uart_buf[i]);
		}

		printf("\r\n");
		

    }
}


