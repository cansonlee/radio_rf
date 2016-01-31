#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "led.h"

void led_blink
(
    uint8_t led, 
    uint8_t times, 
    uint32_t blink_interval
)
{
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    uint8_t i;
    
    switch (led)
    {
        case LED_0:
            GPIOx = GPIOB;
            GPIO_Pin = GPIO_PIN_13;
            break;
        case LED_1:
            GPIOx = GPIOB;
            GPIO_Pin = GPIO_PIN_14;
            break;
        default:
            GPIOx = GPIOB;
            GPIO_Pin = GPIO_PIN_14;
            break;            
    }

    for (i = 0; i < times; i++)
    {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
        osDelay(blink_interval);
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
        osDelay(blink_interval);
    }

    return;
}

int32_t led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    /* LED 0 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    /* LED 1 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    return 0;
}

