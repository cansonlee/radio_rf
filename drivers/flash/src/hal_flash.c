#include "hal_flash.h"

uint8_t hal_flash_byte_read(uint32_t a)
{
    return *(uint8_t *)a;
}

void hal_flash_bytes_read(uint32_t a, uint8_t *p, uint16_t n)
{
	uint16_t i;

	for(i=0; i<n; i+=2)							//16位地址对齐
	{
		*p++ = hal_flash_byte_read(a+i);		
	}
    return;
}

void hal_flash_bytes_write(uint32_t a, const uint8_t *p, uint16_t n)
{
	uint16_t i;
	for(i=0; i<n; i+=2)							//16位地址对齐
	{
		hal_flash_byte_write(a+i, *p++);		
	}
}

void hal_flash_byte_write(uint32_t a, uint8_t b)
{
	uint16_t data;

	data = b & 0x00ff;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, a, data);
}

void hal_flash_page_erase(uint8_t pn)
{

}

void delay_us(uint16_t us)
{
    uint32_t i;
    do
    {   i = 72;
        do
        {
        } while(--i);
    } while (--us);
}
