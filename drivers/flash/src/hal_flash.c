#include "hal_flash.h"

uint8_t hal_flash_byte_read(uint32_t a)
{
    return *(uint8_t *)a;
}

void hal_flash_bytes_read(uint32_t a, uint8_t *p, uint16_t n)
{
	uint16_t i;

	for(i=0; i<n; i++)
	{
		*p++ = hal_flash_byte_read(a+i*2);		//16位地址对齐
	}
    return;
}

void hal_flash_bytes_write(uint32_t a, const uint8_t *p, uint16_t n)
{
	uint16_t i,data;
	for(i=0; i<n; i++)
	{
		data = 0x0000 | *p++;
		hal_flash_byte_write(a+i*2, data);		//16位地址对齐
	}
}

void hal_flash_byte_write(uint32_t a, uint8_t b)
{
	uint16_t data;

	data = 0x0000 | b;
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
