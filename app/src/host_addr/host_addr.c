#include <stdio.h>

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "host_addr.h"



int host_chip_id_read(uint8_t *dst, uint8_t len)
{
    HOST_ADDR_UNIQ_ID_STRU mcu_uniq_id;

	if(len > 12)
	{
		return -1;
	}

	if(dst == NULL)
	{
		return -1;
	}

	mcu_uniq_id.ID_15_0  = *(uint32_t *)MCU_U_ID_15_0;
	mcu_uniq_id.ID_31_16 = *(uint32_t *)MCU_U_ID_31_16;
	mcu_uniq_id.ID_63_32 = *(uint32_t *)MCU_U_ID_63_32;
	mcu_uniq_id.ID_95_64 = *(uint32_t *)MCU_U_ID_95_64;

	memcpy(dst, &mcu_uniq_id, len);

    return 0;
}

