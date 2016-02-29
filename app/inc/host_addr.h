#ifndef __HOST_ADDR_H__
#define __HOST_ADDR_H__


#define MCU_U_ID_15_0		((uint32_t) 0x1FFFF7E8)
#define MCU_U_ID_31_16		((uint32_t) 0x1FFFF7EA)
#define MCU_U_ID_63_32		((uint32_t) 0x1FFFF7EC)
#define MCU_U_ID_95_64		((uint32_t) 0x1FFFF7F0)

typedef struct host_addr_uniq_id_s
{
	uint16_t	ID_15_0;
	uint16_t	ID_31_16;
	uint32_t	ID_63_32;
	uint32_t	ID_95_64;
}HOST_ADDR_UNIQ_ID_STRU;

int host_chip_id_read(uint8_t *dst, uint8_t len);



#endif
