#ifndef __TELEMETRY_COMMON_H__
#define __TELEMETRY_COMMON_H__

typedef void (*USARTIRQFUNC)(uint8_t);
typedef int32_t (*USARTINITFUNC)(void);

int32_t telemetry_init(USARTINITFUNC pfInit, USARTIRQFUNC pfIRQ, uint32_t baudRate);

#endif
