#ifndef __TELEMETRY_COMMON_H__
#define __TELEMETRY_COMMON_H__

#define MCU_INTERRUPTS_DISABLE(flag) do{ \
   flag = __get_PRIMASK();                \
  __disable_irq();                        \
}while(0)                  

#define MCU_INTERRUPTS_ENABLE(flag) do{  \
  if(flag)                                \
    __enable_irq();                       \
}while(0)


typedef void (*USARTIRQFUNC)(uint8_t);
typedef int32_t (*USARTINITFUNC)(void);

int32_t telemetry_init(USARTINITFUNC pfInit, USARTIRQFUNC pfIRQ, uint32_t baudRate);

void telemetry_disable_it(void);
void telemetry_enable_it(void);

#endif
