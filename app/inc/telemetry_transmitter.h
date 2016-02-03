#ifndef __TELEMETRY_TRANSMITTER_H__
#define __TELEMETRY_TRANSMITTER_H__

uint8_t telemetry_rxnum_get(void);
uint8_t telemetry_transmitter_mode_get(void);
void telemetry_transmitter_channel_get(uint8_t *addr, uint8_t len);
void telemetry_radio_ack_send(void* buf, uint8_t len);

#endif
