
#include <stdint.h>
#include <string.h>

#include "stm32f1xx_hal.h"


#include "comm_protocol.h"

extern USART_HandleTypeDef hsuart3;

#define COMM_CHANNEL_VALUES_LENGTH 32

static uint8_t comm_pair_rx_num = 0;
static uint8_t comm_working_mode = 0;
static uint8_t comm_channel_values[COMM_CHANNEL_VALUES_LENGTH];


void comm_protocol_parsed_hook(comm_message_t* msg){
    switch(msg->msgid){
        case COMM_MSG_DO_PAIR:
            comm_pair_rx_num = msg->payload[0];
            comm_working_mode = msg->payload[1];
        break;
        case COMM_MSG_SEND_CHANNELS:
            if (msg->len <= COMM_CHANNEL_VALUES_LENGTH){
                memcpy(comm_channel_values, msg->payload, msg->len);
            }
        break;
        default:
        break;
    }
}

void telemetry_comm_proc(uint8_t c){
    comm_protocol_parse(c, comm_protocol_parsed_hook);
}

uint8_t telemetry_rxnum_get(void){
    return comm_pair_rx_num;
}

uint8_t telemetry_transmitter_mode_get(void){
    return comm_working_mode;
}

void telemetry_transmitter_channel_get(uint8_t *addr, uint8_t len)
{
    if (len > COMM_CHANNEL_VALUES_LENGTH){
        memset(addr, 0, len);

        len = COMM_CHANNEL_VALUES_LENGTH;
    }
    
    memcpy(addr, comm_channel_values, len);
}

void telemetry_radio_ack_send(void* buf, uint8_t len){
    comm_message_t msg;
    
    if (comm_protocol_msg_pack(buf, len, &msg) != 0){
        return;
    }
    
    HAL_USART_Transmit(&hsuart3, (uint8_t*)&msg.magic1, COMM_MSG_LEN_EXCEPT_PAYLOAD + msg.len, 5000);
}

