
#include <stdint.h>
#include <string.h>

#include "main_transmit.h"
#include "stm32f1xx_hal.h"

#include "telemetry.h"

extern UART_HandleTypeDef huart3;

// define UART protocol between radio transmitter and transmitter controller
// like mavlink msg
//
// ---------------------------------------
// STX1 | STX2 | LEN | MSG | PAYLOAD | CK|
// ---------------------------------------
//
// total 5 - 260 Bytes
// 
// STX 0xFE
// LEN payload length
// MSG message id 
//     #0 set protocol type. len = 1, payload = 0 UART, payload = 1 PPM
//     #1 do pair len = 2 payload = rx_num, binding/normal
//     #2 channel values len = n, payload = n Bytes
// CK  XOR check

typedef enum {
    COMM_PARSE_STATE_UNINIT = 0,
    COMM_PARSE_STATE_IDLE,
    COMM_PARSE_STATE_GOT_STX1,
    COMM_PARSE_STATE_GOT_STX2,
    COMM_PARSE_STATE_GOT_LENGTH,
    COMM_PARSE_STATE_GOT_MSGID,
    COMM_PARSE_STATE_GOT_PAYLOAD,
    COMM_PARSE_STATE_GOT_XOR,
    COMM_PARSE_STATE_GOT_BAD_XOR,
} comm_parse_state_t;

typedef enum {
    COMM_FRAMING_INCOMPLETE    = 0,
    COMM_FRAMING_OK            = 1,
    COMM_FRAMING_BAD_CHECK_SUM = 2,
} comm_framing_t;


#define COMM_PAYLOAD_LENGTH    255
#define COMM_CHECK_SUM_LENGTH  1

#define COMM_PROTOCOL_PK_STX1  (uint8_t)0xFE
#define COMM_PROTOCOL_PK_STX2  (uint8_t)0x55

#define COMM_MSG_SET_PROTOCOL  0
#define COMM_MSG_DO_PAIR       1
#define COMM_MSG_SEND_CHANNELS 2

#define COMM_CHANNEL_VALUES_LENGTH 32

typedef struct {
    uint8_t checksum;                                           // caculated xor check 

    uint8_t magic1;                                             // STX1
    uint8_t magic2;                                             // STX2
    uint8_t len;                                                // length of payload
    uint8_t msgid;                                              // ID of message in payload
    uint8_t payload[COMM_PAYLOAD_LENGTH + COMM_CHECK_SUM_LENGTH]; //
} comm_message_t;

typedef struct {
    uint8_t msg_received; 
    comm_parse_state_t parse_state; 
    uint8_t packet_idx;             // index in current payload
}comm_status_t;

static comm_message_t m_comm_buffer;
static comm_status_t m_comm_status;

static uint8_t comm_pair_rx_num = 0;
static uint8_t comm_working_mode = 0;
static uint8_t comm_channel_values[COMM_CHANNEL_VALUES_LENGTH];

uint8_t comm_parse_char(uint8_t c, comm_message_t* r_message, comm_status_t* r_status);
uint8_t comm_frame_char(uint8_t c, comm_message_t* r_message, comm_status_t* r_status);
uint8_t comm_frame_char_buffer(comm_message_t* rxmsg, comm_status_t* status,
    uint8_t c, comm_message_t* r_message, comm_status_t* r_status);
void comm_update_checksum(comm_message_t* msg, uint8_t c);


// return 0 or 1
uint8_t comm_parse_char(uint8_t c, comm_message_t* r_message, comm_status_t* r_status){
    uint8_t msg_received = comm_frame_char(c, r_message, r_status);

    if (msg_received == COMM_FRAMING_BAD_CHECK_SUM){
        m_comm_status.parse_state = COMM_PARSE_STATE_IDLE;
        m_comm_status.msg_received = COMM_FRAMING_INCOMPLETE;
        if (c == COMM_PROTOCOL_PK_STX1){
            m_comm_buffer.magic1 = c;
            m_comm_status.parse_state = COMM_PARSE_STATE_GOT_STX1;
        }

        return COMM_FRAMING_INCOMPLETE;
    }
    
    return msg_received;
}

uint8_t comm_frame_char(uint8_t c, comm_message_t* r_message, comm_status_t* r_status){
    return comm_frame_char_buffer(&m_comm_buffer, &m_comm_status, c, r_message, r_status);
}

uint8_t comm_frame_char_buffer(comm_message_t* rxmsg, comm_status_t* status,
    uint8_t c, comm_message_t* r_message, comm_status_t* r_status){

    status->msg_received = COMM_FRAMING_INCOMPLETE;

    switch(status->parse_state){
        case COMM_PARSE_STATE_UNINIT:
        case COMM_PARSE_STATE_IDLE:
            if (c == COMM_PROTOCOL_PK_STX1){
                rxmsg->magic1 = c;
                
                status->parse_state = COMM_PARSE_STATE_GOT_STX1;
            }
        break;
        case COMM_PARSE_STATE_GOT_STX1:
            if (c == COMM_PROTOCOL_PK_STX2){
                rxmsg->magic2 = c;
                rxmsg->len = 0;

                status->parse_state = COMM_PARSE_STATE_GOT_STX2;
            }
        break;
        case COMM_PARSE_STATE_GOT_STX2:
            rxmsg->len = c;
            status->packet_idx = 0;
            comm_update_checksum(rxmsg, c);

            status->parse_state = COMM_PARSE_STATE_GOT_LENGTH;
        break;
        case COMM_PARSE_STATE_GOT_LENGTH:
            rxmsg->msgid = c;
            comm_update_checksum(rxmsg, c);

            if (rxmsg->len == 0){
                status->parse_state = COMM_PARSE_STATE_GOT_PAYLOAD;
            }else {
                status->parse_state = COMM_PARSE_STATE_GOT_MSGID;
            }
        break;
        case COMM_PARSE_STATE_GOT_MSGID:
            rxmsg->payload[status->packet_idx++] = c;
            comm_update_checksum(rxmsg, c);
            if (status->packet_idx == rxmsg->len){
                status->parse_state = COMM_PARSE_STATE_GOT_PAYLOAD;
            }
        break;
        case COMM_PARSE_STATE_GOT_PAYLOAD:
            if (c != rxmsg->checksum){
                status->parse_state = COMM_PARSE_STATE_GOT_BAD_XOR;
            }else {
                status->parse_state = COMM_PARSE_STATE_GOT_XOR;
            }
            rxmsg->payload[status->packet_idx] = c;
        break;
        case COMM_PARSE_STATE_GOT_XOR:
        case COMM_PARSE_STATE_GOT_BAD_XOR:
            if (status->parse_state == COMM_PARSE_STATE_GOT_XOR){
                status->msg_received = COMM_FRAMING_OK;
            }else {
                status->msg_received = COMM_FRAMING_BAD_CHECK_SUM;
            }

            status->parse_state = COMM_PARSE_STATE_IDLE;
            memcpy(r_message, rxmsg, sizeof(comm_message_t));
        break;
        default:
        break;
    }

    r_status->packet_idx = status->packet_idx;
    r_status->parse_state = status->parse_state;

    return status->msg_received;
}

void comm_update_checksum(comm_message_t* msg, uint8_t c){
    
}

void telemetry_comm_proc(uint8_t c){
    comm_message_t msg;
    comm_status_t status;
    
    if (comm_parse_char(c, &msg, &status)){
        switch(msg.msgid){
            case COMM_MSG_DO_PAIR:
                comm_pair_rx_num = msg.payload[0];
                comm_working_mode = msg.payload[1];
            break;
            case COMM_MSG_SEND_CHANNELS:
                if (msg.len <= COMM_CHANNEL_VALUES_LENGTH){
                    memcpy(comm_channel_values, msg.payload, msg.len);
                }
            break;
            default:
            break;
        }
    }
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


