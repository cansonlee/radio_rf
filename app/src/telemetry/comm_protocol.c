
#include <stdint.h>
#include <string.h>

#include "comm_protocol.h"

static comm_message_t m_comm_buffer;
static comm_status_t m_comm_status;

uint8_t comm_protocol_parse_char(uint8_t c, comm_message_t* r_message, comm_status_t* r_status);
uint8_t comm_protocol_frame_char(uint8_t c, comm_message_t* r_message, comm_status_t* r_status);
uint8_t comm_protocol_frame_char_buffer(comm_message_t* rxmsg, comm_status_t* status,
    uint8_t c, comm_message_t* r_message, comm_status_t* r_status);
void comm_protocol_checksum_start(comm_message_t* msg, uint8_t c);
void comm_protocol_checksum_update(comm_message_t* msg, uint8_t c);
int8_t telemetry_comm_msg_pack(void*buf, uint8_t len, comm_message_t* msg);

void comm_protocol_parse(uint8_t c, void (*pfParsedHook)(comm_message_t*)){
    comm_message_t msg;
    comm_status_t status;
    
    if (comm_protocol_parse_char(c, &msg, &status)){
        if (pfParsedHook != NULL){
            pfParsedHook(&msg);
        }
    }
}

// return 0 or 1
uint8_t comm_protocol_parse_char(uint8_t c, comm_message_t* r_message, comm_status_t* r_status){
    uint8_t msg_received = comm_protocol_frame_char(c, r_message, r_status);

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

uint8_t comm_protocol_frame_char(uint8_t c, comm_message_t* r_message, comm_status_t* r_status){
    return comm_protocol_frame_char_buffer(&m_comm_buffer, &m_comm_status, c, r_message, r_status);
}

uint8_t comm_protocol_frame_char_buffer(comm_message_t* rxmsg, comm_status_t* status,
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
            comm_protocol_checksum_start(rxmsg, c);

            status->parse_state = COMM_PARSE_STATE_GOT_LENGTH;
        break;
        case COMM_PARSE_STATE_GOT_LENGTH:
            rxmsg->msgid = c;
            comm_protocol_checksum_update(rxmsg, c);

            if (rxmsg->len == 0){
                status->parse_state = COMM_PARSE_STATE_GOT_PAYLOAD;
            }else {
                status->parse_state = COMM_PARSE_STATE_GOT_MSGID;
            }
        break;
        case COMM_PARSE_STATE_GOT_MSGID:
            rxmsg->payload[status->packet_idx++] = c;
            comm_protocol_checksum_update(rxmsg, c);
            if (status->packet_idx == rxmsg->len){
                status->parse_state = COMM_PARSE_STATE_GOT_PAYLOAD;
            }
        break;
        case COMM_PARSE_STATE_GOT_PAYLOAD:
            if (c != rxmsg->calc_checksum){
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

void comm_protocol_checksum_start(comm_message_t* msg, uint8_t c){
    msg->calc_checksum = c;
}

void comm_protocol_checksum_update(comm_message_t* msg, uint8_t c){
    msg->calc_checksum ^= c;
}


// return the length of the message need to send
// < 0 error
// packed msg stored in param msg
int8_t comm_protocol_msg_pack(uint8_t id, void*buf, uint8_t len, comm_message_t* msg){

    uint8_t* ptr = buf;
        
    msg->magic1 = COMM_PROTOCOL_PK_STX1;
    msg->magic2 = COMM_PROTOCOL_PK_STX2;

    msg->len = len;
    comm_protocol_checksum_start(msg, len);

    msg->msgid = id;
    comm_protocol_checksum_update(msg, msg->msgid);
    
    if (len > 0){
        memcpy(msg->payload, buf, len);
        for (uint8_t i = 0; i < len; i++){
            comm_protocol_checksum_update(msg, ptr[i]);
        }
    }
    
    msg->payload[len]= msg->calc_checksum;

    return COMM_MSG_LEN_EXCEPT_PAYLOAD + msg->len;
}

