
#ifndef __COMM_PROTOCOL_H__
#define __COMM_PROTOCOL_H__

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
//     #3 radio ack data len = n, payload = n Bytes
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
#define COMM_MSG_LEN_EXCEPT_PAYLOAD \
    (sizeof(comm_message_t) - COMM_PAYLOAD_LENGTH - COMM_CHECK_SUM_LENGTH)

#define COMM_MSG_SET_PROTOCOL  0
#define COMM_MSG_DO_PAIR       1
#define COMM_MSG_SEND_CHANNELS 2
#define COMM_MSG_RECEIVED_ACK  3

typedef struct {
    uint8_t calc_checksum;                                        // caculated xor check 

    uint8_t magic1;                                               // STX1
    uint8_t magic2;                                               // STX2
    uint8_t len;                                                  // length of payload
    uint8_t msgid;                                                // ID of message in payload
    uint8_t payload[COMM_PAYLOAD_LENGTH + COMM_CHECK_SUM_LENGTH]; // easy when send
} comm_message_t;

typedef struct {
    uint8_t msg_received; 
    comm_parse_state_t parse_state; 
    uint8_t packet_idx;             // index in current payload
}comm_status_t;


#define _COMM_MSG_ADDR(msg) (&(msg)->magic1)

void comm_protocol_parse(uint8_t c, void (*pfParsedHook)(comm_message_t*));
int8_t comm_protocol_msg_pack(uint8_t id, void*buf, uint8_t len, comm_message_t* msg);

#endif

