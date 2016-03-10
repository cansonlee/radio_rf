/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic
 * Semiconductor ASA.Terms and conditions of usage are described in detail
 * in NORDIC SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision: 2500 $
 */

/** @file
 * @brief
 * Implementation of Gazell Pairing Library (gzp), Device functions.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "gzll.h"
#include "gzp.h"
#include "pairing_list.h"
//#include "hal_aes.h"
//#include "hal_delay.h"
//#include "hal_flash.h"
//#include "memdefs.h"

//-----------------------------------------------------------------------------
// Misc. defines
//-----------------------------------------------------------------------------
#define GZP_PARAMS_DB_ELEMENT_SYSTEM_ADDRESS 0
#define GZP_PARAMS_DB_ELEMENT_HOST_ID (GZP_PARAMS_DB_ELEMENT_SYSTEM_ADDRESS + GZP_SYSTEM_ADDRESS_WIDTH)
#define GZP_PARAMS_DB_ELEMENT_SIZE ((GZP_SYSTEM_ADDRESS_WIDTH + GZP_HOST_ID_LENGTH)*2)
#define GZP_PARAMS_DB_MAX_ENTRIES 14            // [#elements]. Max 14.

//-----------------------------------------------------------------------------
// Derived parameters
//-----------------------------------------------------------------------------
//lint -esym(40, GZP_PARAMS_STORAGE_ADR) "Undeclare identifier"
#define GZP_PARAMS_DB_ADR GZP_PARAMS_STORAGE_ADR
#define GZP_PARAMS_DB_SIZE (GZP_PARAMS_DB_MAX_ENTRIES * GZP_PARAMS_DB_ELEMENT_SIZE)

#define GZP_INDEX_DB_ADR (GZP_PARAMS_STORAGE_ADR + GZP_PARAMS_DB_SIZE)
#define GZP_INDEX_DB_SIZE ((GZP_DEVICE_PARAMS_STORAGE_SIZE - GZP_PARAMS_DB_SIZE)/2)

#if(GZP_DEVICE_PARAMS_STORAGE_SIZE < GZP_PARAMS_DB_SIZE)
  #error GZP_DEVICE_PARAMS_STORAGE_SIZE must be greater or equal to GZP_PAIRING_PARAMS_DB_SIZE
#elif(GZP_DEVICE_PARAMS_STORAGE_SIZE == GZP_PARAMS_DB_SIZE )
  #warning GZP_DEVICE_PARAMS_STORAGE_SIZE to low to be able store any pairing parameters NV memory
#endif

//-----------------------------------------------------------------------------
// Typedefs
//-----------------------------------------------------------------------------

/**
  Definition of the possible return values for the function gzp_tx_rx_transaction()
*/
typedef enum
{
  GZP_TX_RX_SUCCESS,
  GZP_TX_RX_FAILED_TO_SEND,
  GZP_TX_RX_NO_RESPONSE
} gzp_tx_rx_trans_result_t;

//-----------------------------------------------------------------------------
// External variables
//-----------------------------------------------------------------------------

uint8_t gzp_system_address[GZP_SYSTEM_ADDRESS_WIDTH];
static uint8_t gzp_host_id[GZP_HOST_ID_LENGTH];

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

/**
  Internal state variable.
*/
static bool gzp_id_req_pending;

//-----------------------------------------------------------------------------
// Internal (static) function prototypes
//-----------------------------------------------------------------------------

/**
Function for sending an encrypted packet.

The function waits for the transmission to complete.

@param tx_packet is a pointer to the packet to be sent.
@param length is the length of the packet to be sent.
@param is the pipe on which the packet should be sent.

@return
@retval true if the transmission succeeded.
@retval false if the transmission failed (timed out).
*/
static bool gzp_tx_packet(const uint8_t* tx_packet, uint8_t length, uint8_t pipe);

/**
Function sending the packet *tx_packet and a subsequent packet fetching the response
to *tx_packet.

@param tx_packet is a pointer to the packet to be sent.
@param tx_length is the length of the packet to be sent.
@param rx_dst is a pointer to where the received response packet should be stored.
@param rx_length is a pointer to where the length of the received packet should be stored.
@param pipe is the pipe on which the packet should be sent.
*/
static gzp_tx_rx_trans_result_t gzp_tx_rx_transaction(const uint8_t *tx_packet, uint8_t tx_length, uint8_t *rx_dst, uint8_t *rx_length, uint8_t pipe);

/**
  Function for sending an encrypted packet. The function detects whether the correct
  key was used, and attempts to send a "key update" to the host if the wrong key was being
  used.

  @param tx_packet is a pointer to the packet to be sent.
  @param length is the length of the packet to be sent.

  @retval true if transmission succeeded and packet was decrypted correctly by host.
  @retval false if transmission failed or packet was not decrypted correctly by host.
*/
static bool gzp_crypt_tx_transaction(const uint8_t *tx_packet, uint8_t length);

/**
  Function updateing the "dynamic key" and sending a "key update" to the host.

  @retval true if key update succeeded.
  @retval false if if key update failed.
*/
static bool gzp_key_update(void);

/**
  Function for adding an element to "parameters data base" in non volatile (NV) memory. An element is
  GZP_PARAMS_ELEMENT_SYSTEM_ADDRESS bytes long, holding the "system address" and  "host ID".

  The "parameters data base" can store up to GZP_DEVICE_PAIRING_PARAMS_DB_MAX_ENTRIES
  elements.

  @param src_element is a pointer to the element.
  @param index is a number between 0 and (GZP_PARAMS_DB_MAX_ENTRIES - 1)
  selecting the location in which the element will be stored.
*/
static void gzp_params_db_add(const uint8_t *src_element, uint8_t index);

/**
  Function for reading an element from "parameters data base" in non volatile (NV) memory. An element is
  GZP_PARAMS_ELEMENT_SYSTEM_ADDRESS bytes long, holding the "system address" and  "host ID".

  @param dst_element is a pointer where the read element should be stored.
  @param index is a number between 0 and (GZP_PARAMS_DB_MAX_ENTRIES - 1).
  selecting the location that should be read.
*/
static void gzp_params_db_read(uint8_t* dst_element, uint8_t index);

/**
  Function for writing an index to the "index data base" in non volatile (NV) memory.

  @param index is the index to be written to the data base.
*/
static void gzp_index_db_add(uint8_t index);

/**
  Function for reading the index previously written to the "index data base" in NV memory.
*/
static uint8_t gzp_index_db_read(void);

/**
  Function returning @b true if the "index data base" is full.
*/
static bool gzp_index_db_full(void);

/**
  Function returning @b true if the "index data base" is empty.
*/
static bool gzp_index_db_empty(void);

/**
  Function returning @b true if array contains only 1s (0xff).

  @param *src is a pointer to the array to be evaluated.
  @param length is the length of the array to be evaluated.
*/
static bool gzp_array_is_set(const uint8_t* src, uint8_t length);

/**
Function for storing the current "system address" and "host ID" in NV memory.

@param store_all selects whether only "system address" or both "system address" and
"host ID" should be stored.
@arg true selects that both should be stored.
@arg false selects that only "system address" should be stored.
*/
static bool gzp_params_store(bool store_all);

/**
Function for restoring the "system address" and "host ID" from NV memory.
*/
static bool gzp_params_restore(void);

/**
Delay function. Will add a delay equal to GZLL_RX_PERIOD * rx_periods [us].
*/
void gzp_delay_rx_periods(uint16_t rx_periods);

//-----------------------------------------------------------------------------
// Implementation: Application programming interface (API) functions
//-----------------------------------------------------------------------------
void gzp_init()
{
  gzp_id_req_pending = false;

#ifndef GZP_NV_STORAGE_DISABLE
  gzp_params_restore();

  gzp_system_address[0] = *(uint8_t *)0x0800F800;
  gzp_system_address[1] = *(uint8_t *)0x0800F802;
  gzp_system_address[2] = *(uint8_t *)0x0800F804;
  gzp_system_address[3] = *(uint8_t *)0x0800F806;

  if(gzp_system_address[0] = 0xff)
  {
  	printf("system addr read from flash fail!\r\n");
	gzp_system_address[0] = 0x57;
	gzp_system_address[1] = 0xff;
	gzp_system_address[2] = 0x73;
	gzp_system_address[3] = 0x06;
  }
#endif

  // Update radio parameters from gzp_system_address
  gzp_update_radio_params(gzp_system_address);
}

bool gzp_address_req_send(uint8_t idx)
{
  uint8_t i;
  bool retval = false;
  uint8_t address_req[GZP_CMD_HOST_ADDRESS_REQ_PAYLOAD_LENGTH];
  uint8_t rx_payload[GZLL_MAX_PAYLOAD_LENGTH];
  uint16_t temp_power, temp_tx_timeout, temp_device_mode;

  if(gzll_get_state() == GZLL_IDLE)
  {
    // Store parameters that are temporarily changed
    temp_tx_timeout = gzll_get_param(GZLL_PARAM_TX_TIMEOUT);
    temp_power = gzll_get_param(GZLL_PARAM_OUTPUT_POWER);
    temp_device_mode = gzll_get_param(GZLL_PARAM_DEVICE_MODE);
    
    // Modify parameters
    gzll_set_param(GZLL_PARAM_TX_TIMEOUT, GZP_REQ_TX_TIMEOUT);
    gzll_set_param(GZLL_PARAM_OUTPUT_POWER, GZP_POWER);
    gzll_set_param(GZLL_PARAM_DEVICE_MODE, 0);

    // Flush RX FIFO
    gzll_rx_fifo_flush();

    // Build "request" packet
    address_req[0] = GZP_CMD_HOST_ADDRESS_REQ;

    // Send a number of packets in order to broadcast that devices not within
    // close proximity must back off.
    for(i = 0; i < GZP_MAX_BACKOFF_PACKETS; i++)
    {
      if(!gzp_tx_packet(address_req, GZP_CMD_HOST_ADDRESS_REQ_PAYLOAD_LENGTH, 0))
      {
      	//printf("send packet fail @ %s, %s, %d\r\n", __FILE__, __func__, __LINE__);
        break;
      }
    }

    gzp_delay_rx_periods(GZP_TX_ACK_WAIT_TIMEOUT);

    // Send message for fetching pairing response from host.
    address_req[0] = GZP_CMD_HOST_ADDRESS_FETCH;
	//printf("before fetch host addr @ %s, %s, %d\r\n", __FILE__, __func__, __LINE__);
    if(gzp_tx_packet(&address_req[0], GZP_CMD_HOST_ADDRESS_REQ_PAYLOAD_LENGTH, 0))
    {
      //printf("fetch host addr package send successful @ %s, %s, %d\r\n", __FILE__, __func__, __LINE__);
      // If pairing response received
      if(gzll_rx_fifo_read(rx_payload, NULL, NULL))
      {
      	printf("fetch host addr, read rx_payload[0]=%d @ %s, %s, %d\r\n", rx_payload[0], __FILE__, __func__, __LINE__);
        if(rx_payload[0] == GZP_CMD_HOST_ADDRESS_RESP)
        {
          memcpy(gzp_system_address, &rx_payload[GZP_CMD_HOST_ADDRESS_RESP_ADDRESS], GZP_SYSTEM_ADDRESS_WIDTH);
          gzp_update_radio_params(&rx_payload[GZP_CMD_HOST_ADDRESS_RESP_ADDRESS]);

          //pairing_list_addr_write(idx, gzp_system_address);
          #ifndef GZP_NV_STORAGE_DISABLE
          gzp_params_store(false); // "False" indicates that only "system address" part of DB element will be stored

		  uint32_t page_error;
		  FLASH_EraseInitTypeDef flash_page;
		  flash_page.TypeErase = FLASH_TYPEERASE_PAGES;
		  flash_page.PageAddress = 0x0800F800;
		  flash_page.NbPages = 1;
		  HAL_FLASHEx_Erase(&flash_page, &page_error);
		  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x0800F800, 0x00ff & gzp_system_address[0]);
		  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x0800F802, 0x00ff & gzp_system_address[1]);
		  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x0800F804, 0x00ff & gzp_system_address[2]);
		  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x0800F806, 0x00ff & gzp_system_address[3]);
          #endif
          retval = true;
		  printf("addr resp success @ %s, %s, %d\r\n", __FILE__, __func__, __LINE__);
        }
      }
    }
    else
    {
      gzp_delay_rx_periods(GZP_NOT_PROXIMITY_BACKOFF_RX_TIMEOUT - GZP_TX_ACK_WAIT_TIMEOUT);
	  //printf("fetch host addr package send fail @ %s, %s, %d\r\n", __FILE__, __func__, __LINE__);
    }

    gzp_delay_rx_periods(GZP_STEP1_RX_TIMEOUT);

    // Clean-up and restore parameters temporarily  modified
    gzll_rx_fifo_flush();
    gzll_tx_fifo_flush();
    gzll_set_param(GZLL_PARAM_TX_TIMEOUT, temp_tx_timeout);
    gzll_set_param(GZLL_PARAM_OUTPUT_POWER, temp_power);
    gzll_set_param(GZLL_PARAM_DEVICE_MODE, temp_device_mode);
  }

  return retval;
}

void gzp_update_system_address(uint8_t *sys_addr)
{
    memcpy(gzp_system_address, sys_addr, GZP_SYSTEM_ADDRESS_WIDTH);
    gzp_update_radio_params(sys_addr);
    return;
}

#ifndef GZP_CRYPT_DISABLE

gzp_id_req_res_t gzp_id_req_send()
{
  uint8_t tx_packet[GZP_CMD_HOST_ID_REQ_PAYLOAD_LENGTH];
  uint8_t rx_packet[GZLL_MAX_ACK_PAYLOAD_LENGTH];

  // If no ID request is pendning, send new "ID request"
  if(!gzp_id_req_pending)
  {
    // Build "Host ID request packet"
    tx_packet[0] = GZP_CMD_HOST_ID_REQ;

    // Generate new session token
    gzp_random_numbers_generate(&tx_packet[GZP_CMD_HOST_ID_REQ_SESSION_TOKEN], GZP_SESSION_TOKEN_LENGTH);

    // Send "Host ID request"
    if(gzp_tx_packet(tx_packet, GZP_CMD_HOST_ID_REQ_PAYLOAD_LENGTH, GZP_DATA_PIPE))
    {
      // Update session token if "Host ID request" was successfully transmitted
      gzp_crypt_set_session_token(&tx_packet[GZP_CMD_HOST_ID_REQ_SESSION_TOKEN]);
      gzp_id_req_pending = true;

      return GZP_ID_RESP_PENDING;
    }
  }
  else // If "ID request is pending" send "fetch ID" packet
  {
    // Build "host ID fetch" packet
    tx_packet[0] = GZP_CMD_HOST_ID_FETCH;
    gzp_add_validation_id(&tx_packet[GZP_CMD_HOST_ID_FETCH_VALIDATION_ID]);

    // Encrypt "host ID fetch" packet
    gzp_crypt_select_key(GZP_ID_EXCHANGE);
    gzp_crypt(&tx_packet[1], &tx_packet[1], GZP_CMD_HOST_ID_FETCH_PAYLOAD_LENGTH - 1);

    // If packet was successfully sent AND a response packet was received
    if(gzp_tx_rx_transaction(tx_packet, GZP_CMD_HOST_ID_FETCH_PAYLOAD_LENGTH, rx_packet, NULL, GZP_DATA_PIPE) == GZP_TX_RX_SUCCESS)
    {
      // Validate response packet
      if(rx_packet[0] == GZP_CMD_HOST_ID_FETCH_RESP)
      {
        gzp_crypt(&rx_packet[1], &rx_packet[1], GZP_CMD_HOST_ID_FETCH_RESP_PAYLOAD_LENGTH - 1);
        if(gzp_validate_id(&rx_packet[GZP_CMD_HOST_ID_FETCH_RESP_VALIDATION_ID]))
        {
          switch(rx_packet[GZP_CMD_HOST_ID_FETCH_RESP_STATUS])
          {
            case GZP_ID_RESP_PENDING:
              break;
            case GZP_ID_RESP_REJECTED:
              gzp_id_req_pending = false;
              break;
            case GZP_ID_RESP_GRANTED:
              gzp_set_host_id(&rx_packet[GZP_CMD_HOST_ID_FETCH_RESP_HOST_ID]);
              #ifndef GZP_NV_STORAGE_DISABLE
              gzp_params_store(true);
              #endif
              gzp_id_req_pending = false;
              break;
            default:
              break;
          }

          return (gzp_id_req_res_t)rx_packet[GZP_CMD_HOST_ID_FETCH_RESP_STATUS];
        }
        else
        {
          gzp_id_req_pending = false;
          return GZP_ID_RESP_REJECTED;
        }
      }
    }
  }

  gzp_id_req_pending = false;
  return GZP_ID_RESP_FAILED;
}

void gzp_id_req_cancel()
{
  gzp_id_req_pending = false;
}

bool gzp_crypt_data_send(const uint8_t *src, uint8_t length)
{
  if(length <= GZP_ENCRYPTED_USER_DATA_MAX_LENGTH)
  {
    if(gzp_crypt_tx_transaction(src, length))
    {
      return true;
    }
    else
    {
      // Attempt key update if user data transmission failed
      // during normal operation (!gzp_id_req_pending)
      if(!gzp_id_req_pending)
      {
        gzp_key_update();
        return gzp_crypt_tx_transaction(src, length);
      }
      return false;
    }
  }
  else
  {
    return false;
  }
}

#endif

//-----------------------------------------------------------------------------
// Implementation: Internal (static) functions
//-----------------------------------------------------------------------------
extern uint32_t dbg_tim3_int_cnt;
extern uint32_t dbg_exit1_int_cnt;
static bool gzp_tx_packet(const uint8_t* tx_packet, uint8_t length, uint8_t pipe)
{
  if(gzll_tx_data(tx_packet, length, pipe))
  {
    while(gzll_get_state() != GZLL_IDLE)
    ;
    return gzll_tx_success();
  }
  return false;
}

static gzp_tx_rx_trans_result_t gzp_tx_rx_transaction(const uint8_t *tx_packet, uint8_t tx_length, uint8_t *rx_dst, uint8_t *rx_length, uint8_t pipe)
{
  gzp_tx_rx_trans_result_t retval;
  uint8_t fetch_packet[GZPAR_CMD_FETCH_RESP_PAYLOAD_LENGTH];

  gzll_rx_fifo_flush();

  retval = GZP_TX_RX_FAILED_TO_SEND;

  if(gzp_tx_packet(tx_packet, tx_length, pipe))
  {
    retval = GZP_TX_RX_NO_RESPONSE;

    gzll_rx_fifo_flush();
    fetch_packet[0] = GZP_CMD_FETCH_RESP;

    gzp_delay_rx_periods(GZP_TX_RX_TRANS_DELAY);
    gzp_tx_packet(fetch_packet, GZPAR_CMD_FETCH_RESP_PAYLOAD_LENGTH, pipe);

    if(gzll_rx_fifo_read(rx_dst, rx_length, NULL))
    {
      retval = GZP_TX_RX_SUCCESS;
    }
  }
  return retval;
}

#ifndef GZP_CRYPT_DISABLE

static bool gzp_crypt_tx_transaction(const uint8_t *src, uint8_t length)
{
  uint8_t tx_packet[GZLL_MAX_FW_PAYLOAD_LENGTH];
  uint8_t rx_packet[GZLL_MAX_ACK_PAYLOAD_LENGTH];
  uint8_t tx_packet_length;

  tx_packet_length = length + GZP_USER_DATA_PACKET_OVERHEAD;

  // Assemble tx packet
  tx_packet[0] = GZP_CMD_ENCRYPTED_USER_DATA;
  gzp_add_validation_id(&tx_packet[GZP_CMD_ENCRYPTED_USER_DATA_VALIDATION_ID]);
  memcpy(&tx_packet[GZP_CMD_ENCRYPTED_USER_DATA_PAYLOAD], (uint8_t*)src, length);

  // Encrypt tx packet
  if(gzp_id_req_pending)
  {
    gzp_crypt_select_key(GZP_ID_EXCHANGE);
  }
  else
  {
    gzp_crypt_select_key(GZP_DATA_EXCHANGE);
  }
  gzp_crypt(&tx_packet[1], &tx_packet[1], tx_packet_length - 1);

  // If packet was successfully sent AND a response packet was received
  if(gzp_tx_rx_transaction(tx_packet, tx_packet_length, rx_packet, NULL, GZP_DATA_PIPE) == GZP_TX_RX_SUCCESS)
  {

    if(rx_packet[0] == GZP_CMD_ENCRYPTED_USER_DATA_RESP)
    {
      gzp_crypt(&rx_packet[GZP_CMD_ENCRYPTED_USER_DATA_RESP_VALIDATION_ID], &rx_packet[GZP_CMD_ENCRYPTED_USER_DATA_RESP_VALIDATION_ID], GZP_VALIDATION_ID_LENGTH);

      // Validate response in order to know whether packet was correctly decrypted by host
      if(gzp_validate_id(&rx_packet[GZP_CMD_ENCRYPTED_USER_DATA_RESP_VALIDATION_ID]))
      {
        // Update session token if normal operation (!gzp_id_req_pending)
        if(!gzp_id_req_pending)
        {
          gzp_crypt_set_session_token(&rx_packet[GZP_CMD_ENCRYPTED_USER_DATA_RESP_SESSION_TOKEN]);
        }
        return true;
      }
      else
      {
        return false;
      }
    }
  }

  return false;
}

static bool gzp_key_update(void)
{
  uint8_t tx_packet[GZP_CMD_KEY_UPDATE_PAYLOAD_LENGTH], rx_packet[GZLL_MAX_ACK_PAYLOAD_LENGTH];

  // Send "prepare packet" to get session token to be used for key update
  tx_packet[0] = GZP_CMD_KEY_UPDATE_PREPARE;

  // If packet was successfully sent AND a response packet was received
  if(gzp_tx_rx_transaction(tx_packet, GZP_CMD_KEY_UPDATE_PREPARE_PAYLOAD_LENGTH, rx_packet, NULL, GZP_DATA_PIPE) == GZP_TX_RX_SUCCESS)
  {
    if(rx_packet[0] == GZP_CMD_KEY_UPDATE_PREPARE_RESP)
    {
      gzp_crypt_set_session_token(&rx_packet[GZP_CMD_KEY_UPDATE_PREPARE_RESP_SESSION_TOKEN]);

      // Build "key update" packet
      tx_packet[0] = GZP_CMD_KEY_UPDATE;
      gzp_add_validation_id(&tx_packet[GZP_CMD_KEY_UPDATE_VALIDATION_ID]);
      gzp_random_numbers_generate(&tx_packet[GZP_CMD_KEY_UPDATE_NEW_KEY], GZP_DYN_KEY_LENGTH);
      gzp_crypt_set_dyn_key(&tx_packet[GZP_CMD_KEY_UPDATE_NEW_KEY]);

      // Encrypt "key update packet"
      gzp_crypt_select_key(GZP_KEY_EXCHANGE);
      gzp_crypt(&tx_packet[1], &tx_packet[1], GZP_CMD_KEY_UPDATE_PAYLOAD_LENGTH - 1);

      // Send "key update" packet
      if(gzp_tx_packet(tx_packet, GZP_CMD_KEY_UPDATE_PAYLOAD_LENGTH, GZP_DATA_PIPE))
      {
        return true;
      }
    }
  }

  return false;
}

#endif

bool gzp_set_host_id(const uint8_t * id)
{
  memcpy(gzp_host_id, id, GZP_HOST_ID_LENGTH);
  return true;
}

bool gzp_get_host_id(uint8_t * dst_id)
{
  memcpy(dst_id, gzp_host_id, GZP_HOST_ID_LENGTH);
  return true;
}

static void gzp_params_db_add(const uint8_t* src_element, uint8_t index)
{
  hal_flash_bytes_write(GZP_PARAMS_DB_ADR + (index * GZP_PARAMS_DB_ELEMENT_SIZE), src_element, GZP_PARAMS_DB_ELEMENT_SIZE);
}

static void gzp_params_db_read(uint8_t* dst_element, uint8_t index)
{
  hal_flash_bytes_read(GZP_PARAMS_DB_ADR + (index * GZP_PARAMS_DB_ELEMENT_SIZE), dst_element, GZP_PARAMS_DB_ELEMENT_SIZE);
}

static void gzp_index_db_add(uint8_t val)
{
  int16_t i;
  uint8_t temp_val;

  // Search for unwritten loacation in index DB
  //for(i = 0; i < GZP_INDEX_DB_SIZE; i++)
  for(i = 0; i < GZP_INDEX_DB_SIZE; i+=2)		// 16位地址对齐
  {
    temp_val = hal_flash_byte_read(GZP_INDEX_DB_ADR + i);

    // Lower nibble
    //if(i != (GZP_INDEX_DB_SIZE - 1))
    if(i != (GZP_INDEX_DB_SIZE - 2))			// 16位地址对齐
    {
      if((temp_val & 0x0f) == 0x0f)
      {
        temp_val = (temp_val & 0xf0) | val;
        break;
      }
      // Upper nibble
      else if((temp_val & 0xf0) == 0xf0)
      {
        temp_val = (temp_val & 0x0f) | (val << 4);
        break;
      }
    }
    else
    {
      temp_val = (GZP_PARAMS_DB_MAX_ENTRIES << 4) | val;
      break;
    }
  }

  // Write index DB
  hal_flash_byte_write(GZP_INDEX_DB_ADR + i, temp_val);
}

static uint8_t gzp_index_db_read()
{
  uint8_t retval;
  int16_t i;

  // Search for previously written location
  //for(i = (GZP_INDEX_DB_SIZE - 1); i >= 0; i--)
  for(i = (GZP_INDEX_DB_SIZE - 1); i >= 0; i-=2)		//16位地址对齐
  {
    retval = hal_flash_byte_read(GZP_INDEX_DB_ADR + i);

    if(retval != 0xff)
    {
      break;
    }
  }

  if(retval == 0xff)
  {
    retval = GZP_PARAMS_DB_MAX_ENTRIES;  // index db empty
  }
  else if((retval & 0xf0) != 0xf0)
  {
    retval >>= 4;
  }
  else
  {
    retval &= 0x0f;
  }

  return retval;
}

static bool gzp_index_db_full()
{
  //return ((GZP_INDEX_DB_SIZE == 0) || ((hal_flash_byte_read(GZP_INDEX_DB_ADR + (GZP_INDEX_DB_SIZE - 1)) != 0xff)));
  return ((GZP_INDEX_DB_SIZE == 0) || ((hal_flash_byte_read(GZP_INDEX_DB_ADR + (GZP_INDEX_DB_SIZE - 2)) != 0xff)));	 //16位地址对齐
}

static bool gzp_index_db_empty()
{
  return ((GZP_INDEX_DB_SIZE == 0) || hal_flash_byte_read(GZP_INDEX_DB_ADR) == 0xff);
}

static bool gzp_array_is_set(const uint8_t* src, uint8_t length)
{
  uint8_t i;

  for(i = 0; i < length; i++)
  {
    if(*(src++) != 0xff)
    {
      return false;
    }
  }
  return true;
}

static bool gzp_params_store(bool store_all)
{
  uint8_t i;
  bool write_index_db = false;
  bool write_param_db = false;
  uint8_t new_db_index;
  uint8_t temp_element[GZP_PARAMS_DB_ELEMENT_SIZE];

  // Search param DB to see if current setup exists
  if(store_all)
  {
    // Search for: Current system address and host ID exists
    for(i = 0; i < GZP_PARAMS_DB_MAX_ENTRIES; i++)
    {
      gzp_params_db_read(temp_element, i);

      if(((memcmp(&temp_element[GZP_PARAMS_DB_ELEMENT_SYSTEM_ADDRESS], gzp_system_address, GZP_SYSTEM_ADDRESS_WIDTH)) == 0) && ((memcmp(&temp_element[GZP_PARAMS_DB_ELEMENT_HOST_ID], gzp_host_id, GZP_HOST_ID_LENGTH)) == 0))
      {
        write_index_db = true;
        new_db_index = i;
        break; // System address + host_id allready exists in database
      }
    }

    // Search for: Current system address and cleared host ID
		if(!write_index_db)
    {
      for(i = 0; i < GZP_PARAMS_DB_MAX_ENTRIES; i++)
  	  {
  			gzp_params_db_read(temp_element, i);

        if(((memcmp(&temp_element[GZP_PARAMS_DB_ELEMENT_SYSTEM_ADDRESS], gzp_system_address, GZP_SYSTEM_ADDRESS_WIDTH)) == 0) && \
           (gzp_array_is_set(&temp_element[GZP_PARAMS_DB_ELEMENT_HOST_ID], GZP_HOST_ID_LENGTH)))
        {
  				memcpy(&temp_element[GZP_PARAMS_DB_ELEMENT_HOST_ID], gzp_host_id, GZP_HOST_ID_LENGTH);
  			  new_db_index = i;
          write_index_db = true;
  				write_param_db = true;
  				break;
  			}
      }
		}

    // Search for: Cleared system address and cleared host ID
		if(!write_index_db)
    {
      for(i = 0; i < GZP_PARAMS_DB_MAX_ENTRIES; i++)
  	  {
  			gzp_params_db_read(temp_element, i);

        if(gzp_array_is_set(temp_element, GZP_PARAMS_DB_ELEMENT_SIZE))
        {
  				memcpy(&temp_element[GZP_PARAMS_DB_ELEMENT_SYSTEM_ADDRESS], gzp_system_address, GZP_SYSTEM_ADDRESS_WIDTH);
  				memcpy(&temp_element[GZP_PARAMS_DB_ELEMENT_HOST_ID], gzp_host_id, GZP_HOST_ID_LENGTH);
   			  new_db_index = i;
          write_index_db = true;
  				write_param_db = true;
          break;
  			}
      }
    }
  }
	else
	{
    // Search for: System address + any host ID
    for(i = 0; i < GZP_PARAMS_DB_MAX_ENTRIES; i++)
  	{
  	  gzp_params_db_read(temp_element, i);

  	  if((memcmp(&temp_element[GZP_PARAMS_DB_ELEMENT_SYSTEM_ADDRESS], gzp_system_address, GZP_SYSTEM_ADDRESS_WIDTH)) == 0)
  	  {
  	    //memcpy(&temp_element[GZP_PARAMS_DB_ELEMENT_HOST_ID], gzp_host_id, GZP_HOST_ID_LENGTH);
        write_index_db = true;
  	    new_db_index = i;
        break;
  	  }
    }

    // Search for: System address cleared
    if(!write_index_db)
    {
      for(i = 0; i < GZP_PARAMS_DB_MAX_ENTRIES; i++)
  	  {
  	    gzp_params_db_read(temp_element, i);

        if(gzp_array_is_set(&temp_element[GZP_PARAMS_DB_ELEMENT_SYSTEM_ADDRESS], GZP_SYSTEM_ADDRESS_WIDTH))
    	{
		  memcpy(&temp_element[GZP_PARAMS_DB_ELEMENT_SYSTEM_ADDRESS], gzp_system_address, GZP_SYSTEM_ADDRESS_WIDTH);
    	  write_index_db = true;
    	  write_param_db = true;
    	  new_db_index = i;
          break;
        }
      }
    }
  }

	if(write_param_db)
	{
      gzp_params_db_add(temp_element, new_db_index);
	}

	if(write_index_db)
	{
	  if(!gzp_index_db_full() && (new_db_index != gzp_index_db_read()) && (new_db_index != GZP_PARAMS_DB_MAX_ENTRIES))
	  {
	    gzp_index_db_add(new_db_index);

        return true;
	  }
	}

  return false;
}

static bool gzp_params_restore(void)
{
  uint8_t i;
  uint8_t temp_element[GZP_PARAMS_DB_ELEMENT_SIZE];

  printf("read falsh byte @ %s, %s, %d\r\n", __FILE__, __func__, __LINE__);

  if(!gzp_index_db_full() && !gzp_index_db_empty())
  {
    i = gzp_index_db_read();

    if(i < GZP_PARAMS_DB_MAX_ENTRIES)
    {
      gzp_params_db_read(temp_element, i);
      memcpy(gzp_system_address, &temp_element[GZP_PARAMS_DB_ELEMENT_SYSTEM_ADDRESS], GZP_SYSTEM_ADDRESS_WIDTH);
      gzp_set_host_id(&temp_element[GZP_PARAMS_DB_ELEMENT_HOST_ID]);
      return true;
    }
  }

  return false;
}

void gzp_delay_rx_periods(uint16_t rx_periods)
{
  uint16_t i;
  
  for(i=0; i < rx_periods; i++)
  {
    delay_us(GZLL_DEFAULT_PARAM_RX_PERIOD);
  }
}

