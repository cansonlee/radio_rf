

#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "hal_nrf.h"
#include "radio.h"
#include "pcm_decoder.h"
#include <gzll.h>
#include <gzp.h>
#include "telemetry.h"
#include "pairing_list.h"
#include "led.h"

#ifndef GZLL_HOST_ONLY
#include "common.h"
#endif

extern bool waitting_for_heartbeat;
extern bool mavlink_active;
extern uint32_t timer_1ms;
extern uint32_t last_heart_beat_time;
extern uint8_t gzp_system_address[];

/** The payload sent over the radio. Also contains the recieved data.
 * Should be read with radio_get_pload_byte(). */
static uint8_t pload[RF_PAYLOAD_LENGTH];
static uint8_t ack_pload[RF_PAYLOAD_LENGTH];
/** The current status of the radio. Should be set with radio_set_status(),
 * and read with radio_get_status().
 */

//static uint8_t content[RF_PAYLOAD_LENGTH + 1] = "Hello, world!1234567890qwertyuiop";

static bool g_tx_pairing_enable = ENABLE;

osSemaphoreId radio_sema;
osThreadId radioTaskHandle;

bool radio_pairing_status_get(void);
void print_test(void* buf);
void radio_tx_pairing_enable(bool enable);
bool radio_tx_pairing_usable_get(void);



uint8_t radio_get_pload_byte (uint8_t byte_index)
{
  return pload[byte_index];
}

void radio_get_pload (uint8_t *pload_out)
{
    memcpy(pload_out, pload, RF_PAYLOAD_LENGTH);
}

void radio_active(void)
{
    (void)osSemaphoreReleaseFromISR(radio_sema);
}

void radio_tx_pairing_enable(bool enable)
{
	g_tx_pairing_enable = enable;
}

bool radio_tx_pairing_usable_get(void)
{
	return g_tx_pairing_enable;
}


#ifndef GZLL_HOST_ONLY
int32_t radio_device_init(void)
{
    int32_t ret;
    ret = pairing_list_init();
    if (ret != 0)
    {
        printf("[%s, L%d] call pairing_list_init failed ret %d.\r\n",
            __FILE__, __LINE__, ret);
        return -1;
    }

    osSemaphoreDef(RADIO_SEM);
    radio_sema = osSemaphoreEmptyCreate(osSemaphore(RADIO_SEM));
    if (NULL == radio_sema)
    {
        printf("[%s, L%d] create semaphore failed ret 0x%x.\r\n",
            __FILE__, __LINE__, (unsigned int)radio_sema);
        return -1;
    }

    osThreadDef(radioTask, radio_device_task, osPriorityAboveNormal, 0, 256);
    radioTaskHandle = osThreadCreate(osThread(radioTask), NULL);
    if (NULL == radioTaskHandle)
    {
        printf("[%s, L%d] create thread failed.\r\n",
            __FILE__, __LINE__);
        return -1;
    }

    return 0;
}

bool pairing_ok = false;
extern void gzll_set_system_idle(void);
extern void gzll_set_system_idle_manual(void);

uint32_t test_total_tx=0;
uint32_t test_total_ack=0;
void radio_device_task(void const *argument)
{
    bool pairing_ret;
    uint8_t rx_num;
    uint8_t rx_num_last = 0xff;
    uint8_t radio_data_rx_addr[GZP_SYSTEM_ADDRESS_WIDTH];
	uint8_t addr[5];
	uint8_t pairing_status = 0xff;
	
    argument = argument;

    gzll_init();
    gzp_init();
	printf("gzp init over!\r\n");

	radio_tx_pairing_enable(true);

    for(;;)
    {
        /* waitting for data update notify */
        (void)osSemaphoreWait(radio_sema, osWaitForever);
	
#if 0
        rx_num = telemetry_rxnum_get();
        //printf("rx_num %d\n", rx_num);
        if (rx_num != rx_num_last)
        {
            (void)pairing_list_addr_read(rx_num, radio_data_rx_addr);
            gzp_update_system_address(radio_data_rx_addr);
			printf("the radio_data_rx_addr is:");
			for(uint8_t i=0; i<GZP_SYSTEM_ADDRESS_WIDTH; i++)
			{
				printf("%#x ", radio_data_rx_addr[i]);
			}
			printf("\r\n");
            rx_num_last = rx_num;
        }
#endif
        if (telemetry_transmitter_mode_get() == TRANSMITTING_MODE_BINDING)
        {
        	if(!radio_tx_pairing_usable_get())
    		{
    			continue;
    		}
			
            osDelay(100);
            pairing_ret = gzp_address_req_send(rx_num);
            if (pairing_ret)
            {
                printf("pairing success(%02x%02x%02x%02x)!\r\n",
                    gzp_system_address[0], gzp_system_address[1],
                    gzp_system_address[2], gzp_system_address[3]);
				radio_tx_pairing_enable(false);
            }
            else
            {
                printf("pairing failed, retrying ...\r\n");				
            }
        }
        else
        {       
        	radio_tx_pairing_enable(true);
			
            if (gzll_rx_fifo_read(ack_pload, NULL, NULL))
            {                    	
                //print_test(ack_pload);
                test_total_ack++;
                telemetry_radio_ack_send(ack_pload, GZLL_MAX_ACK_PAYLOAD_LENGTH);
            }

			if((gzll_get_state() != GZLL_IDLE) && ( hal_nrf_tx_fifo_empty()))
			{
				gzll_set_system_idle_manual();
			}
								
            if (gzll_get_state() == GZLL_IDLE)          
            {
                telemetry_transmitter_channel_get(pload, RF_PAYLOAD_LENGTH);

                if (gzll_tx_data(pload, GZLL_MAX_FW_PAYLOAD_LENGTH, 2))
                {		
					test_total_tx++;
                }
                else
                {
                    //printf("call gzll_tx_data failed! %d\r\n",test_fail_tx);
                }		
            }
        }
    }
}
#endif

#ifndef GZLL_DEVICE_ONLY
int32_t radio_host_init(void)
{
    osThreadDef(radioTask, radio_host_task, osPriorityAboveNormal, 0, 256);
    radioTaskHandle = osThreadCreate(osThread(radioTask), NULL);
    if (NULL == radioTaskHandle)
    {
        printf("[%s, L%d] create thread failed.\r\n",
            __FILE__, __LINE__);
        return -1;
    }

    return 0;
}

static bool g_host_pairing_status = false;

bool radio_pairing_status_get(void)
{
    return g_host_pairing_status;
}

void radio_pairing_status_set(bool status)
{
    g_host_pairing_status = status;
    return;
}
uint32_t test_recv_pak_num = 0;
void radio_host_task(void const * argument)
{   
   uint8_t addr[5];
   uint8_t temp;
   argument = argument;

    gzll_init();
    gzp_init();
    gzp_pairing_enable(true);
    gzll_set_param(GZLL_PARAM_RX_PIPES, gzll_get_param(GZLL_PARAM_RX_PIPES) | (1 << 2));
	
    gzll_rx_start();

	hal_nrf_get_address(HAL_NRF_PIPE1, addr);
	printf("HAL_NRF_PIPE1 addr is:");
	for(uint8_t i=0; i<5; i++)
	{
		printf("%#x ", addr[i]);
	}
	printf("\r\n");

	temp=hal_nrf_get_address(HAL_NRF_PIPE2, addr);
	printf("HAL_NRF_PIPE2 addr is: %#x\r\n", addr[0]);
	
    for (;;)
    {		
        if (timer_1ms - last_heart_beat_time > 2000U)
        {
            mavlink_active = false;
            waitting_for_heartbeat = true;
        }

        if (radio_pairing_status_get())
        {        	
            gzp_host_execute();
            if (gzp_address_exchanged())
            {
                led_blink(LED_1, 3, 1000);
                radio_pairing_status_set(false);
            }
        }

        if (gzll_get_rx_data_ready_pipe_number() == 2)
        {
            // pload used to generate PPM
            // 2 Bytes per unit, and changes slowly, so no need to be protect
           // printf("gzll_get_rx_data_ready_pipe_number 2 @ %s,%s,%d\r\n", __FILE__, __func__, __LINE__);
            if (gzll_rx_fifo_read(pload, NULL, NULL))
            {
                //printf("pload:%d\r\n", *(uint16_t*)pload);
                memset(ack_pload, 0, RF_PAYLOAD_LENGTH);

			    telemetry_data_encode(ack_pload);
				test_recv_pak_num++;
                // test
                //print_test(ack_pload);
                //printf("test_recv_pak_num=%d\r\n", test_recv_pak_num);

                gzll_ack_payload_write(ack_pload, GZLL_MAX_ACK_PAYLOAD_LENGTH, 2);
            }
        }
		osDelay(1);
    }
}
#endif

extern void telemetry_data_decode(void* buf, TELEMETRY_DATA* data);
void print_test(void* buf){
    TELEMETRY_DATA data;
    telemetry_data_decode(buf, &data);

    printf("system status armed %d flight mode %d:%d",
        (data.heartbeat.base_mode & 0x80) == 0x80,
        data.heartbeat.type, data.heartbeat.base_mode);
    printf("roll %fdeg, pitch %fdeg, alt %fm, heading %ddegr\n",
        ToDeg(data.attitude.roll), ToDeg(data.attitude.pitch),
        data.vfr_hud.alt);
    printf("battery volt %d current %d remaining %d",
        data.sys_status.voltage_battery,
        data.sys_status.current_battery,
        data.sys_status.battery_remaining);
}
