

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

osSemaphoreId radio_sema;
osThreadId radioTaskHandle;

bool radio_pairing_status_get(void);
void print_test(void* buf);


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
void radio_device_task(void const *argument)
{
    bool pairing_ret;
    uint8_t rx_num;
    uint8_t rx_num_last = 0xff;
    uint8_t radio_data_rx_addr[GZP_SYSTEM_ADDRESS_WIDTH];
    argument = argument;
	uint32_t test_total_tx=0;
	uint32_t test_success_tx=0;
	uint32_t test_fail_tx=0;
	uint32_t test_ack_num=0;
	uint32_t test_last_success_tx=0;
	uint32_t test_out_idle_times=0;
	uint32_t flag;
	uint32_t test_free_stack_space = 0;

    gzll_init();
	printf("gzll init over!\r\n");
    gzp_init();
	printf("gzp init over!\r\n");

    for(;;)
    {
        /* waitting for data update notify */
        //(void)osSemaphoreWait(radio_sema, osWaitForever);
	
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
            osDelay(100);
            pairing_ret = gzp_address_req_send(rx_num);
            if (pairing_ret)
            {
                printf("pairing success(%02x%02x%02x%02x)!\r\n",
                    gzp_system_address[0], gzp_system_address[1],
                    gzp_system_address[2], gzp_system_address[3]);
            }
            else
            {
                printf("pairing failed, retrying ...\r\n");				
            }
        }
        else
        {       
            if (gzll_rx_fifo_read(ack_pload, NULL, NULL))
            {                    	
                //print_test(ack_pload);
                //telemetry_radio_ack_send(ack_pload, GZLL_MAX_ACK_PAYLOAD_LENGTH);
                
				test_ack_num++;
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

                }
                else
                {
                    //printf("call gzll_tx_data failed! %d\r\n",test_fail_tx);
                }		
            }
			//debug
			else
			{
#if 0			
				test_free_stack_space = uxTaskGetStackHighWaterMark(radioTaskHandle);
				extern uint16_t gzll_timeout_counter_get(void);
				extern uint16_t gzll_dyn_params_get(void);
				extern uint32_t timer_1ms;
				printf("tx timeout val=%d, gzll_timeout_counter=%d,timer_1ms=%d, gzll_get_state != GZLL_IDLE @ %s, %s, L%d\r\n", gzll_dyn_params_get(), gzll_timeout_counter_get(), timer_1ms, __FILE__, __func__, __LINE__);
				extern uint32_t dbg_tx_retrans,dbg_int_max_rt,dbg_int_tx_ds,dbg_int_rx_dr,test_max_rt_to_idle,test_tx_ds_to_idle,dbg_exit1_int_cnt,test_exti0_times;
		        printf("dbg_tx_retrans=%d \r\n", dbg_tx_retrans);
		        printf("dbg_int_max_rt=%d \r\n", dbg_int_max_rt);
		        printf("dbg_int_tx_ds=%d\r\n", dbg_int_tx_ds);  
				printf("dbg_int_rx_dr=%d\r\n", dbg_int_rx_dr); 
				printf("test_max_rt_to_idle=%d\r\n", test_max_rt_to_idle); 
				printf("test_tx_ds_to_idle=%d\r\n", test_tx_ds_to_idle); 
				printf("test_total_tx=%d\r\n", test_total_tx);
				printf("test_success_tx=%d\r\n", test_success_tx);
				printf("test_fail_tx=%d\r\n", test_fail_tx);				
				printf("test_ack_num=%d\r\n", test_ack_num);				
				printf("test_free_stack_space=%d\r\n", test_free_stack_space);
				printf("SPI1->SR=%#x\r\n", SPI1->SR);
				printf("rx-e:%d, rx-f:%d, tx-e:%d, tx-f:%d\r\n", hal_nrf_rx_fifo_empty(), hal_nrf_rx_fifo_full(), hal_nrf_tx_fifo_empty(),hal_nrf_tx_fifo_full());
			
				extern uint32_t test_zero_status;
				printf("status=%#x, INT times=%d\r\n", test_zero_status, dbg_exit1_int_cnt);
#endif					
			}
        }
		osDelay(1);
    }
}
#endif

#ifndef GZLL_DEVICE_ONLY
int32_t radio_host_init(void)
{
    osThreadDef(radioTask, radio_host_task, osPriorityAboveNormal, 0, 2048);
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
   argument = argument;

    gzll_init();
    gzp_init();
    gzp_pairing_enable(true);
    gzll_set_param(GZLL_PARAM_RX_PIPES, gzll_get_param(GZLL_PARAM_RX_PIPES) | (1 << 2));
	
    gzll_rx_start();
	
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
