

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

    osThreadDef(radioTask, radio_device_task, osPriorityNormal, 0, 2048);
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
void radio_device_task(void const *argument)
{
    bool pairing_ret;
    uint8_t rx_num;
    uint8_t rx_num_last = 0xff;
    uint8_t radio_data_rx_addr[GZP_SYSTEM_ADDRESS_WIDTH];
    argument = argument;
	uint8_t temp;
	uint8_t addr[5];

    gzll_init();
    gzp_init();
#if 0
	hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, 32);
	hal_nrf_set_rx_payload_width(HAL_NRF_PIPE1, 32);
	hal_nrf_set_rx_payload_width(HAL_NRF_PIPE2, 32);
	hal_nrf_set_rx_payload_width(HAL_NRF_PIPE3, 32);
	hal_nrf_set_rx_payload_width(HAL_NRF_PIPE4, 32);
	hal_nrf_set_rx_payload_width(HAL_NRF_PIPE5, 32);
#endif
    for(;;)
    {
        /* waitting for data update notify */
        (void)osSemaphoreWait(radio_sema, osWaitForever);
#if 0
		temp = hal_nrf_read_reg(CONFIG);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", CONFIG, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(EN_AA);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", EN_AA, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(EN_RXADDR);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", EN_RXADDR, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(SETUP_AW);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", SETUP_AW, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(SETUP_RETR);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", SETUP_RETR, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RF_CH);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RF_CH, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RF_SETUP);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RF_SETUP, temp, __FILE__, __func__, __LINE__);
	
		temp = hal_nrf_read_reg(STATUS);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", STATUS, temp, __FILE__, __func__, __LINE__);
		
		temp = hal_nrf_read_reg(OBSERVE_TX);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", OBSERVE_TX, temp, __FILE__, __func__, __LINE__);
	
		temp = hal_nrf_get_address(HAL_NRF_PIPE0, addr);
		printf("the addr of p0 is:");
		for(uint8_t i=0; i<5; i++)
		{
			printf("%#x ", addr[i]);
		}
		printf("\r\n");

		hal_nrf_get_address(HAL_NRF_PIPE1, addr);
		printf("the addr of p1 is:");
		for(uint8_t i=0; i<5; i++)
		{
			printf("%#x ", addr[i]);
		}
		printf("\r\n");
		
		temp = hal_nrf_read_reg(RX_ADDR_P2);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_ADDR_P2, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_ADDR_P3);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_ADDR_P3, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_ADDR_P4);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_ADDR_P4, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_ADDR_P5);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_ADDR_P5, temp, __FILE__, __func__, __LINE__);

		hal_nrf_get_address(HAL_NRF_TX, addr);
		printf("the addr of tx is:");
		for(uint8_t i=0; i<5; i++)
		{
			printf("%#x ", addr[i]);
		}
		printf("\r\n");

		temp = hal_nrf_read_reg(RX_PW_P0);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_PW_P0, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_PW_P1);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_PW_P1, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_PW_P2);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_PW_P2, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_PW_P3);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_PW_P3, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_PW_P4);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_PW_P4, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_PW_P5);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_PW_P5, temp, __FILE__, __func__, __LINE__);
		
		temp = hal_nrf_read_reg(FIFO_STATUS);
    	printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", FIFO_STATUS, temp, __FILE__, __func__, __LINE__);
#endif		
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
            	extern uint32_t dbg_tx_retrans,dbg_int_max_rt,dbg_int_tx_ds,dbg_int_rx_dr;
                printf("pairing failed, retrying ...\r\n");
#if 0				
                printf("dbg_tx_retrans=%d @ %s, %s, %d\r\n", dbg_tx_retrans, __FILE__, __func__, __LINE__);
                printf("dbg_int_max_rt=%d \r\n", dbg_int_max_rt);
                printf("dbg_int_tx_ds=%d\r\n", dbg_int_tx_ds);  
				printf("dbg_int_rx_dr=%d\r\n", dbg_int_rx_dr); 
				temp = hal_nrf_read_reg(EN_AA);
				printf("read reg:%#x, val:%#x\r\n", EN_AA, temp);
				temp = hal_nrf_read_reg(DYNPD);
				printf("read reg:%#x, val:%#x\r\n", DYNPD, temp);
				temp = hal_nrf_read_reg(FEATURE);
				printf("read reg:%#x, val:%#x\r\n", FEATURE, temp);

				temp = hal_nrf_get_address(HAL_NRF_PIPE0, addr);
				printf("the addr of p0 is:");
				for(uint8_t i=0; i<5; i++)
				{
					printf("%#x ", addr[i]);
				}
				printf("\r\n");

				hal_nrf_get_address(HAL_NRF_TX, addr);
				printf("the addr of tx is:");
				for(uint8_t i=0; i<5; i++)
				{
					printf("%#x ", addr[i]);
				}
				printf("\r\n");	

				temp = hal_nrf_read_reg(SETUP_RETR);
				printf("read reg:%#x, val:%#x\r\n", SETUP_RETR, temp);
#endif				
            }
        }
        else
        {       
            if (gzll_get_state() == GZLL_IDLE)
            {
                //pcm_ppm_channel_get(pload, RF_PAYLOAD_LENGTH);
                telemetry_transmitter_channel_get(pload, RF_PAYLOAD_LENGTH);
				
				printf("channel pload:\r\n");
				for(uint8_t i=0; i<32; i++)
				{
					printf("%#x\n", pload[i]);
				}
                printf("\r\n");
				
                if (gzll_tx_data(pload, GZLL_MAX_FW_PAYLOAD_LENGTH, 2))
                {

                    if (gzll_rx_fifo_read(ack_pload, NULL, NULL))
                    {
                        print_test(ack_pload);
                        telemetry_radio_ack_send(ack_pload, GZLL_MAX_ACK_PAYLOAD_LENGTH);
                    }
					//debug
					else
					{
						temp = hal_nrf_get_address(HAL_NRF_PIPE0, addr);
						printf("the addr of p0 is:");
						for(uint8_t i=0; i<5; i++)
						{
							printf("%#x ", addr[i]);
						}
						printf("\r\n");

						hal_nrf_get_address(HAL_NRF_TX, addr);
						printf("the addr of tx is:");
						for(uint8_t i=0; i<5; i++)
						{
							printf("%#x ", addr[i]);
						}
						printf("\r\n");	
					}
                }
                else
                {
                    printf("call gzll_tx_data failed!\r\n");
                }			
            }
				
        }
    }
}
#endif

#ifndef GZLL_DEVICE_ONLY
int32_t radio_host_init(void)
{
    osThreadDef(radioTask, radio_host_task, osPriorityNormal, 0, 2048);
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

void radio_host_task(void const * argument)
{
   argument = argument;
   uint8_t addr[5];
   uint8_t temp;

    gzll_init();
    gzp_init();
    gzp_pairing_enable(true);
    gzll_set_param(GZLL_PARAM_RX_PIPES, gzll_get_param(GZLL_PARAM_RX_PIPES) | (1 << 2));
	
    gzll_rx_start();
	printf("gzll_rx_start @ %s, %s, %d\r\n",  __FILE__, __func__, __LINE__);
	
    for (;;)
    {
#if 0    
    	temp = hal_nrf_read_reg(CONFIG);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", CONFIG, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(EN_AA);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", EN_AA, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(EN_RXADDR);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", EN_RXADDR, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(SETUP_AW);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", SETUP_AW, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(SETUP_RETR);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", SETUP_RETR, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RF_CH);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RF_CH, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RF_SETUP);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RF_SETUP, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(STATUS);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", STATUS, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(OBSERVE_TX);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", OBSERVE_TX, temp, __FILE__, __func__, __LINE__);
		
		temp = hal_nrf_get_address(HAL_NRF_PIPE0, addr);
		printf("the addr of p0 is:");
		for(uint8_t i=0; i<5; i++)
		{
			printf("%#x ", addr[i]);
		}
		printf("\r\n");

		hal_nrf_get_address(HAL_NRF_PIPE1, addr);
		printf("the addr of p1 is:");
		for(uint8_t i=0; i<5; i++)
		{
			printf("%#x ", addr[i]);
		}
		printf("\r\n");
		
		temp = hal_nrf_read_reg(RX_ADDR_P2);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_ADDR_P2, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_ADDR_P3);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_ADDR_P3, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_ADDR_P4);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_ADDR_P4, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_ADDR_P5);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_ADDR_P5, temp, __FILE__, __func__, __LINE__);

		hal_nrf_get_address(HAL_NRF_TX, addr);
		printf("the addr of tx is:");
		for(uint8_t i=0; i<5; i++)
		{
			printf("%#x ", addr[i]);
		}
		printf("\r\n");

		temp = hal_nrf_read_reg(RX_PW_P0);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_PW_P0, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_PW_P1);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_PW_P1, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_PW_P2);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_PW_P2, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_PW_P3);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_PW_P3, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_PW_P4);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_PW_P4, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(RX_PW_P5);
		printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", RX_PW_P5, temp, __FILE__, __func__, __LINE__);
		temp = hal_nrf_read_reg(FIFO_STATUS);
    	printf("read reg:%#x, val:%#x @ %s, %s, %d\r\n", FIFO_STATUS, temp, __FILE__, __func__, __LINE__);
#endif		
        #if 0
        if (gzll_rx_fifo_read(pload, NULL, NULL))
        {
            printf("pload:%d\r\n", *(uint16_t*)pload);

            ack_pload[0] = 0x55;
            ack_pload[1] = 0x5a;
            ack_pload[2] = 0x55;
            ack_pload[3] = 0x5a;
            ack_pload[4] = 0x55;

            gzll_ack_payload_write(ack_pload, 5, 0);
        }
        #endif

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
				printf("gzp_address_exchanged return true @ %s,%s,%d\r\n", __FILE__, __func__, __LINE__);
				
				extern uint32_t dbg_tx_retrans,dbg_int_max_rt,dbg_int_tx_ds,dbg_int_rx_dr;
                printf("dbg_tx_retrans=%d \r\n", dbg_tx_retrans);
                printf("dbg_int_max_rt=%d \r\n", dbg_int_max_rt);
                printf("dbg_int_tx_ds=%d\r\n", dbg_int_tx_ds);  
				printf("dbg_int_rx_dr=%d\r\n", dbg_int_rx_dr); 
				temp = hal_nrf_read_reg(EN_AA);
				printf("read reg:%#x, val:%#x\r\n", EN_AA, temp);
				temp = hal_nrf_read_reg(DYNPD);
				printf("read reg:%#x, val:%#x\r\n", DYNPD, temp);
				temp = hal_nrf_read_reg(FEATURE);
				printf("read reg:%#x, val:%#x\r\n", FEATURE, temp);
				
				temp = hal_nrf_get_address(HAL_NRF_PIPE0, addr);
				printf("the addr of p0 is:");
				for(uint8_t i=0; i<5; i++)
				{
					printf("%#x ", addr[i]);
				}
				printf("\r\n");

				hal_nrf_get_address(HAL_NRF_TX, addr);
				printf("the addr of tx is:");
				for(uint8_t i=0; i<5; i++)
				{
					printf("%#x ", addr[i]);
				}
				printf("\r\n");				

				temp = hal_nrf_read_reg(SETUP_RETR);
				printf("read reg:%#x, val:%#x\r\n", SETUP_RETR, temp);
            }
        }

        if (gzll_get_rx_data_ready_pipe_number() == 2)
        {
            // pload used to generate PPM
            // 2 Bytes per unit, and changes slowly, so no need to be protect
            printf("gzll_get_rx_data_ready_pipe_number 2 @ %s,%s,%d\r\n", __FILE__, __func__, __LINE__);
            if (gzll_rx_fifo_read(pload, NULL, NULL))
            {
                printf("pload:%d\r\n", *(uint16_t*)pload);
                memset(ack_pload, 0, RF_PAYLOAD_LENGTH);

			    telemetry_data_encode(ack_pload);

                // test
                print_test(ack_pload);

                gzll_ack_payload_write(ack_pload, GZLL_MAX_ACK_PAYLOAD_LENGTH, 2);
            }
        }
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
    printf("roll %f°, pitch %f°, alt %fm, heading %d°\r\n",
        ToDeg(data.attitude.roll), ToDeg(data.attitude.pitch),
        data.vfr_hud.alt);
    printf("battery volt %d current %d remaining %d",
        data.sys_status.voltage_battery,
        data.sys_status.current_battery,
        data.sys_status.battery_remaining);
}
