#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include <common/mavlink.h>
#include "telemetry.h"

extern uint32_t timer_1ms;

mavlink_system_t mavlink_system = {20, 3, 0, 0 ,0 ,0};
mavlink_system_t target_system;

UART_HandleTypeDef huart2;
TELEMETRY_DATA telemetry;
osSemaphoreId telemetry_sema;
osThreadId telemetryTaskHandle;
static uint32_t packet_drops;
bool mavlink_active = false;
static bool request_send = false;
bool waitting_for_heartbeat = true;
uint32_t last_heart_beat_time;
uint32_t heartbeat_cnt;

static bool p_isFlying = false;

// MAV_DATA_STREAM_RAW_SENSORS
//   RAW_IMU
//   SCALED_IMU2
//   SCALED_IMU3
//   SCALED_PRESSURE
//   SCALED_PRESSURE2
//   SENSOR_OFFSETS(#150 not common msg)
// MAV_DATA_STREAM_EXTENDED_STATUS
//   SYS_STATUS
//   POWER_STATUS
//   MISSION_CURRENT
//   GPS_RAW_INT
//   GPS_RTK(if rtk avalible)
//   GPS2_RAW
//   GPS2_RTK(if rtk avalible)
//   NAV_CONTROLLER_OUTPUT
//   FENCE_STATUS(#162 not common msg, only send when fence enabled) 
// MAV_DATA_STREAM_RC_CHANNELS
//   RC_CHANNELS_RAW
//   RC_CHANNELS
//   SERVO_OUTPUT_RAW
// MAV_DATA_STREAM_POSITION
//   GLOBAL_POSITION_INT
//   LOCAL_POSITION_NED
// MAV_DATA_STREAM_EXTRA1
//   ATTITUDE
//   AHRS2(#178 not common msg)
//   PID_TUNING
// MAV_DATA_STREAM_EXTRA2
//   VFR_HUD
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
TELEMETRY_STREAM streams[] = 
{
    {MAV_DATA_STREAM_RAW_SENSORS,          2},
    {MAV_DATA_STREAM_EXTENDED_STATUS,      2},
    {MAV_DATA_STREAM_RC_CHANNELS,          5},
    {MAV_DATA_STREAM_POSITION,             2},
    {MAV_DATA_STREAM_EXTRA1,               5},
    {MAV_DATA_STREAM_EXTRA2,               2}
};

uint32_t streams_max = sizeof(streams) / sizeof(streams[0]);

void telemetry_uart2_init(void);
void telemetry_process_task(void const *argument);
void telemetry_mavlink_proc(uint8_t c);
void telemetry_data_request_read(void);
void USART2_IRQHandler(void);

int32_t telemetry_init(void)
{
    telemetry_uart2_init();
    
    osSemaphoreDef(TELEMETRY_SEM);
    telemetry_sema = osSemaphoreEmptyCreate(osSemaphore(TELEMETRY_SEM));
    if (NULL == telemetry_sema)
    {
        printf("[%s, L%d] create semaphore failed ret 0x%x.\r\n", 
            __FILE__, __LINE__, (unsigned int)telemetry_sema);
        return -1;
    }

    osThreadDef(telemetryTask, telemetry_process_task, osPriorityNormal, 0, 1024);
    telemetryTaskHandle = osThreadCreate(osThread(telemetryTask), NULL);
    if (NULL == telemetryTaskHandle)
    {
        printf("[%s, L%d] create thread failed.\r\n", 
            __FILE__, __LINE__);
        return -1;        
    }

    HAL_NVIC_SetPriority(USART2_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    
    return 0;
}

/* USART2 init function */
void telemetry_uart2_init(void)
{

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 57600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    return;
}

void telemetry_mavlink_proc(uint8_t c)
{
    mavlink_message_t msg;
    mavlink_status_t  status;
    
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
        switch (msg.msgid)
        {
            case MAVLINK_MSG_ID_HEARTBEAT:
			{
                heartbeat_cnt++;
                mavlink_active = true;
                target_system.sysid = msg.sysid;
                target_system.compid = msg.compid;

				telemetry.heartbeat.base_mode =
					mavlink_msg_heartbeat_get_base_mode(&msg);
				telemetry.heartbeat.custom_mode = 
					mavlink_msg_heartbeat_get_custom_mode(&msg);
				telemetry.heartbeat.type = 
					mavlink_msg_heartbeat_get_type(&msg);
				telemetry.heartbeat.system_status = 
					mavlink_msg_heartbeat_get_system_status(&msg);

				last_heart_beat_time = timer_1ms;
                if (waitting_for_heartbeat)
                {
                    request_send = true;
                }
                (void)osSemaphoreReleaseFromISR(telemetry_sema);
            }
            break;
            case MAVLINK_MSG_ID_SYS_STATUS:
            {
				telemetry.sys_status.voltage_battery = 
                    mavlink_msg_sys_status_get_voltage_battery(&msg);
                telemetry.sys_status.current_battery = 
                    mavlink_msg_sys_status_get_current_battery(&msg);
                telemetry.sys_status.battery_remaining =
                    mavlink_msg_sys_status_get_battery_remaining(&msg);
            }
			break;
			case MAVLINK_MSG_ID_GPS_RAW_INT:
            {
				telemetry.gps_raw.lat = mavlink_msg_gps_raw_int_get_lat(&msg);
                telemetry.gps_raw.lon = mavlink_msg_gps_raw_int_get_lon(&msg);
                telemetry.gps_raw.fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
                telemetry.gps_raw.satellites_visible = 
					mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
			}
			break;
			case MAVLINK_MSG_ID_VFR_HUD:
			{
				telemetry.vfr_hud.alt = mavlink_msg_vfr_hud_get_alt(&msg);
				telemetry.vfr_hud.heading = mavlink_msg_vfr_hud_get_heading(&msg);
				telemetry.vfr_hud.throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
			}
			break;
			case MAVLINK_MSG_ID_ATTITUDE:
            {
				telemetry.attitude.roll  = mavlink_msg_attitude_get_roll(&msg);
                telemetry.attitude.pitch = mavlink_msg_attitude_get_pitch(&msg);
			}
			break;
            case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
			{
				telemetry.rc_channels_raw.rssi = 
					mavlink_msg_rc_channels_raw_get_rssi(&msg);
			}
			break;
            default:
                break;
                
        }
    }

    packet_drops += status.packet_rx_drop_count;

    return;
}

void USART2_IRQHandler(void)
{
    uint8_t c;
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
    {
        c = huart2.Instance->DR;
        //printf("0x%x \r\n", c);
        telemetry_mavlink_proc(c);
    }
}

void telemetry_data_request_read(void)
{
    mavlink_message_t msg;
    uint16_t len;
    uint32_t i;

    for (i = 0; i < streams_max; i++)
    {
        mavlink_msg_request_data_stream_pack(mavlink_system.sysid, 
            mavlink_system.compid, &msg, target_system.sysid, target_system.compid, 
            streams[i].stream_id, streams[i].freq, 1);
            
        len = mavlink_msg_to_send_buffer(buf, &msg);

        HAL_UART_Transmit(&huart2, buf, len, 5000);
    }

    return;
}

// armed/disarmed, is flying, flight mode
// 3 Bytes
uint8_t telemetry_push_mode(void* buf){
    uint8_t arm_type = 0;
	uint8_t* ptr = buf;
    uint8_t len;

    uint8_t status = 0;
	bool isFlying = false;
	uint8_t system_status = telemetry.heartbeat.system_status;

    telemetry.heartbeat.system_status;

	// arm/disarm
    if (telemetry.heartbeat.base_mode & ((uint8_t)1<<6) == 0x40){
	    status = 1; // armed
	}
	
	// is flying
    if (system_status == MAV_STATE.MAV_STATE_ACTIVE ||
		 (p_isFlying && 
		   (system_status == MAV_STATE.MAV_STATE_CRITICAL || 
		    system_status == MAV_STATE.MAV_STATE_EMERGENCY)){
	    // is flying now
	    status |= 0x10;
    }
	
	*ptr = status;
    ptr += sizeof(uint8_t);
    len = sizeof(uint8_t);
	
    // flight mode
    *ptr = telemetry.heartbeat.type & 0x7F;
	ptr += sizeof(uint8_t);
    len = sizeof(uint8_t);

	*ptr = (uint8_t)telemetry.heartbeat.custom_mode;
	len += sizeof(uint8_t);

	return len;
}

// 1 Byte
uint8_t telemetry_push_throttle(void* buf){
	*(uint8_t*)buf = (uint8_t)telemetry.vfr_hud.throttle;

	return sizeof(uint8_t);
}

// 1 Byte
uint8_t telemetry_push_rssi(void* buf){
    *(uint8_t*)buf = (uint8_t)telemetry.rc_channels_raw.rssi;

	return sizeof(uint8_t);
}


// battery
// 5 Bytes
uint8_t telemetry_push_volt_cur
(
    void *buf
)
{
    uint8_t* ptr = buf;
    uint8_t len;

	memcpy(ptr, &telemetry.sys_status.voltage_battery, sizeof(uint16_t));
    ptr += sizeof(uint16_t);
    len = sizeof(uint16_t);

	memcpy(ptr, &telemetry.sys_status.current_battery, sizeof(int16_t));
    ptr += sizeof(int16_t);
    len += sizeof(int16_t);

	memcpy(ptr, &telemetry.sys_status.battery_remaining, sizeof(int8_t));
    len += sizeof(int8_t);

    return len;
}

// gps fix type and satellites num.
// 2 Bytes
uint8_t telemetry_push_gps_status(void *buf){
    uint8_t *ptr = buf;
    uint8_t len;
    
    *ptr = telemetry.gps_raw.fix_type;
    ptr += sizeof(uint8_t);
    len = sizeof(uint8_t);
    
    *ptr = telemetry.gps_raw.satellites_visible;
    ptr += sizeof(uint8_t);
    len = sizeof(uint8_t);

    return len;
}


// 8 Bytes
uint8_t telemetry_push_attitude
(
    void *buf
)
{
    uint8_t *ptr = buf;
    uint8_t len;

	memcpy(ptr, &telemetry.attitude.roll, sizeof(float));
    ptr += sizeof(float);
    len = sizeof(float);

	memcpy(ptr, &telemetry.attitude.pitch, sizeof(float));
    len += sizeof(float);

    return len;
}

// heading 
// 2 Bytes
uint8_t telemetry_push_heading(void* buf){
    telemetry.vfr_hud.heading;
}

// alt
// 4 Bytes
uint8_t telemetry_push_alt(void* buf){
    telemetry.vfr_hud.alt;
}

// distance from home
// 2 Bytes
uint8_t telemetry_push_distance(void* buf){
}

void telemetry_process_task(void const *argument)
{
    uint32_t i;
    argument = argument;
    
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

    for (;;)
    {
        /* waitting for data update notify */
        (void)osSemaphoreWait(telemetry_sema, osWaitForever);

        if (request_send)
        {
            for (i = 0; i < 3; i++)
            {
                telemetry_data_request_read();
            }

            request_send = false;
            waitting_for_heartbeat = false;
        }


        printf("[raw imu]xg:%d, yg:%d, zg:%d\r\n", telemetry.raw_imu.xgyro, telemetry.raw_imu.ygyro, 
        telemetry.raw_imu.zgyro);
        printf("[attitude]roll:%f pitch:%f yaw:%f\r\n", ToDeg(telemetry.attitude.roll), 
        ToDeg(telemetry.attitude.pitch), ToDeg(telemetry.attitude.yaw));
    }
}

