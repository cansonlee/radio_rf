#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__


#define ToDeg(x) (x*57.2957795131)	// *180/pi

typedef struct _telemetry_heartbeat_s
{
    uint32_t custom_mode;   ///< A bitfield for use for autopilot-specific flags.
    uint8_t  type;          ///< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
    uint8_t  base_mode;     ///< System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
    uint8_t  system_status; ///< System status flag, see MAV_STATE ENUM
} TELEMETRY_HEARTBEAT;

typedef struct _telemetry_sys_status_s
{
    uint16_t voltage_battery;   ///< Battery voltage, in millivolts (1 = 1 millivolt)
    int16_t  current_battery;   ///< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
    int8_t   battery_remaining; ///< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
} TELEMETRY_SYS_STATUS;


typedef struct _telemetry_gps_raw_s
{
    int32_t lat;                ///< Latitude in 1E7 degrees
    int32_t lon;                ///< Longitude in 1E7 degrees
    uint8_t fix_type;           ///< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will 
                                ///< not use the value of this field unless it is at least two, 
                                ///< so always correctly fill in the fix.
    uint8_t satellites_visible; ///< Number of satellites visible. If unknown, set to 255
} TELEMETRY_GPS_RAW;

typedef struct _telemetry_vfr_hud_t
{
    float airspeed;    ///< Current airspeed in m/s
    float groundspeed; ///< Current ground speed in m/s
    float alt;         ///< Current altitude (MSL), in meters
    float climb;       ///< Current climb rate in meters/second
    int16_t heading;   ///< Current heading in degrees, in compass units (0..360, 0=north)
    uint16_t throttle; ///< Current throttle setting in integer percent, 0 to 100
} TELEMETRY_VFR_HUD;

typedef struct _telemetry_attitude_s
{
    float roll;  ///< Roll angle (rad, -pi..+pi)
    float pitch; ///< Pitch angle (rad, -pi..+pi)
} TELEMETRY_ATTITUDE;

typedef struct _telemetry_nav_controller_output_t
{
    float    nav_roll;       ///< Current desired roll in degrees
    float    nav_pitch;      ///< Current desired pitch in degrees
    float    alt_error;      ///< Current altitude error in meters
    float    aspd_error;     ///< Current airspeed error in meters/second
    float    xtrack_error;   ///< Current crosstrack error on x-y plane in meters
    int16_t  nav_bearing;    ///< Current desired heading in degrees
    int16_t  target_bearing; ///< Bearing to current MISSION/target in degrees
    uint16_t wp_dist;        ///< Distance to active MISSION in meters
} TELEMETRY_NAV_CONTROLLER_OUTPUT;

typedef struct _telemetry_mission_current_t
{
    uint16_t seq; ///< Sequence
} TELEMETRY_MISSION_CURRENT;

typedef struct _telemetry_rc_channels_raw_t
{
    uint16_t chan1_raw; ///< RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    uint16_t chan2_raw; ///< RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    uint16_t chan5_raw; ///< RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    uint16_t chan6_raw; ///< RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    uint16_t chan7_raw; ///< RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    uint16_t chan8_raw; ///< RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
    uint8_t rssi;       ///< Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
} TELEMETRY_RC_CHANNELS_RAW;

/** 
 *  distance from home is calc by transmitter
 
    float rads = fabs(osd_home_lat) * 0.0174532925;
    double scaleLongDown = cos(rads);

    //DST to Home
    dstlat = fabs(osd_home_lat - osd_lat) * 111319.5;
    dstlon = fabs(osd_home_lon - osd_lon) * 111319.5 * scaleLongDown;
    osd_home_distance = sqrt(sq(dstlat) + sq(dstlon));
*/
typedef struct _telemetry_data_s
{   
    TELEMETRY_HEARTBEAT  heartbeat;            // armed/disarmed(base_mode bit7), flight mode(identified by custom_mode and type)
    TELEMETRY_SYS_STATUS sys_status;           // battery
	TELEMETRY_GPS_RAW    gps_raw;              // GPS num, lock/unlock
	TELEMETRY_VFR_HUD    vfr_hud;              // alt, heading, throttle	
	TELEMETRY_ATTITUDE	 attitude;             // roll, pitch, yaw    
	TELEMETRY_RC_CHANNELS_RAW rc_channels_raw; // rssi
    } TELEMETRY_DATA;

typedef struct _telemetry_stream_s
{
    uint8_t  stream_id;
    uint16_t freq;
} TELEMETRY_STREAM;

typedef void (*USARTIRQFUNC)(uint8_t);
typedef int32_t (*USARTINITFUNC)(void);

int32_t telemetry_init(USARTINITFUNC pfInit, USARTIRQFUNC pfIRQ);
int32_t telemetry_receiver_init(void);

void telemetry_mavlink_proc(uint8_t c);
uint8_t telemetry_data_encode(void* out_buf);

void telemetry_comm_proc(uint8_t c);
uint8_t telemetry_rxnum_get(void);
uint8_t telemetry_transmitter_mode_get(void);
void telemetry_transmitter_channel_get(uint8_t *addr, uint8_t len);


#endif
