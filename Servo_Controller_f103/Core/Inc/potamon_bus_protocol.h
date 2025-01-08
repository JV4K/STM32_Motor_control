#ifndef __POTAMON_PROTOCOL_H__
#define __POTAMON_PROTOCOL_H__

#include "main.h"

/*=========== FSM packets IDs ===========*/
#define	STATE_STBY	        0x00
#define	STATE_VEL_CTRL	    0x01
#define	STATE_POS_CTRL	    0x02
#define	STATE_IND_WH_CTRL	0x03

/*=========== SYNC packets IDs ===========*/
#define	SYNC_CTRL_S1	    0x06
#define	SYNC_CTRL_S2	    0x07
#define	SYNC_VEL_VEC_TASK	0x08
#define	SYNC_ENC_S1	        0x09
#define	SYNC_ENC_S2	        0x0A
#define	SYNC_CUR_S1	        0x0B
#define	SYNC_CUR_S2	        0x0C
#define	SYNC_ODO	        0x0D
#define	SYNC_IMU	        0x0E
#define	SYNC_TRAJ_N	        0x0F

/*=========== SYNC packet size (2 bytes each - ID + CRC8) ===========*/
#define SYNC_SIZE           2

/*=========== DATA packets IDs ===========*/
/*=========== Control packets ===========*/
#define	DATA_CTRL_S1	    0x40
#define	DATA_CTRL_S2	    0x41
#define	DATA_VEL_VEC_TASK	0x42
#define	DATA_TRAJ_N	        0x43
#define	DATA_TRAJ_CHUNK	    0x44

/*=========== Feedback packets ===========*/
#define	DATA_ENC_S1	        0x50
#define	DATA_ENC_S2	        0x51
#define	DATA_CUR_S1	        0x52
#define	DATA_CUR_S2	        0x53
#define	DATA_ODO	        0x54
#define	DATA_IMU	        0x55

/*=========== Data packet sizes in bytes ===========*/
#define	S_DATA_CTRL_S1	    16
#define	S_DATA_CTRL_S2	    16
#define	S_DATA_VEL_VEC_TASK 9
#define	S_DATA_TRAJ_N	    6
#define	S_DATA_TRAJ_CHUNK   67

#define	S_DATA_ENC_S1	    15
#define	S_DATA_ENC_S2	    15
#define	S_DATA_CUR_S1	    7
#define	S_DATA_CUR_S2	    7
#define	S_DATA_ODO	        17
#define	S_DATA_IMU	        25


/* Types for packets*/

typedef struct 
{
    uint8_t ID;
    uint8_t crc8;
}__attribute__((packed)) pack_sync_t;

// Type for packet of DATA_CTRL_S1 (0x40) and DATA_CTRL_S2 (0x41)
typedef struct 
{
    uint8_t ID;
    int32_t angle_1;
    int16_t velocity_1;
    int32_t angle_2;
    int16_t velocity_2;
    uint8_t mode;
    uint16_t crc16;
}__attribute__((packed)) pack_data_ctrl_servo_t;

// Type for packet of DATA_VEL_VEC_TASK (0x42)
typedef struct 
{
    uint8_t ID;
    int16_t vx_task;
    int16_t vy_task;
    int16_t omega_task;
    uint16_t crc16;
}__attribute__((packed)) pack_data_vel_vec_t;

// Type for packet of DATA_TRAJ_N (0x43)
typedef struct 
{
    uint8_t ID;
    uint8_t no_of_chunks; // Number of chunks to recieve after this sync packet
    uint16_t total_points; // Total number of points to recieve
    uint16_t crc16;
}__attribute__((packed)) pack_data_traj_n_t;

// Type for packet of DATA_TRAJ_CHUNK (0x44)
typedef struct 
{
    uint8_t ID;
    // arrays of X and Y points. Multiplied by 1000 to store in int16
    int16_t xp_arr[16];
    int16_t yp_arr[16];
    uint16_t crc16;
}__attribute__((packed)) pack_data_points_chunk_t;

// Type for packets of DATA_ENC_S1 (0x50) and DATA_ENC_S2 (0x51)
typedef struct 
{
    uint8_t ID;
    int32_t angle_1;
    int16_t velocity_1;
    int32_t angle_2;
    int16_t velocity_2;
    uint16_t crc16;
}__attribute__((packed)) pack_data_encoder_t;

// Type for packet of DATA_TRAJ_N (0
typedef struct 
{
    uint8_t ID;
    int16_t vx_odo;
    int16_t vy_odo;
    int16_t omega_odo;
    int16_t x_pos_odo;
    int16_t y_pos_odo;
    int16_t heading_angle_odo;
    int16_t trajectory_progress;
    uint16_t crc16;
}__attribute__((packed)) pack_data_odo_t;

typedef struct 
{
    uint8_t ID;
    int16_t quat_W;
    int16_t quat_X;
    int16_t quat_Y;
    int16_t quat_Z;
    int16_t euler_roll;
    int16_t euler_pitch;
    int16_t euler_yaw;
    int16_t y_velocity_wtfisthis_iforgor;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    uint16_t crc16;
}__attribute__((packed)) pack_data_imu_t;


#endif