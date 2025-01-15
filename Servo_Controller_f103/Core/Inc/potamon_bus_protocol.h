/*******************************************************
 * File: potamon_bus_protocol.h
 * Description: Header file with definitions of packet IDs, sizes
 *              and struct types for every packet used in potamon
 *              mecanum platform RS-485 bus communication
 * 
 * Author: JV4K
 * Created: January 5, 2025
 * Last Modified: January 10, 2025
 * 
 *******************************************************/
#ifndef __POTAMON_PROTOCOL_H__
#define __POTAMON_PROTOCOL_H__

#include "main.h"

// ========================================
//                PACKET IDS
// ========================================

// ----------- FSM packets IDs ------------
#define	ID_STATE_STBY	        0x00
#define	ID_STATE_VEL_CTRL	    0x01
#define	ID_STATE_POS_CTRL	    0x02
#define	ID_STATE_IND_WH_CTRL	0x03

// ----------- SYNC packets IDs ------------
#define	ID_SYNC_CTRL_S1	        0x06
#define	ID_SYNC_CTRL_S2	        0x07
#define	ID_SYNC_VEL_VEC_TASK	0x08
#define	ID_SYNC_ENC_S1	        0x09
#define	ID_SYNC_ENC_S2	        0x0A
#define	ID_SYNC_CUR_S1	        0x0B
#define	ID_SYNC_CUR_S2	        0x0C
#define	ID_SYNC_ODO	            0x0D
#define	ID_SYNC_IMU	            0x0E
#define	ID_SYNC_TRAJ_N	        0x0F
#define SYNC_RESET_ODO          0x10


// ----------- DATA packets IDs: control ------------
#define	ID_DATA_CTRL_S1	        0x40
#define	ID_DATA_CTRL_S2	        0x41
#define	ID_DATA_VEL_VEC_TASK	0x42
#define	ID_DATA_TRAJ_N	        0x43
#define	ID_DATA_TRAJ_CHUNK	    0x44

// ----------- DATA packets IDs: feedback ------------
#define	ID_DATA_ENC_S1	        0x50
#define	ID_DATA_ENC_S2	        0x51
#define	ID_DATA_CUR_S1	        0x52
#define	ID_DATA_CUR_S2	        0x53
#define	ID_DATA_ODO	            0x54
#define	ID_DATA_IMU	            0x55


// ========================================
//            PACKET SIZES (BYTES)
// ========================================

// ----------- SYNC packets size ------------
#define S_SYNC           2

// ----------- SYNC packets sizes -----------
#define	S_DATA_CTRL_S1	        16
#define	S_DATA_CTRL_S2	        16
#define	S_DATA_VEL_VEC_TASK     9
#define	S_DATA_TRAJ_N	        8
#define	S_DATA_TRAJ_CHUNK       67

#define	S_DATA_ENC_S1	        15
#define	S_DATA_ENC_S2	        15
#define	S_DATA_CUR_S1	        7
#define	S_DATA_CUR_S2	        7
#define	S_DATA_ODO	            17
#define	S_DATA_IMU	            29


// ========================================
//         STRUCT TYPES FOR PACKETS
// ========================================

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
    uint16_t velocity_cap;
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

// Type for packets of DATA_CUR_S1 (0x52) and DATA_CUR_S2 (0x53)
typedef struct 
{
    uint8_t ID;
    int16_t current1;
    int16_t current2;
    uint16_t crc16;
}__attribute__((packed)) pack_data_current_t;

// Type for packet of DATA_ODO (0x54)
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

// Type for packet of DATA_IMU(0x55)
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
    int16_t gavity_x;
    int16_t gavity_y;
    int16_t gavity_z;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    uint16_t crc16;
}__attribute__((packed)) pack_data_imu_t;

typedef enum
{
    SYNC,
    DATA_CTRL_S1,
    DATA_CTRL_S2,
    DATA_VEL_VEC_TASK,
    DATA_ENC_S1,
    DATA_ENC_S2,
    DATA_CUR_S1,
    DATA_CUR_S2,
    DATA_ODO,
    DATA_IMU,
    DATA_TRAJ_N,
    DATA_TRAJ_CHUNK
} next_packet_t;

#endif