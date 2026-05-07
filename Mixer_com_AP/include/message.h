#ifndef MESSAGE_H
#define MESSAGE_H

#include <stdint.h>
#include <stddef.h>

// fixed point math for Q16.16 
#define FP_SCALE (65536L)
#define FP_FROM_FLOAT(x) ((int32_t)((x) * FP_SCALE))
#define FP_TO_FLOAT(x) ((float)(x)  / FP_SCALE)
#define FP_MUL(a, b) ((int32_t)(((int64_t)(a) * (b)) >> 16))
#define FP_DIV(a, b) ((int32_t)(((int64_t)(a) << 16) / (b)))

#define BOLT_MAX_PAYLOAD 64u

// Structure of messages
typedef struct {
    uint32_t timestamp_ms;
    int32_t x_fp; // x coordinate
    int32_t y_fp; // y coordinate
    int32_t theta_fp; // orientation
    int32_t v_fp; // linear velocity
    uint8_t robot_id; // 0, 1, or 2
    uint8_t status; // see POSE_STATUS_* flags
} __attribute__((packed)) RobotPoseMsg_t;

// status flags for RobotPoseMsg
#define POSE_STATUS_VALID (1u << 0)   // data is fresh
#define POSE_STATUS_STALE (1u << 1)   // no update for over 500ms
#define POSE_STATUS_INITIALISED (1u << 2)   // first message has been received

#define NUM_ROBOTS 3
#define MESSAGE_SIZE sizeof(RobotPoseMsg_t)


// BOLT packages

#define MX_PAYLOAD_ONLY  20  
#define AGGREGATE_SIZE    6

// for communication
enum bolt_pkt_type
{
    BOLT_SYNC = 0,
    BOLT_POSE = 1,
    BOLT_LIDAR = 2,

};


typedef struct __attribute__((packed))
{
    uint16_t round;
} sync_pkt_t;


typedef struct __attribute__((packed))
{
    uint8_t  robot_id;
    int32_t  x_fp;
    int32_t  y_fp;
    int32_t  theta_fp;
    int32_t  v_fp;
    uint32_t timestamp_ms;
} pose_pkt_t;


typedef struct __attribute__((packed))
{
    struct __attribute__((packed))
    {
        uint8_t type;
        uint8_t pad;
    };
    union __attribute__((packed))
    {
        uint8_t     payload_start;
        sync_pkt_t  sync;
        pose_pkt_t  pose;
    };
} bolt_pkt_t;

#define BOLT_PKT_HEADER_SIZE  offsetof(bolt_pkt_t, payload_start)
#define LEN_BOLT_POSE         (BOLT_PKT_HEADER_SIZE + sizeof(pose_pkt_t))
#define LEN_BOLT_SYNC         (BOLT_PKT_HEADER_SIZE + sizeof(sync_pkt_t))

#endif 
