#ifndef PI_UART_H
#define PI_UART_H

#include <stdint.h>
#include "message.h"

#define PI_UART_START_A 0xCAu
#define PI_UART_START_B 0xFEu
#define PI_UART_MSG_POSE 0x01u
#define PI_UART_MSG_PEER_POSE 0x02u
#define PI_UART_MAX_PAYLOAD 64u


typedef struct {
    float x;
    float y;
    float theta;
    float v;
} __attribute__((packed)) PiPayload_t;

void pi_uart_init(uint8_t robot_id);
void pi_uart_poll(uint32_t now_ms);
uint8_t pi_uart_get_pose(RobotPoseMsg_t *out_pose);
void    pi_uart_send_pose(const RobotPoseMsg_t *pose);

#endif
