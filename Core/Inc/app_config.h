#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define COMM_MODE_USB 0U
#define COMM_MODE_CAN 1U

/* Change only this value: 0 = USB CDC, 1 = CAN. */
#ifndef COMMUNICATION_MODE
#define COMMUNICATION_MODE 0U
#endif

#if (COMMUNICATION_MODE != COMM_MODE_USB) && \
    (COMMUNICATION_MODE != COMM_MODE_CAN)
#error "COMMUNICATION_MODE must be COMM_MODE_USB (0) or COMM_MODE_CAN (1)"
#endif

/* Standard CAN identifier used by Comm_Send(). */
#ifndef COMM_CAN_TX_STD_ID
#define COMM_CAN_TX_STD_ID 0x300U
#endif

#if (COMM_CAN_TX_STD_ID > 0x7FFU)
#error "COMM_CAN_TX_STD_ID must be an 11-bit standard CAN identifier"
#endif

#endif /* APP_CONFIG_H */
