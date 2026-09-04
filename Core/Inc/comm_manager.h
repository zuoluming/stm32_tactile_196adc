#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define COMM_MAX_MESSAGE_SIZE 512U

typedef enum
{
  COMM_STATUS_OK = 0,
  COMM_STATUS_BUSY,
  COMM_STATUS_ERROR
} Comm_StatusTypeDef;

Comm_StatusTypeDef Comm_Init(void);
Comm_StatusTypeDef Comm_Send(const uint8_t *data, uint16_t length);
void Comm_Process(void);
uint8_t Comm_IsBusy(void);
uint8_t Comm_IrqHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* COMM_MANAGER_H */
