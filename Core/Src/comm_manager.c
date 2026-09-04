#include "comm_manager.h"

#include "app_config.h"
#include <string.h>

#if COMMUNICATION_MODE == COMM_MODE_USB
#include "usb_device.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_FS;
#else
#include "can.h"
#endif

static uint8_t tx_buffer[COMM_MAX_MESSAGE_SIZE];

#if COMMUNICATION_MODE == COMM_MODE_CAN
#define CAN_FRAGMENT_DATA_SIZE 6U
#define CAN_FRAGMENT_LAST_FLAG 0x80U

static uint16_t can_tx_length;
static uint16_t can_tx_offset;
static uint8_t can_tx_sequence;
static uint8_t can_tx_active;

static Comm_StatusTypeDef CAN_CommInit(void)
{
  CAN_FilterTypeDef filter = {0};

  MX_CAN_Init();

  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0U;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0U;
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
  {
    return COMM_STATUS_ERROR;
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    return COMM_STATUS_ERROR;
  }

  return COMM_STATUS_OK;
}
#endif

Comm_StatusTypeDef Comm_Init(void)
{
#if COMMUNICATION_MODE == COMM_MODE_USB
  MX_USB_DEVICE_Init();
  return COMM_STATUS_OK;
#else
  return CAN_CommInit();
#endif
}

Comm_StatusTypeDef Comm_Send(const uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U) ||
      (length > COMM_MAX_MESSAGE_SIZE))
  {
    return COMM_STATUS_ERROR;
  }

#if COMMUNICATION_MODE == COMM_MODE_USB
  USBD_CDC_HandleTypeDef *cdc;
  uint8_t usb_status;

  if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) ||
      (hUsbDeviceFS.pClassData == NULL))
  {
    return COMM_STATUS_BUSY;
  }

  cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (cdc->TxState != 0U)
  {
    return COMM_STATUS_BUSY;
  }

  memcpy(tx_buffer, data, length);
  usb_status = CDC_Transmit_FS(tx_buffer, length);
  if (usb_status == USBD_OK)
  {
    return COMM_STATUS_OK;
  }

  return (usb_status == USBD_BUSY) ? COMM_STATUS_BUSY : COMM_STATUS_ERROR;
#else
  if (can_tx_active != 0U)
  {
    return COMM_STATUS_BUSY;
  }

  memcpy(tx_buffer, data, length);
  can_tx_length = length;
  can_tx_offset = 0U;
  ++can_tx_sequence;
  can_tx_active = 1U;
  Comm_Process();
  return COMM_STATUS_OK;
#endif
}

void Comm_Process(void)
{
#if COMMUNICATION_MODE == COMM_MODE_CAN
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox;
  uint16_t remaining;
  uint8_t payload_length;
  uint8_t frame[8];
  uint8_t fragment_index;

  header.StdId = COMM_CAN_TX_STD_ID;
  header.ExtId = 0U;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.TransmitGlobalTime = DISABLE;

  while ((can_tx_active != 0U) &&
         (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0U))
  {
    remaining = (uint16_t)(can_tx_length - can_tx_offset);
    payload_length = (remaining > CAN_FRAGMENT_DATA_SIZE) ?
                     CAN_FRAGMENT_DATA_SIZE : (uint8_t)remaining;
    fragment_index = (uint8_t)(can_tx_offset / CAN_FRAGMENT_DATA_SIZE);

    frame[0] = can_tx_sequence;
    frame[1] = fragment_index;
    if (payload_length == remaining)
    {
      frame[1] |= CAN_FRAGMENT_LAST_FLAG;
    }
    memcpy(&frame[2], &tx_buffer[can_tx_offset], payload_length);
    header.DLC = (uint32_t)payload_length + 2U;

    if (HAL_CAN_AddTxMessage(&hcan, &header, frame, &mailbox) != HAL_OK)
    {
      return;
    }

    can_tx_offset = (uint16_t)(can_tx_offset + payload_length);
    if (can_tx_offset >= can_tx_length)
    {
      can_tx_active = 0U;
    }
  }
#endif
}

uint8_t Comm_IsBusy(void)
{
#if COMMUNICATION_MODE == COMM_MODE_USB
  USBD_CDC_HandleTypeDef *cdc;

  if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) ||
      (hUsbDeviceFS.pClassData == NULL))
  {
    return 1U;
  }

  cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  return (cdc->TxState != 0U) ? 1U : 0U;
#else
  return can_tx_active;
#endif
}

uint8_t Comm_IrqHandler(void)
{
#if COMMUNICATION_MODE == COMM_MODE_USB
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
#else
  HAL_CAN_IRQHandler(&hcan);
#endif
  return 1U;
}
