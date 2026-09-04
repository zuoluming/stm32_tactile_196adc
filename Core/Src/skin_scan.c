#include "skin_scan.h"

#include "adc.h"
#include "comm_manager.h"
#include "main.h"
#include <string.h>

#define QB_PIN               GPIO_PIN_13
#define HB_PIN               GPIO_PIN_14
#define COLUMN_GPIO_PORT     GPIOC
#define ROW_SETTLE_US        5U
#define COLUMN_SETTLE_US     5U
#define ADC_GROUP_TIMEOUT_US 250U

#define BASELINE_Q_SHIFT     16U
#define BASELINE_Q_ROUND     (1L << (BASELINE_Q_SHIFT - 1U))
#define FRAME_HEADER_SIZE    2U
#define FRAME_PAYLOAD_SIZE   (SKIN_ROWS * SKIN_COLS * 2U)
#define FRAME_SIZE           (FRAME_HEADER_SIZE + FRAME_PAYLOAD_SIZE)

#if SKIN_CALIBRATION_FRAMES <= 2U
#error "SKIN_CALIBRATION_FRAMES must be greater than 2"
#endif

#if SKIN_OVERSAMPLE_COUNT <= 2U
#error "SKIN_OVERSAMPLE_COUNT must be greater than 2"
#endif

#if FRAME_SIZE > COMM_MAX_MESSAGE_SIZE
#error "COMM_MAX_MESSAGE_SIZE is too small for one skin data frame"
#endif

uint16_t g_skin_matrix[SKIN_ROWS][SKIN_COLS];
uint16_t g_skin_baseline[SKIN_ROWS][SKIN_COLS];
uint16_t g_skin_pressure[SKIN_ROWS][SKIN_COLS];
uint8_t g_skin_pressed[SKIN_ROWS][SKIN_COLS];
volatile uint32_t g_skin_frame_count;
volatile uint32_t g_skin_output_count;
volatile uint32_t g_skin_dropped_frames;
volatile uint16_t g_skin_calibration_frames;
volatile uint8_t g_skin_baseline_ready;

static volatile uint8_t adc_transfer_done;
static volatile HAL_StatusTypeDef adc_transfer_status;
static uint8_t current_row;
static uint32_t cycles_per_us;
static uint8_t aggregate_count;
static uint32_t aggregate_sum[SKIN_ROWS][SKIN_COLS];
static uint16_t aggregate_min[SKIN_ROWS][SKIN_COLS];
static uint16_t aggregate_max[SKIN_ROWS][SKIN_COLS];
static uint8_t aggregate_locked[SKIN_ROWS][SKIN_COLS];
static int32_t baseline_q16[SKIN_ROWS][SKIN_COLS];
static uint8_t press_count[SKIN_ROWS][SKIN_COLS];
static uint8_t release_count[SKIN_ROWS][SKIN_COLS];
static uint8_t frame_buffer[FRAME_SIZE];

static uint32_t elapsed_cycles(uint32_t start)
{
  return (uint32_t)(DWT->CYCCNT - start);
}

static void delay_us(uint32_t delay)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t delay_cycles = delay * cycles_per_us;

  while (elapsed_cycles(start) < delay_cycles)
  {
  }
}

static void disable_rows(void)
{
  GPIOB->BRR = ROW_EN1_Pin | ROW_EN2_Pin;
}

static void disable_columns(void)
{
  COLUMN_GPIO_PORT->BRR = QB_PIN | HB_PIN;
}

static void set_row_address(uint8_t address)
{
  uint16_t address_pins = ROW_A0_Pin | ROW_A1_Pin | ROW_A2_Pin;
  uint16_t pins_to_set = 0U;

  if ((address & 0x01U) != 0U)
  {
    pins_to_set |= ROW_A0_Pin;
  }
  if ((address & 0x02U) != 0U)
  {
    pins_to_set |= ROW_A1_Pin;
  }
  if ((address & 0x04U) != 0U)
  {
    pins_to_set |= ROW_A2_Pin;
  }

  GPIOB->BSRR = ((uint32_t)address_pins << 16U) | pins_to_set;
}

static void enable_row(uint8_t row)
{
  GPIOB->BSRR = (row < 8U) ? ROW_EN1_Pin : ROW_EN2_Pin;
}

static HAL_StatusTypeDef sample_group(uint16_t *destination)
{
  uint32_t start;
  HAL_StatusTypeDef conversion_status;
  HAL_StatusTypeDef status;

  adc_transfer_done = 0U;
  adc_transfer_status = HAL_BUSY;

  status = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)destination, 7U);
  if (status != HAL_OK)
  {
    return status;
  }

  start = DWT->CYCCNT;
  while (adc_transfer_done == 0U)
  {
    if (elapsed_cycles(start) >= (ADC_GROUP_TIMEOUT_US * cycles_per_us))
    {
      (void)HAL_ADC_Stop_DMA(&hadc1);
      return HAL_TIMEOUT;
    }
  }

  conversion_status = adc_transfer_status;
  if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return conversion_status;
}

static uint16_t baseline_value(uint8_t row, uint8_t col)
{
  return (uint16_t)((baseline_q16[row][col] + BASELINE_Q_ROUND) >>
                    BASELINE_Q_SHIFT);
}

static uint16_t pressure_delta(uint16_t sample, uint16_t baseline)
{
#if SKIN_PRESS_ADC_DECREASES != 0U
  return (baseline > sample) ? (uint16_t)(baseline - sample) : 0U;
#else
  return (sample > baseline) ? (uint16_t)(sample - baseline) : 0U;
#endif
}

static uint16_t absolute_difference(uint16_t first, uint16_t second)
{
  return (first > second) ? (uint16_t)(first - second) :
                            (uint16_t)(second - first);
}

static void aggregate_reset(void)
{
  uint8_t row;
  uint8_t col;

  aggregate_count = 0U;
  for (row = 0U; row < SKIN_ROWS; ++row)
  {
    for (col = 0U; col < SKIN_COLS; ++col)
    {
      aggregate_sum[row][col] = 0U;
      aggregate_min[row][col] = 0xFFFFU;
      aggregate_max[row][col] = 0U;
      aggregate_locked[row][col] = 0U;
    }
  }
}

static void aggregate_add_frame(void)
{
  uint8_t row;
  uint8_t col;
  uint16_t sample;

  for (row = 0U; row < SKIN_ROWS; ++row)
  {
    for (col = 0U; col < SKIN_COLS; ++col)
    {
      sample = g_skin_matrix[row][col];
      aggregate_sum[row][col] += sample;
      if (sample < aggregate_min[row][col])
      {
        aggregate_min[row][col] = sample;
      }
      if (sample > aggregate_max[row][col])
      {
        aggregate_max[row][col] = sample;
      }
    }
  }
  ++aggregate_count;
}

static void finish_initial_calibration(void)
{
  uint8_t row;
  uint8_t col;
  uint32_t trimmed_sum;
  uint16_t initial_baseline;

  for (row = 0U; row < SKIN_ROWS; ++row)
  {
    for (col = 0U; col < SKIN_COLS; ++col)
    {
      trimmed_sum = aggregate_sum[row][col] -
                    aggregate_min[row][col] - aggregate_max[row][col];
      initial_baseline = (uint16_t)((trimmed_sum +
          ((SKIN_CALIBRATION_FRAMES - 2U) / 2U)) /
          (SKIN_CALIBRATION_FRAMES - 2U));
      baseline_q16[row][col] =
          (int32_t)((uint32_t)initial_baseline << BASELINE_Q_SHIFT);
      g_skin_baseline[row][col] = initial_baseline;
    }
  }

  aggregate_reset();
  g_skin_baseline_ready = 1U;
}

static void update_press_state(void)
{
  uint8_t row;
  uint8_t col;
  uint16_t sample;
  uint16_t baseline;
  uint16_t delta;

  for (row = 0U; row < SKIN_ROWS; ++row)
  {
    for (col = 0U; col < SKIN_COLS; ++col)
    {
      sample = g_skin_matrix[row][col];
      baseline = baseline_value(row, col);
      delta = pressure_delta(sample, baseline);

      if (g_skin_pressed[row][col] != 0U)
      {
        aggregate_locked[row][col] = 1U;
        if (delta <= SKIN_PRESS_OFF_THRESHOLD)
        {
          if (release_count[row][col] < SKIN_RELEASE_CONFIRM_FRAMES)
          {
            ++release_count[row][col];
          }
          if (release_count[row][col] >= SKIN_RELEASE_CONFIRM_FRAMES)
          {
            g_skin_pressed[row][col] = 0U;
            press_count[row][col] = 0U;
            release_count[row][col] = 0U;
          }
        }
        else
        {
          release_count[row][col] = 0U;
        }
      }
      else if (delta >= SKIN_PRESS_ON_THRESHOLD)
      {
        aggregate_locked[row][col] = 1U;
        if (press_count[row][col] < SKIN_PRESS_CONFIRM_FRAMES)
        {
          ++press_count[row][col];
        }
        if (press_count[row][col] >= SKIN_PRESS_CONFIRM_FRAMES)
        {
          g_skin_pressed[row][col] = 1U;
          release_count[row][col] = 0U;
        }
      }
      else
      {
        press_count[row][col] = 0U;
        release_count[row][col] = 0U;
        if (absolute_difference(sample, baseline) >=
            SKIN_PRESS_ON_THRESHOLD)
        {
          aggregate_locked[row][col] = 1U;
        }
      }
    }
  }
}

static void put_u16_le(uint8_t *output, uint16_t value)
{
  output[0] = (uint8_t)value;
  output[1] = (uint8_t)(value >> 8U);
}

static void build_output_frame(void)
{
  uint16_t offset = FRAME_HEADER_SIZE;
  uint8_t row;
  uint8_t col;

  frame_buffer[0] = 0xA5U;
  frame_buffer[1] = 0x5AU;

  for (row = 0U; row < SKIN_ROWS; ++row)
  {
    for (col = 0U; col < SKIN_COLS; ++col)
    {
      put_u16_le(&frame_buffer[offset], g_skin_pressure[row][col]);
      offset += 2U;
    }
  }
}

static void process_oversampled_frame(void)
{
  uint8_t row;
  uint8_t col;
  uint32_t trimmed_sum;
  uint16_t sample;
  uint16_t baseline;
  int32_t target_q16;
  int32_t baseline_error;
  Comm_StatusTypeDef comm_status;

  for (row = 0U; row < SKIN_ROWS; ++row)
  {
    for (col = 0U; col < SKIN_COLS; ++col)
    {
      trimmed_sum = aggregate_sum[row][col] -
                    aggregate_min[row][col] - aggregate_max[row][col];
      sample = (uint16_t)((trimmed_sum +
               ((SKIN_OVERSAMPLE_COUNT - 2U) / 2U)) /
               (SKIN_OVERSAMPLE_COUNT - 2U));
      baseline = baseline_value(row, col);

      if (g_skin_pressed[row][col] != 0U)
      {
        g_skin_pressure[row][col] = pressure_delta(sample, baseline);
      }
      else
      {
        g_skin_pressure[row][col] = 0U;
      }

      if ((aggregate_locked[row][col] == 0U) &&
          (absolute_difference(sample, baseline) <
           SKIN_PRESS_ON_THRESHOLD))
      {
        target_q16 = (int32_t)((uint32_t)sample << BASELINE_Q_SHIFT);
        baseline_error = target_q16 - baseline_q16[row][col];
        baseline_q16[row][col] +=
            baseline_error / SKIN_BASELINE_ALPHA_DENOMINATOR;
      }
      g_skin_baseline[row][col] = baseline_value(row, col);
    }
  }

  build_output_frame();
  ++g_skin_output_count;
  comm_status = Comm_Send(frame_buffer, FRAME_SIZE);
  if (comm_status != COMM_STATUS_OK)
  {
    ++g_skin_dropped_frames;
  }
  aggregate_reset();
}

static void process_complete_raw_frame(void)
{
  if (g_skin_baseline_ready == 0U)
  {
    aggregate_add_frame();
    ++g_skin_calibration_frames;
    if (g_skin_calibration_frames >= SKIN_CALIBRATION_FRAMES)
    {
      finish_initial_calibration();
    }
    return;
  }

  update_press_state();
  aggregate_add_frame();
  if (aggregate_count >= SKIN_OVERSAMPLE_COUNT)
  {
    process_oversampled_frame();
  }
}

HAL_StatusTypeDef SkinScan_Init(void)
{
  disable_rows();
  disable_columns();
  current_row = 0U;
  g_skin_frame_count = 0U;
  g_skin_output_count = 0U;
  g_skin_dropped_frames = 0U;
  g_skin_calibration_frames = 0U;
  g_skin_baseline_ready = 0U;

  memset(g_skin_matrix, 0, sizeof(g_skin_matrix));
  memset(g_skin_baseline, 0, sizeof(g_skin_baseline));
  memset(g_skin_pressure, 0, sizeof(g_skin_pressure));
  memset(g_skin_pressed, 0, sizeof(g_skin_pressed));
  memset(baseline_q16, 0, sizeof(baseline_q16));
  memset(press_count, 0, sizeof(press_count));
  memset(release_count, 0, sizeof(release_count));
  aggregate_reset();

  cycles_per_us = SystemCoreClock / 1000000U;
  if (cycles_per_us == 0U)
  {
    return HAL_ERROR;
  }

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
  {
    return HAL_ERROR;
  }

  HAL_Delay(SKIN_STARTUP_SETTLE_MS);
  return HAL_ADCEx_Calibration_Start(&hadc1);
}

HAL_StatusTypeDef SkinScan_ScanNextRow(void)
{
  uint8_t address = (uint8_t)(current_row & 0x07U);
  uint32_t row_start;
  HAL_StatusTypeDef status;

  disable_rows();
  disable_columns();
  set_row_address(address);

  /* Select the first seven columns before applying the row excitation. */
  COLUMN_GPIO_PORT->BSRR = QB_PIN;
  row_start = DWT->CYCCNT;
  enable_row(current_row);
  delay_us(ROW_SETTLE_US);

  status = sample_group(&g_skin_matrix[current_row][0]);
  if (status != HAL_OK)
  {
    disable_rows();
    disable_columns();
    return status;
  }

  COLUMN_GPIO_PORT->BRR = QB_PIN;
  COLUMN_GPIO_PORT->BSRR = HB_PIN;
  delay_us(COLUMN_SETTLE_US);

  status = sample_group(&g_skin_matrix[current_row][7]);
  COLUMN_GPIO_PORT->BRR = HB_PIN;
  if (status != HAL_OK)
  {
    disable_rows();
    return status;
  }

  while (elapsed_cycles(row_start) <
         (SKIN_ROW_EXCITATION_US * cycles_per_us))
  {
  }
  disable_rows();

  ++current_row;
  if (current_row >= SKIN_ROWS)
  {
    current_row = 0U;
    ++g_skin_frame_count;
    process_complete_raw_frame();
  }

  return HAL_OK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_transfer_status = HAL_OK;
    adc_transfer_done = 1U;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_transfer_status = HAL_ERROR;
    adc_transfer_done = 1U;
  }
}
