#ifndef SKIN_SCAN_H
#define SKIN_SCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define SKIN_ROWS              14U
#define SKIN_COLS              14U
#define SKIN_ROW_EXCITATION_US 350U
#define SKIN_STARTUP_SETTLE_MS 300U
#define SKIN_CALIBRATION_FRAMES 100U
#define SKIN_OVERSAMPLE_COUNT    8U
#define SKIN_BASELINE_ALPHA_DENOMINATOR 100L
#define SKIN_PRESS_ON_THRESHOLD  40U
#define SKIN_PRESS_OFF_THRESHOLD 20U
#define SKIN_PRESS_CONFIRM_FRAMES  3U
#define SKIN_RELEASE_CONFIRM_FRAMES 5U

/* Set to 0 if hardware tests show that pressing increases the ADC value. */
#define SKIN_PRESS_ADC_DECREASES 1U

extern uint16_t g_skin_matrix[SKIN_ROWS][SKIN_COLS];
extern uint16_t g_skin_baseline[SKIN_ROWS][SKIN_COLS];
extern uint16_t g_skin_pressure[SKIN_ROWS][SKIN_COLS];
extern uint8_t g_skin_pressed[SKIN_ROWS][SKIN_COLS];
extern volatile uint32_t g_skin_frame_count;
extern volatile uint32_t g_skin_output_count;
extern volatile uint32_t g_skin_dropped_frames;
extern volatile uint16_t g_skin_calibration_frames;
extern volatile uint8_t g_skin_baseline_ready;

HAL_StatusTypeDef SkinScan_Init(void);
HAL_StatusTypeDef SkinScan_ScanNextRow(void);

#ifdef __cplusplus
}
#endif

#endif /* SKIN_SCAN_H */
