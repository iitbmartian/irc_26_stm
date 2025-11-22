#include "quad_gpio.h"
volatile int32_t quad_count = 0;

volatile uint8_t prev_state = 0;
volatile uint8_t curr_state;

// Quadrature state lookup table
const int8_t quad_table[16] = {
    0,  // 00 -> 00 (no change)
    1,  // 00 -> 01 (CW)
   -1,  // 00 -> 10 (CCW)
    0,  // 00 -> 11 (invalid)
   -1,  // 01 -> 00 (CCW)
    0,  // 01 -> 01 (no change)
    0,  // 01 -> 10 (invalid)
    1,  // 01 -> 11 (CW)
    1,  // 10 -> 00 (CW)
    0,  // 10 -> 01 (invalid)
    0,  // 10 -> 10 (no change)
   -1,  // 10 -> 11 (CCW)
    0,  // 11 -> 00 (invalid)
   -1,  // 11 -> 01 (CCW)
    1,  // 11 -> 10 (CW)
    0   // 11 -> 11 (no change)
};

void Encoder_Init(void)
{
    // Read initial state
    uint8_t a = HAL_GPIO_ReadPin(DRILL_QUAD_A_GPIO_Port, DRILL_QUAD_A_Pin);
    uint8_t b = HAL_GPIO_ReadPin(DRILL_QUAD_B_GPIO_Port, DRILL_QUAD_B_Pin);
    prev_state = (b << 1) | a;
}

void Encoder_Process(void)
{
    // Read current state
    uint8_t a = HAL_GPIO_ReadPin(DRILL_QUAD_A_GPIO_Port, DRILL_QUAD_A_Pin);
    uint8_t b = HAL_GPIO_ReadPin(DRILL_QUAD_B_GPIO_Port, DRILL_QUAD_B_Pin);
    curr_state = (b << 1) | a;

    // Create lookup index: [prev_state:2][curr_state:2]
    uint8_t index = (prev_state << 2) | curr_state;

    // Update counter based on transition
    quad_count += quad_table[index];

    // Update previous state
    prev_state = curr_state;
}
