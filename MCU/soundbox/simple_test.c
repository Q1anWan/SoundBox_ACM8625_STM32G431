/**
 * @file simple_test.c
 * @brief Minimal test to verify CMSIS-DSP header inclusion
 */

#include "arm_math.h"

// Simple test to verify the header is found
static float32_t test_data[4] = {1.0f, 2.0f, 3.0f, 4.0f};

/**
 * @brief Minimal test function to verify CMSIS-DSP integration
 */
void simple_cmsis_test(void)
{
    float32_t result;
    
    // Just test that the function exists and can be called
    arm_mean_f32(test_data, 4, &result);
    
    // Basic math operation
    float32_t output[4];
    arm_add_f32(test_data, test_data, output, 4);
}