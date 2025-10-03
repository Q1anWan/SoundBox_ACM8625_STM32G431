/**
 * @file minimal_cmsis_test.c
 * @brief Minimal CMSIS-DSP test with reduced Flash footprint
 */

#include "arm_math.h"
#include "main.h"

/**
 * @brief Minimal test function using only essential CMSIS-DSP functions
 * @note Uses only BasicMath and Statistics functions to minimize Flash usage
 */
void minimal_cmsis_test(void)
{
    // Test basic math functions (very small Flash footprint)
    float32_t input[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    float32_t output[4];
    
    // Basic vector addition - minimal Flash usage
    arm_add_f32(input, input, output, 4);
    
    // Basic statistics - small Flash usage
    float32_t mean_value;
    arm_mean_f32(input, 4, &mean_value);
    
    // Only include if you really need FFT (comment out to save Flash)
    /*
    arm_rfft_fast_instance_f32 fft_instance;
    arm_rfft_fast_init_f32(&fft_instance, 64);  // Only 64-point FFT
    */
    
    // Only include if you really need matrix operations (comment out to save Flash)
    /*
    arm_matrix_instance_f32 mat_a, mat_b, mat_result;
    float32_t mat_a_data[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    float32_t mat_b_data[4] = {5.0f, 6.0f, 7.0f, 8.0f};
    float32_t mat_result_data[4];
    
    arm_mat_init_f32(&mat_a, 2, 2, mat_a_data);
    arm_mat_init_f32(&mat_b, 2, 2, mat_b_data);
    arm_mat_init_f32(&mat_result, 2, 2, mat_result_data);
    
    arm_mat_mult_f32(&mat_a, &mat_b, &mat_result);
    */
}