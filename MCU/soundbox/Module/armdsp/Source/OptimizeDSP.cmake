# CMSIS-DSP Optimization Configuration
# This file allows fine-tuning of CMSIS-DSP features to reduce Flash usage

# Add compilation definitions to disable unused features
target_compile_definitions(CMSISDSP PRIVATE
    # Disable unused data types if not needed
    # ARM_MATH_MATRIX_CHECK=0        # Disable matrix size checks (already set)
    # ARM_MATH_ROUNDING=0            # Disable rounding (already set)
    
    # Disable unused FFT sizes (uncomment to disable specific sizes)
    # ARM_TABLE_BITREV_1024=0
    # ARM_TABLE_TWIDDLECOEF_F32_4096=0
    # ARM_TABLE_TWIDDLECOEF_F32_2048=0
    # ARM_TABLE_TWIDDLECOEF_F32_1024=0
    # ARM_TABLE_TWIDDLECOEF_F32_512=0
    # ARM_TABLE_TWIDDLECOEF_F32_256=0
    # ARM_TABLE_TWIDDLECOEF_F32_128=0
    
    # Keep only essential FFT sizes (64 and below for your example)
    ARM_TABLE_TWIDDLECOEF_F32_64=1
    ARM_TABLE_TWIDDLECOEF_F32_32=1
    ARM_TABLE_TWIDDLECOEF_F32_16=1
)