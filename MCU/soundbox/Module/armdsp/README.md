# CMSIS-DSP Static Library Build System

This directory contains the CMSIS-DSP library configured for STM32G431 with persistent static library caching.

## How it works

1. **First Build**: When `libCMSISDSP.a` doesn't exist, the library is compiled from source
2. **Subsequent Builds**: If `libCMSISDSP.a` exists, it's used directly (no recompilation)
3. **Library Persistence**: After successful compilation, the `.a` file is copied to `Module/armdsp/libCMSISDSP.a`

## Files Structure

```
Module/armdsp/
├── CMakeLists.txt          # Main entry point
├── libCMSISDSP.a          # Persistent static library (generated)
├── clean_lib.sh           # Script to force recompilation
├── README.md              # This file
├── Include/               # CMSIS-DSP headers
├── PrivateInclude/        # Internal headers
└── Source/                # Source code and build configuration
    ├── CMakeLists.txt     # Build configuration
    ├── configDsp.cmake    # DSP configuration
    └── */                 # Function modules
```

## Usage

### Normal Build
```bash
cmake --build build/Debug
```

### Force Recompilation
If you need to rebuild the CMSIS-DSP library (e.g., after changing compilation options):

```bash
./Module/armdsp/clean_lib.sh
cmake --build build/Debug
```

### Configuration Options

The library is pre-configured for STM32G431 (Cortex-M4 with FPU) with these optimizations:
- `LOOPUNROLL`: ON (Enable loop unrolling)
- `ROUNDING`: OFF (Better performance)
- `MATRIXCHECK`: OFF (Better performance) 
- `DISABLEFLOAT16`: ON (Not needed for STM32G431)

## Integration

The library automatically integrates with STM32CubeMX configuration and provides all CMSIS-DSP functions:

```c
#include "arm_math.h"

void example_usage() {
    // FFT functions
    arm_rfft_fast_instance_f32 fft_instance;
    arm_rfft_fast_init_f32(&fft_instance, 1024);
    
    // Math functions
    float32_t input[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    float32_t output[4];
    arm_add_f32(input, input, output, 4);
    
    // Matrix operations
    arm_matrix_instance_f32 matrix;
    // ... and many more
}
```