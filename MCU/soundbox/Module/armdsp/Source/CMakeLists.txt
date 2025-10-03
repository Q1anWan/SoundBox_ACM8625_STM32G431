cmake_minimum_required (VERSION 3.14)
cmake_policy(SET CMP0077 NEW)
project(CMSISDSP)

# DSP Sources
SET(DSP ${CMAKE_CURRENT_SOURCE_DIR}/..)

option(NEON "Neon acceleration" OFF)
option(NEONEXPERIMENTAL "Neon experimental acceleration" OFF)
option(HELIUMEXPERIMENTAL "Helium experimental acceleration" OFF)
option(LOOPUNROLL "Loop unrolling" ON)
option(ROUNDING "Rounding" OFF)
option(MATRIXCHECK "Matrix Checks" OFF)
option(HELIUM "Helium acceleration (MVEF and MVEI supported)" OFF)
option(MVEF "MVEF intrinsics supported" OFF)
option(MVEI "MVEI intrinsics supported" OFF)
option(MVEFLOAT16 "Float16 MVE intrinsics supported" OFF)
option(DISABLEFLOAT16 "Disable building float16 kernels" ON)
option(HOST "Build for host" OFF)
option(AUTOVECTORIZE "Prefer autovectorizable code to one using C intrinsics" OFF)
option(LAXVECTORCONVERSIONS "Lax vector conversions" ON)

###########################
#
# CMSIS DSP
#
###########################

# Define the static library path
set(CMSISDSP_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../libCMSISDSP.a")

# Check if pre-built static library exists
if(EXISTS "${CMSISDSP_LIB_PATH}")
    # Use existing static library
    message(STATUS "Found existing CMSIS-DSP library: ${CMSISDSP_LIB_PATH}")
    add_library(CMSISDSP STATIC IMPORTED GLOBAL)
    set_target_properties(CMSISDSP PROPERTIES
        IMPORTED_LOCATION "${CMSISDSP_LIB_PATH}"
        INTERFACE_INCLUDE_DIRECTORIES "${DSP}/Include"
    )
    
    # Add interface link to stm32cubemx if available
    if(TARGET stm32cubemx)
        set_property(TARGET CMSISDSP APPEND PROPERTY 
            INTERFACE_LINK_LIBRARIES stm32cubemx)
    endif()
    
    # Fallback CMSIS includes if stm32cubemx not available
    if (DEFINED CMSISCORE AND NOT TARGET stm32cubemx)
        set_property(TARGET CMSISDSP APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
            "${CMSISCORE}/Include"
            "${CMSISCORE}/Device/ST/STM32G4xx/Include"
        )
    endif()
    
else()
    # Build the static library from source
    message(STATUS "Building CMSIS-DSP library from source...")
    
    add_library(CMSISDSP STATIC)
    
    # Only include essential modules to reduce Flash usage
    include(BasicMathFunctions/Config.cmake)    # For arm_add_f32
    include(StatisticsFunctions/Config.cmake)   # For arm_mean_f32
    include(MatrixFunctions/Config.cmake)       # For matrix operations
    include(TransformFunctions/Config.cmake)    # For FFT functions
    include(CommonTables/Config.cmake)          # Required for transform functions
    include(SupportFunctions/Config.cmake)     # Support functions
    
    # Uncomment only if needed:
    include(ComplexMathFunctions/Config.cmake)
    include(QuaternionMathFunctions/Config.cmake)
    include(ControllerFunctions/Config.cmake)
    include(FastMathFunctions/Config.cmake)
    include(FilteringFunctions/Config.cmake)
    include(SVMFunctions/Config.cmake)
    include(BayesFunctions/Config.cmake)
    include(DistanceFunctions/Config.cmake)
    include(InterpolationFunctions/Config.cmake)
    include(WindowFunctions/Config.cmake)

    ### Includes
    target_include_directories(CMSISDSP PUBLIC "${DSP}/Include")

    # Use stm32cubemx configuration if available
    if(TARGET stm32cubemx)
        target_link_libraries(CMSISDSP PUBLIC stm32cubemx)
    endif()

    # Fallback to manual CMSIS core inclusion if stm32cubemx not available
    if (DEFINED CMSISCORE AND NOT TARGET stm32cubemx)
        target_include_directories(CMSISDSP PUBLIC "${CMSISCORE}/Include")
        target_include_directories(CMSISDSP PUBLIC "${CMSISCORE}/Device/ST/STM32G4xx/Include")
    endif()

    include(configDsp.cmake)
    configDsp(CMSISDSP)
    
    # Include additional optimizations
    # include(OptimizeDSP.cmake)
    
    # Apply optimization settings for size reduction - inherit from parent
    # The global compile options will be inherited, but add specific DSP optimizations
    target_compile_options(CMSISDSP PRIVATE
        -ffast-math              # Enable fast math for DSP operations
        # -fno-math-errno          # Don't set errno for math functions
        # -fno-signed-zeros        # Allow optimizations for signed zeros
        # -fno-trapping-math       # Allow optimizations assuming no FP traps
    )
    
    # Add custom command to copy the built library to persistent location
    add_custom_command(TARGET CMSISDSP POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:CMSISDSP>
        "${CMSISDSP_LIB_PATH}"
        COMMENT "Copying CMSIS-DSP library to persistent location: ${CMSISDSP_LIB_PATH}"
    )
    
endif()
