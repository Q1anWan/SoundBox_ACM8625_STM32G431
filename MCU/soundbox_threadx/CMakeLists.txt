cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

# Add global compiler flags for size optimization
add_compile_options()

# These infotmation are missed in cubemx generated files
if(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" OR CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(   
        -fno-builtin-printf # Disable builtin printf for size
        -fno-exceptions # Disable C++ exceptions (not needed for embedded)
        -fshort-enums # Use smallest possible enum type
        -ffast-math # Enable fast math optimizations
    )
endif()

# Add microcontroller-specific optimizations for all builds
add_compile_options(
    -fno-common # Allocate uninitialized data in bss
)

# Enable LTO only when the toolchain supports GNU-style LTO objects
if(CMAKE_C_COMPILER_ID MATCHES "GNU")
    add_compile_options(
        -flto # Link Time Optimization
    )
    add_link_options(
        -flto # Link Time Optimization
    )
endif()

# Set the project name
set(CMAKE_PROJECT_NAME soundbox_threadx)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Set CMSIS core path for CMSIS-DSP library
set(CMSISCORE ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS)

add_subdirectory(Module/armdsp)
add_subdirectory(Module/tinyusb)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Module/armdsp/Include # CMSIS-DSP headers must be explicitly added for IMPORTED libraries
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    CMSISDSP
    # Add user defined libraries
)

tinyusb_configure_target(${CMAKE_PROJECT_NAME})
