#!/bin/bash
# 
# Script to clean CMSIS-DSP static library
# Use this script when you want to force recompilation of CMSIS-DSP library
#

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
LIB_PATH="${SCRIPT_DIR}/libCMSISDSP.a"

if [ -f "${LIB_PATH}" ]; then
    echo "Removing existing CMSIS-DSP library: ${LIB_PATH}"
    rm -f "${LIB_PATH}"
    echo "CMSIS-DSP library removed. Next build will recompile from source."
else
    echo "No existing CMSIS-DSP library found at: ${LIB_PATH}"
fi