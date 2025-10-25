add_subdirectory(src)

set(TINYUSB_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")

function(tinyusb_configure_target target)
    tinyusb_target_add(${target})

    target_sources(${target} PRIVATE
        ${TINYUSB_ROOT_DIR}/bsp/board.c
        ${TINYUSB_ROOT_DIR}/bsp/board_port.c
        ${TINYUSB_ROOT_DIR}/bsp/family.c
        ${TINYUSB_ROOT_DIR}/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
        ${TINYUSB_ROOT_DIR}/user/audio.c
        ${TINYUSB_ROOT_DIR}/user/usb_descriptors.c
    )

    target_include_directories(${target} PRIVATE
        ${TINYUSB_ROOT_DIR}/bsp
        ${TINYUSB_ROOT_DIR}/user
    )
endfunction()
