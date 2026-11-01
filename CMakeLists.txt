cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(turing_sequencer C CXX ASM)

set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(workshopFetichist
    main.cpp
    usb_descriptors.c
    usb_midi_host.c
    usb_midi_host_app_driver.c
)

pico_set_binary_type(workshopFetichist copy_to_ram)

target_include_directories(workshopFetichist PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(workshopFetichist PUBLIC
    PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64
    PICO_STDIO_USB_USE_DEFAULT_DESCRIPTORS=0
)

target_compile_options(workshopFetichist PRIVATE
    -Wdouble-promotion
    -Wfloat-conversion
    -Wall
    -Wextra
    -O3
)

target_link_options(workshopFetichist PRIVATE
    -Wl,--print-memory-usage
)

target_link_libraries(workshopFetichist
    pico_unique_id
    pico_stdlib
    hardware_dma
    hardware_i2c
    hardware_pwm
    hardware_adc
    hardware_spi
    hardware_timer
    hardware_flash
    hardware_sync
    hardware_vreg
    hardware_clocks
    pico_multicore
    tinyusb_device
    tinyusb_host
    tinyusb_board
)

pico_enable_stdio_uart(workshopFetichist 0)
pico_enable_stdio_usb(workshopFetichist 0)

pico_add_extra_outputs(workshopFetichist)