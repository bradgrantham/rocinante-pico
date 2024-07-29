cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rocinante C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(rocinante rocinante.c frame.c)

pico_generate_pio_header(rocinante ${CMAKE_CURRENT_LIST_DIR}/rocinante.pio)

pico_enable_stdio_usb(rocinante 1)
pico_enable_stdio_uart(rocinante 1)

pico_add_extra_outputs(rocinante)

target_link_libraries(rocinante pico_stdlib pico_divider pico_multicore pico_bootsel_via_double_reset hardware_pio hardware_dma hardware_adc hardware_irq hardware_clocks hardware_pll)
