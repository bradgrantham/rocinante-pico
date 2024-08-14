cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rocinante C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_compile_definitions(ROSA=1)

add_executable(rocinante rocinante.c cpp-support.cpp events.cpp hid.cpp rosa/api/key-repeat.cpp rosa/api/text-mode.cpp rosa/api/8x16.cpp rosa/api/ui.cpp rosa/apps/coleco/tms9918.cpp rosa/apps/coleco/emulator.cpp rosa/apps/coleco/coleco_platform_rosa.cpp rosa/apps/coleco/z80emu-cv.c syscalls.c rosa/apps/launcher/launcher.cpp)

target_include_directories(rocinante PRIVATE rosa/api)

pico_generate_pio_header(rocinante ${CMAKE_CURRENT_LIST_DIR}/rocinante.pio)

# pico_enable_stdio_usb(rocinante 1)
# pico_enable_stdio_uart(rocinante 1)

pico_add_extra_outputs(rocinante)

target_link_libraries(rocinante pico_stdlib pico_divider pico_multicore pico_bootsel_via_double_reset hardware_pio hardware_dma hardware_adc hardware_irq hardware_clocks hardware_pll)
