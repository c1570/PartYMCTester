cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)
#include(pico_extras_import_optional.cmake)

project(partymctester C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

#add_subdirectory(pico-ws2812)

add_executable(partymctester
	partymctester.cpp
	pico-ws2812/ws2812.cpp
)

target_link_libraries(partymctester
	hardware_pio
	pico_stdlib
	pico_multicore
	pico_util
	hardware_adc
	hardware_pwm
#	pico_ws2812
)

target_compile_options(partymctester PRIVATE -Wall -save-temps)

pico_generate_pio_header(partymctester ${CMAKE_CURRENT_LIST_DIR}/samplepin.pio)

pico_enable_stdio_usb(partymctester 0)
pico_enable_stdio_uart(partymctester 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(partymctester)
