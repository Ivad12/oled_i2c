# CMAKE minimum version required
cmake_minimum_required(VERSION 3.19)

# import pico SDK
include(pico_sdk_import.cmake)

# set project
project(oled_i2c)

# init pico SDK
pico_sdk_init()

# select main executable/s
add_executable(oled_i2c oled_i2c.c)

# enable USB communication and disable UART
pico_enable_stdio_usb(oled_i2c 1)
pico_enable_stdio_uart(oled_i2c 0)

# add additional target libraries
target_link_libraries(oled_i2c pico_stdlib)

# add any user required libraries
target_link_libraries(oled_i2c
        hardware_i2c
        )

# generate .uf2 file
pico_add_extra_outputs(oled_i2c)