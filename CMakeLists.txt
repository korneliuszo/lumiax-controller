
set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ===== DO NOT REMOVE =====
cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# ===== DO NOT REMOVE =====

project(controller-fw)

target_sources(app PRIVATE
	src/main.cpp
	src/uart.cpp
	src/bt.c
	src/display.cpp
	src/font.c
)
