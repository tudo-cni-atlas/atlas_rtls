cmake_minimum_required(VERSION 2.6)

set(SERIAL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(SERIAL_INCLUDE_DIR ${SERIAL_INCLUDE_DIR} PARENT_SCOPE)

if(TARGET serial)
	message("serial is already defined")
else()
	include_directories(${SERIAL_INCLUDE_DIR})
	set(LIB_SRCS serial.cpp serial.h)
	add_library(serial STATIC ${LIB_SRCS})
endif()