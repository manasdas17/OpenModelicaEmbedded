set(CMAKE_BUILD_TYPE Release)

cmake_minimum_required(VERSION 2.8)

if(WIN32)
	add_definitions(-DWINDOWS)
elseif(UNIX AND NOT APPLE)
	add_definitions(-DLINUX)
elseif(APPLE)
	add_definitions(-DMACOSX)
	SET( CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -framework CoreFoundation -framework IOKit" )
else()
	message(SEND_ERROR "Undefined platform")
endif()



set(SRC modelPlugFirmata.cpp modelPlugFirmata.h serial.cpp serial.h)

add_library(modelPlugFirmata SHARED ${SRC})
