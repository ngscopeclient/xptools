add_library(xptools STATIC
	Socket.cpp
	UART.cpp
	HID.cpp)

target_link_libraries(xptools hidapi)

if(WIN32)
target_link_libraries(xptools ws2_32)
endif()
