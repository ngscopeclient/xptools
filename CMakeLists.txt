add_library(xptools STATIC
	Socket.cpp
	UART.cpp
	HID.cpp)

find_package(hidapi REQUIRED)
target_link_libraries(xptools hidapi::hidapi)

if(WIN32)
target_link_libraries(xptools ws2_32)
endif()
