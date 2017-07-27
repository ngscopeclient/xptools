add_library(xptools STATIC
	Socket.cpp
	UART.cpp)

if(WIN32)
target_link_libraries(xptools ws2_32)
endif()
