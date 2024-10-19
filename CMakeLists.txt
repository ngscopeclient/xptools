add_library(xptools STATIC
	Socket.cpp
	UART.cpp
	HID.cpp)

if(UNIX AND NOT APPLE)
	# hidapi-config.cmake not provided by ubuntu usbapi package
	target_link_libraries(xptools hidapi-hidraw)
else()
	find_package(hidapi REQUIRED)
	target_link_libraries(xptools hidapi::hidapi)
endif()

if(WIN32)
target_link_libraries(xptools ws2_32)
endif()
