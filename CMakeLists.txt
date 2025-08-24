add_library(xptools STATIC
	Socket.cpp
	UART.cpp
	HID.cpp
	TimeUtil.cpp)

find_package(PkgConfig MODULE REQUIRED)

pkg_check_modules(HIDAPI IMPORTED_TARGET hidapi)
if(NOT HIDAPI_FOUND)
	pkg_check_modules(HIDAPI IMPORTED_TARGET hidapi-hidraw) # look for older version too
endif()
if(NOT HIDAPI_FOUND)
	message(FATAL_ERROR "Unable to find any version of hidapi or hidapi-hidraw; this is required to build xptools (part of ngscopeclient).")
endif()

target_include_directories(xptools SYSTEM PUBLIC ${HIDAPI_INCLUDE_DIRS})
target_link_libraries(xptools PUBLIC PkgConfig::HIDAPI)

if(WIN32)
	target_link_libraries(xptools ws2_32)
endif()
