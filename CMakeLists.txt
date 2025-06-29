add_library(xptools STATIC
	Socket.cpp
	UART.cpp
	HID.cpp
	TimeUtil.cpp)

find_package(PkgConfig MODULE REQUIRED)

pkg_check_modules(HIDAPI QUIET IMPORTED_TARGET hidapi-hidraw) # look for older version too
if(NOT HIDAPI_FOUND)
	message(FATAL_ERROR "Unable to find any version of hidapi-hidraw; this is required to build xptools (part of ngscopeclient).")
endif()

target_include_directories(xptools SYSTEM PRIVATE ${HIDAPI_INCLUDE_DIRS})
target_link_libraries(xptools PRIVATE ${HIDAPI_LIBRARIES})

if(WIN32)
target_link_libraries(xptools ws2_32)
endif()
