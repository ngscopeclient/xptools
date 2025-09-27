add_library(xptools STATIC
	Socket.cpp
	UART.cpp
	HID.cpp
	TimeUtil.cpp)

if(UNIX)
	include(CheckStructHasMember)
	check_struct_has_member(
		termios
		c_ispeed
		termios.h
		TERMIOS_HAS_ISPEED
		LANGUAGE CXX)
	if(NOT TERMIOS_HAS_ISPEED)
		target_compile_definitions(xptools PRIVATE "USE_TERMIOS2")
	endif()
endif()

find_package(PkgConfig MODULE REQUIRED)

pkg_check_modules(HIDAPI IMPORTED_TARGET hidapi)
if(NOT HIDAPI_FOUND)
	pkg_check_modules(HIDAPI IMPORTED_TARGET hidapi-hidraw) # look for older version too
endif()
if(NOT HIDAPI_FOUND)
	message(FATAL_ERROR "Unable to find any version of hidapi or hidapi-hidraw; this is required to build xptools (part of ngscopeclient).")
endif()

target_include_directories(xptools SYSTEM PUBLIC ${HIDAPI_INCLUDE_DIRS})

if(WIN32)
	target_link_libraries(xptools PUBLIC PkgConfig::HIDAPI ws2_32)
else()
	target_link_libraries(xptools PUBLIC PkgConfig::HIDAPI)
endif()
