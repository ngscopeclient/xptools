/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2016 Andrew D. Zonenberg                                                                          *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/**
	@file
	@author Frederic BORRY
	@brief Implementation of HID transport layer
 */

//#include "jtaghal.h"
#include "HID.h"
#include <stdio.h>
#include <memory.h>
#include <cstring>

using namespace std;

/**
	@brief Constructor
 */
HID::HID() : m_handle(NULL)
{
}


/**
	@brief Destructor
 */
HID::~HID() {
	Close();
}

/** @brief Open a HID device using a Vendor ID (VID), Product ID
	(PID) and optionally a serial number.

	If @p serial_number is NULL, the first device with the
	specified VID and PID is opened.

	@ingroup API
	@param vendorId The Vendor ID (VID) of the device to open.
	@param productId The Product ID (PID) of the device to open.
	@param serialNumber The Serial Number of the device to open
							(Optionally NULL).

	@returns
		This function returns a pointer to a #hid_device object on
		success or NULL on failure.
*/
bool HID::Connect(unsigned short vendorId, unsigned short productId, const char* serialNumber)
{
	wchar_t serialWc[128];
	if(serialNumber)
	{
		const size_t cSize = strlen(serialNumber)+1;
		if(cSize>=128)
		{
			LogError("Invalid serial number '%s' (too long)\n", serialNumber);
			return false;
		}
		mbstowcs (serialWc, serialNumber, cSize);
	}

	m_handle = hid_open(vendorId, productId, serialNumber ? serialWc : NULL);
	if (!m_handle) {
		LogError("Could not open HID device %x:%x:%s\n",vendorId,productId,serialNumber);
		hid_exit();
		return false;
	}
	#define MAX_STR 255
	wchar_t wstr[MAX_STR];
	// Read the Manufacturer String
	wstr[0] = 0x0000;
	int res = hid_get_manufacturer_string(m_handle, wstr, MAX_STR);
	if (res < 0)
		LogError("Unable to read manufacturer string\n");
	LogError("Manufacturer String: %ls\n", wstr);

	// Read the Product String
	wstr[0] = 0x0000;
	res = hid_get_product_string(m_handle, wstr, MAX_STR);
	if (res < 0)
		LogError("Unable to read product string\n");
	LogError("Product String: %ls\n", wstr);

	// Read the Serial Number String
	wstr[0] = 0x0000;
	res = hid_get_serial_number_string(m_handle, wstr, MAX_STR);
	if (res < 0)
		LogError("Unable to read serial number string\n");
	LogError("Serial Number String: (%d) %ls\n", wstr[0], wstr);

	return true;
}

/**
	@brief Disconnects from the serial port
 */
void HID::Close()
{
	hid_close(m_handle);
	/* Free static HIDAPI objects. */
	hid_exit();
	m_handle = NULL;
}

bool HID::Read(unsigned char* data, int len)
{
	int x = 0;
	while( (x = hid_read(m_handle, data, len)) > 0)
	{
		len -= x;
		data += x;
		if(len == 0)
			break;
	}

	if(x < 0)
	{
		LogWarning("UART read failed\n");
		return false;
	}
	else if(x == 0)
	{
		//LogWarning("Communication closed unexpectedly\n");
		return false;
	}

	return true;
}

bool HID::Write(const unsigned char* data, int len)
{
	int x = 0;
	while((x = hid_write(	m_handle,   // Handle to the Serial port
							data,     	// Data to be written to the port
							len)) > 0)  //No of bytes to write
	{
		len -= x;
		data += x;
		if(len == 0)
			break;
	}

	if(x < 0)
	{
		LogWarning("HID write failed\n");
		return false;
	}
	else if(x == 0)
	{
		//LogWarning("Communication closed unexpectedly\n");
		return false;
	}

	return true;
}
