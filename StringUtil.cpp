/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2026 Andrew D. Zonenberg and contributors                                                         *
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
	@brief Implementation of string utility functions
 */
#include "StringUtil.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <locale>
#include <codecvt>
#endif

/**
 * @brief Convert a std::wstring into an std::string
 * @param wstr the wstring to convert
 * @return the converted string
 */
std::string WstringToString(const std::wstring &wstr)
{
#ifdef _WIN32
	std::string res;
	if(!wstr.empty())
	{
		int size_needed = WideCharToMultiByte(CP_UTF8, 0, &wstr[0],(int)wstr.size(), NULL, 0, NULL, NULL);
		if (size_needed)
		{
			res = std::string(size_needed, 0);
			WideCharToMultiByte (CP_UTF8, 0, &wstr[0],(int)wstr.size(), &res[0], size_needed, NULL, NULL);
		}
	}
	return res;
#else
    // Linux / macOS
    std::wstring ws(wstr);
    std::wstring_convert<std::codecvt_utf8<wchar_t>> conv;
    return conv.to_bytes(ws);
#endif // _WIN32
}

/**
 * @brief Convert a std::string into an std::wstring
 * @param wstr the string to convert
 * @return the converted wstring
 */
std::wstring StringToWstring(const std::string &str)
{
#ifdef _WIN32
	std::wstring res;
	if( !str.empty())
	{
		int size_needed = MultiByteToWideChar(CP_UTF8, 0, &str[0],(int)str.size(), NULL, 0);
		if (size_needed)
		{
			res = std::wstring(size_needed, 0);
			MultiByteToWideChar(CP_UTF8, 0, &str[0],(int)str.size(), &res[0], size_needed);
		}
	}
	return res;
#else
    // Linux / macOS
    std::wstring_convert<std::codecvt_utf8<wchar_t>> conv;
    return conv.from_bytes(str);
#endif // _WIN32
}
