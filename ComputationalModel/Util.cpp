/* Copyright (c) <2009> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

// stdafx.cpp : source file that includes just the standard includes
// NewView.pch will be the pre-compiled header
// stdafx.obj will contain the pre-compiled type information


#include <Util.h>

// TODO: reference any additional headers you need in STDAFX.H
// and not in this file


// Windows user assets path
void GetWorkingFileName (const char* const name, char* const outPathName)
{
#if defined (_MSC_VER)

    //GetAplicationDirectory (appPath);
		char appPath [256];
		GetModuleFileNameA(NULL, appPath, sizeof (appPath));
		strlwr (appPath);

		char* const end = strstr (appPath, "applications");
		end [0] = 0;
		sprintf (outPathName, "%sapplications/media/%s", appPath, name);

#elif defined(__APPLE__)
    char tmp[2048];
    CFURLRef appURL (CFBundleCopyBundleURL(CFBundleGetMainBundle()));
    CFStringRef filePath (CFURLCopyFileSystemPath (appURL, kCFURLPOSIXPathStyle));
    CFStringGetCString (filePath, tmp, PATH_MAX, kCFStringEncodingUTF8);
    //char* const ptr = strstr (tmp, "applications");
    //ptr [0] = 0;
    //sprintf (outPathName, "%sapplications/media/%s", tmp, name);
    sprintf (outPathName, "%s/Contents/Resources/%s", tmp, name);

    // Clean up
    CFRelease( appURL );
    CFRelease( filePath );

#elif (defined (_POSIX_VER) || defined (_POSIX_VER_64))

    char id[2048];
		char appPath[2048];

		sprintf(id, "/proc/%d/exe", getpid());
		memset (appPath, 0, sizeof (appPath));
		readlink(id, appPath, sizeof (appPath));
		char* const end = strstr (appPath, "applications");
		*end = 0;
		sprintf (outPathName, "%sapplications/media/%s", appPath, name);

#endif
}

// little Indian/big Indian conversion
#ifdef __ppc__
unsigned short SWAP_INT16(unsigned short x)
	{
		return ((x >> 8) & 0xff) + ((x & 0xff) << 8);
	}
	unsigned SWAP_INT32(unsigned x)
	{
		return SWAP_INT16 ( x >> 16) + (SWAP_INT16 (x) << 16);
	}


	void SWAP_FLOAT32_ARRAY (void* const array, dInt32 count)
	{
		dInt32* const ptr = (dInt32*) array;
		count /= sizeof (dInt32);
		for (dInt32 i = 0; i < count; i ++) {
			dInt32 x;
			x = SWAP_INT32 (ptr[i]);
			ptr[i] = x;
		}
	}

#else

unsigned SWAP_INT32(unsigned x)
{
    return x;
}

unsigned short SWAP_INT16(unsigned short x)
{
    return x;
}

void SWAP_FLOAT32_ARRAY (void* const array, dInt32 count)
{
}
#endif

void PrintTime(){
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
    std::string str(buffer);

    std::cout << str;
}
