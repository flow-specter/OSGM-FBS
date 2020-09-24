#ifndef _BASE_TYPEDEF_H
#define _BASE_TYPEDEF_H

#include <stdlib.h>


#ifdef WIN32
#include <tchar.h>
#else
	#if defined(_UNICODE) || defined(UNICODE)
		#include <wchar.h>
		#define _T(x) L##x
	#else
		#define _T(x) x
	#endif

    // ignore case
	#define stricmp strcasecmp

	#if defined(_UNICODE) || defined(UNICODE)
		#define _tcschr     wcschr
		#define _tcsstr     wcsstr

		// 实际的字符数，不是字节数
		#define _tcsclen    wcslen
		#define _tcsicmp	wcsicmp
		#define _tcscmp		wcscmp
	#else
		#define _tcschr     strchr
		#define _tcsstr     strstr

		// 实际的字符数，不是字节数
		#define _tcsclen    strlen
		#define _tcsicmp	stricmp
		#define _tcscmp		strcmp
	#endif
#endif

#if defined(_UNICODE) || defined(UNICODE)
	typedef wchar_t				Char;

	// 字符数，是字节数的一半
	#define	StrLen			wcslen
#else
	typedef char				gfChar;

	// 按英文计算的字符数，中文算两个，就是实际的字节数
	#define	gfStrLen			strlen
#endif



#if defined(WIN32) || defined(_WIN32)
	#define PLATFORM_32
	#define PLATFORM_WINDOWS
#elif defined(WIN64) || defined(_WIN64)
	#define PLATFORM_64
	#define PLATFORM_WINDOWS
#elif defined(LINUX32) || defined(_LINUX32)
    #define PLATFORM_32
	#define PLATFORM_LINUX
#elif defined(LINUX64) || defined(_LINUX64)
	#define PLATFORM_64
    #define PLATFORM_LINUX
#else
	#error "Platform not Specified!"
#endif

#ifndef PLATFORM_WINDOWS
	typedef unsigned int UINT;
	typedef unsigned char BYTE;
	#define stricmp strcasecmp
#endif

#include <float.h>


#ifndef NOMINMAX

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#endif  /* NOMINMAX */


#ifndef interface
#define interface struct
#endif


#ifndef ZERO
#define ZERO 1e-6
#endif
#ifndef NULL
#define NULL 0
#endif


#ifndef PI
#define  PI 3.1415926535897932384626433832795
#endif

#ifndef GM
#define    GM 3.9860047e+14
#endif


#ifndef BYTE
typedef unsigned char BYTE;
#endif

#ifndef SWIG
	#ifdef PLATFORM_WINDOWS
		#define HIP_EXPORT __declspec(dllexport)
		#define HIP_IMPORT __declspec(dllimport)
		#define HIP_HIDDEN
	#else
		#define HIP_EXPORT __attribute__ ((visibility ("default")))
		#define HIP_IMPORT __attribute__ ((visibility ("hidden")))
		#define HIP_HIDDEN
	#endif
#else
	#define HIP_EXPORT
	#define HIP_IMPORT
	#define HIP_HIDDEN
#endif


typedef enum 

{
	CE1_IMAGE_SENSOR,
	CE2_IMAGE_SENSOR,
	CE2_RESAMPLE_MODEL,
    HRSC_IMAGE_SENSOR,
	HRSC_RESAMPLE_MODEL,
    HIRISE_IMAGE_SENSOR,
	HIRSIE_RESAMPLE_MODEL,
    LRONAC_IMAGE_SENSOR,
    MOC_IMAGE_SENSOR,
	LROWAC_IMAGE_SENSOR
} OrbiterSensorType;

#endif




