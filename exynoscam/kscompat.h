//
// Win32 types the (user + kernel) KS headers expect, missing from the kernel
// headers of this toolchain.  Include after <ntddk.h>, before <ks.h>.
//
#ifndef __KSCOMPAT_H__
#define __KSCOMPAT_H__

typedef int                BOOL;
typedef float              FLOAT;
typedef unsigned long      DWORD;
typedef unsigned short     WORD;
typedef unsigned char      BYTE;
typedef long long          REFERENCE_TIME;
typedef struct tagRECT { LONG left, top, right, bottom; } RECT;
typedef struct tagSIZE { LONG cx, cy; } SIZE;
typedef unsigned int       UINT;
#define MAX_PATH 260
#define WINAPI
#define CALLBACK
#ifndef CDECL
#define CDECL
#endif

#endif
