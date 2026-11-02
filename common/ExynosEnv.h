/** @file
  Windows kernel environment for code shared with UEFI (Exynos5420Pkg
  Drivers/SpiDxe/SpiCore.c, Drivers/Audio/Wm5102Core.c, ExynosI2sCore.c),
  built unchanged: just enough of UEFI's base types and library calls.

  A driver with its own ExynosEnv.h (exynosbatt: the bit-banged I2C bits)
  finds that one first on the include path.  Exactly one file of a driver
  defines OSLOG_GLOBAL before including this (it owns the log ring).

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef __EXYNOS_ENV_H__
#define __EXYNOS_ENV_H__

#include <ntddk.h>
#ifndef OSLOG_GLOBAL
#define OSLOG_EXTERN        // the ring pointer lives in the driver's main file
#endif
#include "oslog.h"

typedef unsigned char       UINT8;
typedef unsigned short      UINT16;
typedef unsigned int        UINT32;
typedef unsigned long long  UINT64;
typedef short               INT16;
typedef int                 INT32;
typedef ULONG_PTR           UINTN;
typedef LONG_PTR            INTN;
typedef char                CHAR8;

#define STATIC    static
#define CONST     const
#define IN
#define OUT
#define OPTIONAL
#define EFIAPI

#define MIN(a, b)       (((a) < (b)) ? (a) : (b))
#define MAX(a, b)       (((a) > (b)) ? (a) : (b))
#define ABS(a)          (((a) < 0) ? -(a) : (a))
#define ARRAY_SIZE(a)   (sizeof (a) / sizeof ((a)[0]))

static __inline UINT32 MmioRead32 (UINTN A)             { return READ_REGISTER_ULONG ((volatile ULONG *)A); }
static __inline VOID   MmioWrite32 (UINTN A, UINT32 V)  { WRITE_REGISTER_ULONG ((volatile ULONG *)A, V); }
static __inline VOID   MmioOr32 (UINTN A, UINT32 V)     { MmioWrite32 (A, MmioRead32 (A) | V); }
static __inline VOID   MmioAnd32 (UINTN A, UINT32 V)    { MmioWrite32 (A, MmioRead32 (A) & V); }
static __inline VOID   MmioAndThenOr32 (UINTN A, UINT32 And, UINT32 Or) { MmioWrite32 (A, (MmioRead32 (A) & And) | Or); }

#define MicroSecondDelay(Us)  KeStallExecutionProcessor ((ULONG)(Us))

static __inline VOID
EnvSleepMs (UINTN Ms)
{
  LARGE_INTEGER  Interval;

  Interval.QuadPart = -(LONGLONG)Ms * 10000;
  KeDelayExecutionThread (KernelMode, FALSE, &Interval);
}

//
// DEBUG ((Level, Format, ...)) goes to the OS log ring, INFO included .  EDK2's %a (ASCII
// string) becomes %s: to the kernel's printf %a is a hex double.
//
#define DEBUG_INFO     0x00000040
#define DEBUG_ERROR    0x80000000
#define DEBUG(Expression)  EnvDebug Expression

static __inline VOID
EnvDebug (ULONG Level, const char *Format, ...)
{
  char     Fmt[200];
  char     Line[200];
  size_t   In;
  size_t   Out;
  size_t   Length;
  va_list  Args;

  UNREFERENCED_PARAMETER (Level);

  for (In = 0, Out = 0; (Format[In] != '\0') && (Out + 3 < sizeof (Fmt)); In++) {
    if ((Format[In] == '%') && (Format[In + 1] == 'a')) {
      Fmt[Out++] = '%';
      Fmt[Out++] = 's';
      In++;
    } else {
      Fmt[Out++] = Format[In];
    }
  }
  Fmt[Out] = '\0';

  Line[0] = '\0';
  OSLOG_VA_START (Args, Format);
  RtlStringCbVPrintfA (Line, sizeof (Line), Fmt, Args);
  OSLOG_VA_END (Args);
  RtlStringCbLengthA (Line, sizeof (Line), &Length);
  OsLogWrite (Line, Length);           // Format carries its own newline
}

#endif // __EXYNOS_ENV_H__
