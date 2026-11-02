/** @file
  Windows kernel environment for the code shared with UEFI: the DesignWare
  MMC host (Exynos5420Pkg Drivers/SdCardDxe: MmcHost.h, Dwmci.c) and the
  Broadcom SDIO core (Drivers/BcmWlan/BcmSdio.c), built unchanged.

  Like exynosemmc/MmcEnv.h, plus what the Broadcom core uses, and with
  DEBUG_INFO kept (the Wi-Fi bring-up logs through it).  Everything runs at
  PASSIVE_LEVEL in the driver's own thread (or MiniportInitializeEx), so
  waits of a millisecond or more sleep instead of spinning.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef __MMC_ENV_H__
#define __MMC_ENV_H__

#include <ntddk.h>
#ifndef OSLOG_GLOBAL
#define OSLOG_EXTERN        // the ring pointer lives in exynoswlan.c
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
typedef UINT64              EFI_LBA;
typedef UINTN               EFI_STATUS;

#define STATIC    static
#define CONST     const
#define IN
#define OUT
#define OPTIONAL
#define EFIAPI

#define EFI_ENCODE_ERROR(x)    ((EFI_STATUS)(0x80000000u | (x)))
#define EFI_SUCCESS            ((EFI_STATUS)0)
#define EFI_INVALID_PARAMETER  EFI_ENCODE_ERROR (2)
#define EFI_UNSUPPORTED        EFI_ENCODE_ERROR (3)
#define EFI_BAD_BUFFER_SIZE    EFI_ENCODE_ERROR (4)
#define EFI_NOT_READY          EFI_ENCODE_ERROR (6)
#define EFI_DEVICE_ERROR       EFI_ENCODE_ERROR (7)
#define EFI_OUT_OF_RESOURCES   EFI_ENCODE_ERROR (9)
#define EFI_NO_MEDIA           EFI_ENCODE_ERROR (12)
#define EFI_NOT_FOUND          EFI_ENCODE_ERROR (14)
#define EFI_TIMEOUT            EFI_ENCODE_ERROR (18)
#define EFI_CRC_ERROR          EFI_ENCODE_ERROR (27)
#define EFI_ERROR(s)           (((INTN)(s)) < 0)

#define EFI_PAGE_SIZE          PAGE_SIZE
#define EFI_SIZE_TO_PAGES(n)   (((n) + PAGE_SIZE - 1) / PAGE_SIZE)

#ifndef MIN
#define MIN(a, b)       (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b)       (((a) > (b)) ? (a) : (b))
#endif
#define ARRAY_SIZE(a)   (sizeof (a) / sizeof ((a)[0]))
#define ZeroMem(p, n)            RtlZeroMemory ((p), (n))
#define CopyMem(d, s, n)         RtlCopyMemory ((d), (s), (n))
#define SetMem(p, n, v)          RtlFillMemory ((p), (n), (UCHAR)(v))

//
// EDK2 semantics (sign of the first differing byte), not RtlCompareMemory's.
//
static __inline INTN
CompareMem (const VOID *A, const VOID *B, UINTN Length)
{
  const UINT8  *P = A;
  const UINT8  *Q = B;

  for (; Length > 0; Length--, P++, Q++) {
    if (*P != *Q) {
      return (INTN)*P - (INTN)*Q;
    }
  }
  return 0;
}
#define InvalidateDataCacheRange(p, n)  ((VOID)(p), (VOID)(n))
#define WriteBackDataCacheRange(p, n)   ((VOID)(p), (VOID)(n))

static __inline UINT32 MmioRead32 (UINTN A)             { return READ_REGISTER_ULONG ((volatile ULONG *)A); }
static __inline VOID   MmioWrite32 (UINTN A, UINT32 V)  { WRITE_REGISTER_ULONG ((volatile ULONG *)A, V); }
static __inline VOID   MmioOr32 (UINTN A, UINT32 V)     { MmioWrite32 (A, MmioRead32 (A) | V); }
static __inline VOID   MmioAnd32 (UINTN A, UINT32 V)    { MmioWrite32 (A, MmioRead32 (A) & V); }

static __inline VOID *
DmaAllocatePages (UINTN Pages)
{
  PHYSICAL_ADDRESS  Low;
  PHYSICAL_ADDRESS  High;
  PHYSICAL_ADDRESS  Skip;

  Low.QuadPart  = 0;
  High.QuadPart = 0xFFFFFFFF;
  Skip.QuadPart = 0;
  return MmAllocateContiguousMemorySpecifyCache (Pages * PAGE_SIZE, Low, High, Skip, MmNonCached);
}

#define DMA_ADDRESS(Va)  ((UINT32)MmGetPhysicalAddress ((PVOID)(Va)).LowPart)

//
// gBS->Stall (Microseconds): short waits spin, a millisecond or more sleeps.
//
typedef struct {
  VOID  (*Stall)(UINTN Microseconds);
} EFI_SHIM_BOOT_SERVICES;

static VOID
EfiShimStall (UINTN Microseconds)
{
  LARGE_INTEGER  Interval;

  if ((Microseconds >= 1000) && (KeGetCurrentIrql () == PASSIVE_LEVEL)) {
    Interval.QuadPart = -(LONGLONG)Microseconds * 10;
    KeDelayExecutionThread (KernelMode, FALSE, &Interval);
  } else {
    KeStallExecutionProcessor ((ULONG)Microseconds);
  }
}
static EFI_SHIM_BOOT_SERVICES  mEfiShimBs = { EfiShimStall };

#define GetPerformanceCounter()  ((UINT64)KeQueryPerformanceCounter (NULL).QuadPart)
#define gBS  (&mEfiShimBs)

//
// DEBUG ((Level, Format, ...)) to the OS log ring.  EDK2's %a (ASCII), %r
// (EFI_STATUS) and %l (64-bit) become %s, %lx and %I64.
//
#define DEBUG_INFO     0x00000040
#define DEBUG_VERBOSE  0x00400000
#define DEBUG_ERROR    0x80000000
#define DEBUG(Expression)  EfiShimDebug Expression

static __inline VOID
EfiShimDebug (ULONG Level, const char *Format, ...)
{
  char     Fmt[200];
  char     Line[200];
  size_t   In;
  size_t   Out;
  size_t   Length;
  va_list  Args;

  if ((Level & (DEBUG_ERROR | DEBUG_INFO)) == 0) {
    return;
  }
  for (In = 0, Out = 0; (Format[In] != '\0') && (Out + 5 < sizeof (Fmt)); In++) {
    if ((Format[In] == '%') && (Format[In + 1] == 'a')) {
      Fmt[Out++] = '%';
      Fmt[Out++] = 's';
      In++;
    } else if ((Format[In] == '%') && (Format[In + 1] == 'l')) {
      Fmt[Out++] = '%';
      Fmt[Out++] = 'I';
      Fmt[Out++] = '6';
      Fmt[Out++] = '4';
      In++;
    } else if ((Format[In] == '%') && (Format[In + 1] == 'r')) {
      Fmt[Out++] = '%';
      Fmt[Out++] = 'l';
      Fmt[Out++] = 'x';
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

#endif // __MMC_ENV_H__
