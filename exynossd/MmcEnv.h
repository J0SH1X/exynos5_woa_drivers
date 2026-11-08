/** @file
  Windows kernel environment for the shared MMC code (Exynos5420Pkg
  Drivers/SdCardDxe: MmcHost.h, MmcCore.c, Dwmci.c, built unchanged).

  Just enough of UEFI's base types and library calls.  Pointers are 32-bit
  on ARM, so a host's UINT32 Base holds the mapped virtual address of its
  registers.  The DMA pool is physically contiguous and uncached, so the
  cache maintenance calls are no-ops.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef __MMC_ENV_H__
#define __MMC_ENV_H__

#include <ntddk.h>
#ifndef OSLOG_GLOBAL
#define OSLOG_EXTERN        // the ring pointer lives in exynossd.c
#endif
#include "oslog.h"

typedef unsigned char       UINT8;
typedef unsigned short      UINT16;
typedef unsigned int        UINT32;
typedef unsigned long long  UINT64;
typedef ULONG_PTR           UINTN;
typedef LONG_PTR            INTN;
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
#define EFI_DEVICE_ERROR       EFI_ENCODE_ERROR (7)
#define EFI_OUT_OF_RESOURCES   EFI_ENCODE_ERROR (9)
#define EFI_NO_MEDIA           EFI_ENCODE_ERROR (12)
#define EFI_TIMEOUT            EFI_ENCODE_ERROR (18)
#define EFI_UNSUPPORTED        EFI_ENCODE_ERROR (3)
#define EFI_CRC_ERROR          EFI_ENCODE_ERROR (27)
#define EFI_ERROR(s)           (((INTN)(s)) < 0)

#define EFI_PAGE_SIZE          PAGE_SIZE
#define EFI_SIZE_TO_PAGES(n)   (((n) + PAGE_SIZE - 1) / PAGE_SIZE)

#define MIN(a, b)       (((a) < (b)) ? (a) : (b))
#define ARRAY_SIZE(a)   (sizeof (a) / sizeof ((a)[0]))
#define ZeroMem(p, n)            RtlZeroMemory ((p), (n))
#define CopyMem(d, s, n)         RtlCopyMemory ((d), (s), (n))
#define SetMem(p, n, v)          RtlFillMemory ((p), (n), (UCHAR)(v))
#define InvalidateDataCacheRange(p, n)  ((VOID)(p), (VOID)(n))
#define WriteBackDataCacheRange(p, n)   ((VOID)(p), (VOID)(n))

static __inline UINT32 MmioRead32 (UINTN A)             { return READ_REGISTER_ULONG ((volatile ULONG *)A); }
static __inline VOID   MmioWrite32 (UINTN A, UINT32 V)  { WRITE_REGISTER_ULONG ((volatile ULONG *)A, V); }
static __inline VOID   MmioOr32 (UINTN A, UINT32 V)     { MmioWrite32 (A, MmioRead32 (A) | V); }
static __inline VOID   MmioAnd32 (UINTN A, UINT32 V)    { MmioWrite32 (A, MmioRead32 (A) & V); }

//
// DMA pool: physically contiguous, uncached, below 4 GiB (32-bit IDMAC).
//
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
// gBS->Stall (Microseconds): busy wait (PASSIVE level in the Storport worker
// or during HwFindAdapter).
//
typedef struct {
  VOID  (*Stall)(UINTN Microseconds);
} EFI_SHIM_BOOT_SERVICES;

static VOID EfiShimStall (UINTN Microseconds) { KeStallExecutionProcessor ((ULONG)Microseconds); }
static EFI_SHIM_BOOT_SERVICES  mEfiShimBs = { EfiShimStall };
#define gBS  (&mEfiShimBs)

//
// DEBUG ((Level, Format, ...)) goes to the OS log ring.  EDK2's %a (ASCII
// string), %r (EFI_STATUS) and %l (64-bit) become %s, %lx and %I64: to the
// kernel's printf %a is a hex double and %l 32-bit, either of which would
// read the wrong arguments.  DEBUG_INFO is
// dropped (the UEFI build shows it), DEBUG_ERROR kept.
//
#define DEBUG_INFO     0x00000040
#define DEBUG_VERBOSE  0x00400000
#define DEBUG_ERROR    0x80000000
#define DEBUG(Expression)  EfiShimDebug Expression

static __inline VOID
EfiShimDebug (ULONG Level, const char *Format, ...)
{
  char     Fmt[160];
  char     Line[160];
  size_t   In;
  size_t   Out;
  size_t   Length;
  va_list  Args;

  if ((Level & DEBUG_ERROR) == 0) {
    return;
  }

  for (In = 0, Out = 0; (Format[In] != '\0') && (Out + 3 < sizeof (Fmt)); In++) {
    if ((Format[In] == '%') && (Format[In + 1] == 'a')) {
      Fmt[Out++] = '%';
      Fmt[Out++] = 's';
      In++;
    } else if ((Format[In] == '%') && (Format[In + 1] == 'l') && (Out + 5 < sizeof (Fmt))) {
      Fmt[Out++] = '%';                // EDK2 %l = 64-bit, kernel printf: %I64
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
