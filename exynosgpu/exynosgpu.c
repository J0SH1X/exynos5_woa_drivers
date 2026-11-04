/** @file
  Exynos5420 Mali-T628 MP6 for Windows RT 8.1 (KMDF 1.13): the hardware
  base of the GPU driver.  It powers the GPU through the shared core
  (UEFI Drivers/Gpu/MaliCore.c: vdd_g3d, clock, G3D domain, reset, cores),
  owns the three interrupts (job, MMU, GPU) and runs a self-test at start:
  a NULL job and a WRITE_VALUE job through address space 0, completion
  taken from the job interrupt.

  exynoswddm (the WDDM adapter) uses the GPU through the direct-call
  interface in common/ExGpuIf.h: GPU info, page mapping in address space 0,
  job chains run to completion.  exynosgpu publishes it as a device
  interface and answers IRP_MN_QUERY_INTERFACE for it.

  Resources (DSDT EXYN5013): GPU registers 0x11800000 + 0x5000, the job /
  MMU / GPU interrupts (GSI 251 / 106 / 149), the PMIC (I2C, MAX77802
  BUCK4 = vdd_g3d).

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ExynosEnv.h>
#include <wdf.h>
#define RESHUB_USE_HELPER_ROUTINES
#include <reshub.h>
#include <spb.h>
#include <initguid.h>
#include "../../uefi/Exynos5420Pkg/Drivers/Gpu/MaliCore.h"
#include "../common/ExGpuIf.h"
#include "../common/ExPmicIf.h"

#define GPU_TAG          'uGxE'
#define GPU_MHZ          266
#define GPU_VDD_UV       (862500 + 25000)          // gpu_exynos5420.c 266 MHz OPP + margin
#define BUF_BYTES        (64 * 1024)
#define PT_BYTES         (8 * 1024 * 1024)      // address space 0 tables: the whole 0x10000000..0xF0000000 range
                                                // (bump-allocated, never freed: 2 MiB ran out after ~20k chains)
#define JOB_TIMEOUT_MS   500

//
// Job interrupt registers (panfrost_regs.h), for the ISR.
//
#define JS_COMMAND(n)    (0x1800 + (n) * 0x80 + 0x20)
#define JS_STATUS(n)     (0x1800 + (n) * 0x80 + 0x24)
#define JS_HARD_STOP     3
#define JOB_INT_STAT     0x100C
#define JOB_INT_CLEAR    0x1004
#define JOB_INT_MASK     0x1008
#define MMU_INT_STAT     0x200C
#define MMU_INT_CLEAR    0x2004
#define MMU_INT_MASK     0x2008
#define GPU_INT_STAT     0x002C
#define GPU_INT_CLEAR    0x0024
#define GPU_INT_MASK     0x0028

typedef struct {
  EXYNOS_MALI      Mali;
  WDFIOTARGET      Pmic;               // old DSDT: own PMIC connection
  LARGE_INTEGER    PmicId;
  EX_PMIC_INTERFACE PmicIf;            // new DSDT: through exynospmic
  BOOLEAN          PmicIfOk;
  WDFTIMER         PmicTimer;          // retries the exynospmic connection (vdd_g3d)
  ULONG            PmicTries;
  WDFINTERRUPT     Interrupt[3];       // job, MMU, GPU
  ULONG            InterruptCount;
  volatile LONG    JobDone;            // JOB_INT bits seen
  volatile LONG    MmuFault;
  volatile LONG    GpuIrq;
  KEVENT           JobEvent;
  PUCHAR           Buf;                // GPU-visible buffer (non-cached)
  PHYSICAL_ADDRESS BufPa;
  MALI_PT_POOL     Pool;
  BOOLEAN          Up;
  FAST_MUTEX       Lock;               // GPU work (interface calls, self-test)
  ULONG            Chains;
  ULONG            ChainFaults;
} GPU_CONTEXT, *PGPU_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (GPU_CONTEXT, GetGpu)

DRIVER_INITIALIZE                DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD        EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE  EvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE  EvtReleaseHardware;
EVT_WDF_DEVICE_D0_ENTRY_POST_INTERRUPTS_ENABLED EvtD0EntryPostInterrupts;
EVT_WDF_DEVICE_D0_EXIT_PRE_INTERRUPTS_DISABLED  EvtD0ExitPreInterrupts;
EVT_WDF_INTERRUPT_ISR            EvtIsr;
EVT_WDF_INTERRUPT_DPC            EvtDpc;
EVT_WDF_TIMER                    EvtPmicTimer;

//
// The buffers are non-cached (MmNonCached contiguous memory): nothing to
// clean or invalidate.
//
VOID
EnvCacheClean (
  IN VOID   *Va,
  IN UINTN  Size
  )
{
  UNREFERENCED_PARAMETER (Va);
  UNREFERENCED_PARAMETER (Size);
  __builtin_arm_dmb (15);
}

VOID
EnvCacheInvalidate (
  IN VOID   *Va,
  IN UINTN  Size
  )
{
  UNREFERENCED_PARAMETER (Va);
  UNREFERENCED_PARAMETER (Size);
  __builtin_arm_dmb (15);
}

static NTSTATUS
PmicWrite (PGPU_CONTEXT Ctx, UCHAR Reg, UCHAR Value)
{
  UCHAR                  Buf[2] = { Reg, Value };
  WDF_MEMORY_DESCRIPTOR  In;

  if (Ctx->Pmic == NULL) {
    return Ctx->PmicIf.Write (Ctx->PmicIf.Header.Context, Reg, Value);
  }
  WDF_MEMORY_DESCRIPTOR_INIT_BUFFER (&In, Buf, sizeof (Buf));
  return WdfIoTargetSendWriteSynchronously (Ctx->Pmic, NULL, &In, NULL, NULL, NULL);
}

static NTSTATUS
PmicRead (PGPU_CONTEXT Ctx, UCHAR Reg, PUCHAR Value)
{
  SPB_TRANSFER_LIST_AND_ENTRIES (2) Seq;
  WDF_MEMORY_DESCRIPTOR          In;

  if (Ctx->Pmic == NULL) {
    return Ctx->PmicIf.Read (Ctx->PmicIf.Header.Context, Reg, Value);
  }
  SPB_TRANSFER_LIST_INIT (&Seq.List, 2);
  Seq.List.Transfers[0] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE (SpbTransferDirectionToDevice, 0, &Reg, 1);
  Seq.List.Transfers[1] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE (SpbTransferDirectionFromDevice, 0, Value, 1);
  WDF_MEMORY_DESCRIPTOR_INIT_BUFFER (&In, &Seq, sizeof (Seq));
  return WdfIoTargetSendIoctlSynchronously (Ctx->Pmic, NULL, IOCTL_SPB_EXECUTE_SEQUENCE, &In, NULL, NULL, NULL);
}

/**
  vdd_g3d: all eight BUCK4 DVS registers to the level, the buck on.
**/
static NTSTATUS
SetVddG3d (PGPU_CONTEXT Ctx, ULONG MicroVolts)
{
  UCHAR     Ctrl = 0;
  UCHAR     Level = 0;
  ULONG     Index;
  NTSTATUS  Status;

  Status = PmicRead (Ctx, MALI_PMIC_BUCK4_CTRL1, &Ctrl);
  for (Index = 0; NT_SUCCESS (Status) && (Index < 8); Index++) {
    Status = PmicWrite (Ctx, (UCHAR)(MALI_PMIC_BUCK4_DVS1 + Index), MALI_BUCK4_CODE (MicroVolts));
  }
  if (NT_SUCCESS (Status) && (((Ctrl >> 4) & 3) != 3)) {
    Status = PmicWrite (Ctx, MALI_PMIC_BUCK4_CTRL1, (UCHAR)(Ctrl | (3u << 4)));
  }
  KeStallExecutionProcessor (2000);
  if (NT_SUCCESS (Status)) {
    Status = PmicRead (Ctx, MALI_PMIC_BUCK4_DVS1, &Level);
  }
  OsLog ("exynosgpu: vdd_g3d %lu mV (BUCK4 CTRL1 0x%02x) - 0x%08lx", (600000 + Level * 6250) / 1000, Ctrl, Status);
  return Status;
}

//
// ---------------------------------------------------------------------------
// Interrupts: the job interrupt completes jobs; MMU / GPU ones are faults.
// ---------------------------------------------------------------------------
//

BOOLEAN
EvtIsr (WDFINTERRUPT Interrupt, ULONG MessageId)
{
  PGPU_CONTEXT  Ctx = GetGpu (WdfInterruptGetDevice (Interrupt));
  UINTN         Gpu = Ctx->Mali.Gpu;
  ULONG         Stat;
  BOOLEAN       Mine = FALSE;

  UNREFERENCED_PARAMETER (MessageId);

  if (!Ctx->Up) {
    return FALSE;
  }
  Stat = MmioRead32 (Gpu + JOB_INT_STAT);
  if (Stat != 0) {
    MmioWrite32 (Gpu + JOB_INT_CLEAR, Stat);
    InterlockedOr (&Ctx->JobDone, (LONG)Stat);
    Mine = TRUE;
  }
  Stat = MmioRead32 (Gpu + MMU_INT_STAT);
  if (Stat != 0) {
    MmioWrite32 (Gpu + MMU_INT_MASK, 0);          // left to the DPC to log, then cleared
    InterlockedOr (&Ctx->MmuFault, (LONG)Stat);
    Mine = TRUE;
  }
  Stat = MmioRead32 (Gpu + GPU_INT_STAT);
  if (Stat != 0) {
    MmioWrite32 (Gpu + GPU_INT_CLEAR, Stat);
    InterlockedOr (&Ctx->GpuIrq, (LONG)Stat);
    Mine = TRUE;
  }
  if (Mine) {
    WdfInterruptQueueDpcForIsr (Interrupt);
  }
  return Mine;
}

VOID
EvtDpc (WDFINTERRUPT Interrupt, WDFOBJECT Device)
{
  PGPU_CONTEXT  Ctx = GetGpu (Device);

  UNREFERENCED_PARAMETER (Interrupt);

  if (Ctx->MmuFault != 0) {
    OsLog ("exynosgpu: MMU interrupt 0x%08lx", Ctx->MmuFault);
    MaliLogFault (&Ctx->Mali, 0, 0);
    MmioWrite32 (Ctx->Mali.Gpu + MMU_INT_CLEAR, 0xFFFFFFFF);
    MmioWrite32 (Ctx->Mali.Gpu + MMU_INT_MASK, 0xFFFFFFFF);
    InterlockedExchange (&Ctx->MmuFault, 0);
  }
  if (Ctx->GpuIrq != 0) {
    OsLog ("exynosgpu: GPU interrupt 0x%08lx", InterlockedExchange (&Ctx->GpuIrq, 0));
  }
  if (Ctx->JobDone != 0) {
    KeSetEvent (&Ctx->JobEvent, IO_NO_INCREMENT, FALSE);
  }
}

/**
  One chain on a job slot, address space 0; done or failed from the job
  interrupt (JOB_INT bit n done, bit 16 + n failed).
**/
static LONG
RunChain (PGPU_CONTEXT Ctx, PCSTR Name, ULONG Slot, ULONGLONG ChainVa)
{
  LARGE_INTEGER  Timeout;
  LONG           Bits;

  InterlockedExchange (&Ctx->JobDone, 0);
  KeClearEvent (&Ctx->JobEvent);
  if (MaliJobStart (&Ctx->Mali, Slot, 0, ChainVa) != 0) {
    OsLog ("exynosgpu: %s: slot busy", Name);
    return -1;
  }
  Timeout.QuadPart = -10000LL * JOB_TIMEOUT_MS;
  KeWaitForSingleObject (&Ctx->JobEvent, Executive, KernelMode, FALSE, &Timeout);
  Bits = InterlockedExchange (&Ctx->JobDone, 0);
  OsLog ("exynosgpu: %s: %s (JOB_INT 0x%08lx)", Name,
         (Bits & (0x10000 << Slot)) ? "FAULT" : (Bits & (1 << Slot)) ? "done" : "TIMEOUT", Bits);
  if ((Bits & (1 << Slot)) == 0) {
    MaliLogFault (&Ctx->Mali, Slot, 0);
  }
  return Bits;
}

static VOID
SelfTest (PGPU_CONTEXT Ctx)
{
  ULONGLONG              Va = (ULONGLONG)Ctx->BufPa.QuadPart;      // mapped 1:1
  MALI_JOB_HEADER        *Null = (MALI_JOB_HEADER *)(Ctx->Buf + 0x000);
  MALI_WRITE_VALUE_JOB   *Write = (MALI_WRITE_VALUE_JOB *)(Ctx->Buf + 0x040);
  volatile ULONG         *Result = (volatile ULONG *)(Ctx->Buf + 0x1000);

  MaliBuildNullJob (Null, 1, 0);
  RunChain (Ctx, "NULL job", MALI_SLOT_OTHER, Va);

  // T62x WRITE_VALUE has no immediates: "zero" over a preset, then a timestamp.
  Result[0] = 0xFFFFFFFF;
  Result[1] = 0xFFFFFFFF;
  MaliBuildWriteValueJob (Write, 1, 0, Va + 0x1000, MALI_WRITE_VALUE_ZERO);
  RunChain (Ctx, "WRITE_VALUE zero", MALI_SLOT_OTHER, Va + 0x040);
  OsLog ("exynosgpu: result 0x%08lx%08lx (%s)", Result[1], Result[0], ((Result[0] | Result[1]) == 0) ? "GPU wrote it" : "unchanged");
  MaliBuildWriteValueJob (Write, 1, 0, Va + 0x1000, MALI_WRITE_VALUE_TIMESTAMP);
  RunChain (Ctx, "WRITE_VALUE timestamp", MALI_SLOT_OTHER, Va + 0x040);
  OsLog ("exynosgpu: timestamp 0x%08lx%08lx", Result[1], Result[0]);

  //
  // Clear-only render pass (fragment job, slot 0) into a 256 x 32 buffer at
  // +0x4000 (polygon list at +0x2000, framebuffer descriptor at +0x100).
  //
  {
    MALI_FRAMEBUFFER   *Fbd  = (MALI_FRAMEBUFFER *)(Ctx->Buf + 0x100);
    MALI_FRAGMENT_JOB  *Frag = (MALI_FRAGMENT_JOB *)(Ctx->Buf + 0x280);
    volatile ULONG     *Px   = (volatile ULONG *)(Ctx->Buf + 0x4000);
    ULONG              Index;
    ULONG              Good;

    for (Index = 0; Index < 256 * 32; Index++) {
      Px[Index] = 0x01010101;
    }
    RtlZeroMemory (Ctx->Buf + 0x2000, MALI_POLYGON_LIST_BYTES);
    MaliBuildClearJob (Frag, Fbd, Va + 0x100, Va + 0x2000, Va + 0x4000, 256, 32, 256 * 4, 0xFF2080E0);
    RunChain (Ctx, "clear 256x32", MALI_SLOT_FRAGMENT, Va + 0x280);
    for (Index = 0, Good = 0; Index < 256 * 32; Index++) {
      Good += (Px[Index] == 0xFF2080E0) ? 1 : 0;
    }
    OsLog ("exynosgpu: clear: %lu of %lu pixels (first 0x%08lx)", Good, 256UL * 32, Px[0]);
  }
}

//
// ---------------------------------------------------------------------------
// Interface for exynoswddm (common/ExGpuIf.h)
// ---------------------------------------------------------------------------
//

static VOID
IfGetInfo (PVOID Context, EX_GPU_INFO *Info)
{
  PGPU_CONTEXT  Ctx = (PGPU_CONTEXT)Context;
  UINTN         Gpu = Ctx->Mali.Gpu;
  ULONG         N;

  RtlZeroMemory (Info, sizeof (*Info));
  Info->Up = Ctx->Up;
  if (!Ctx->Up) {
    return;
  }
  Info->GpuId         = Ctx->Mali.GpuId;
  Info->L2Features    = Ctx->Mali.L2Features;
  Info->CoreFeatures  = Ctx->Mali.CoreFeatures;
  Info->TilerFeatures = Ctx->Mali.TilerFeatures;
  Info->MemFeatures   = Ctx->Mali.MemFeatures;
  Info->MmuFeatures   = Ctx->Mali.MmuFeatures;
  Info->AsPresent     = Ctx->Mali.AsPresent;
  Info->JsPresent     = Ctx->Mali.JsPresent;
  for (N = 0; N < 16; N++) {
    Info->JsFeatures[N] = MmioRead32 (Gpu + 0xC0 + N * 4);
  }
  for (N = 0; N < 4; N++) {
    Info->TextureFeatures[N] = MmioRead32 (Gpu + 0xB0 + N * 4);
  }
  Info->ThreadMaxThreads       = MmioRead32 (Gpu + 0xA0);
  Info->ThreadMaxWorkgroupSize = MmioRead32 (Gpu + 0xA4);
  Info->ThreadMaxBarrierSize   = MmioRead32 (Gpu + 0xA8);
  Info->ThreadFeatures         = MmioRead32 (Gpu + 0xAC);
  Info->ShaderPresent = Ctx->Mali.ShaderPresent;
  Info->TilerPresent  = Ctx->Mali.TilerPresent;
  Info->L2Present     = Ctx->Mali.L2Present;
  Info->StackPresent  = MmioRead32 (Gpu + 0xE00) | ((ULONGLONG)MmioRead32 (Gpu + 0xE04) << 32);
  Info->ClockMhz      = Ctx->Mali.ClockHz / 1000000;
}

static NTSTATUS
IfMapPages (PVOID Context, ULONGLONG GpuVa, CONST PFN_NUMBER *Pfn, ULONG Pages)
{
  PGPU_CONTEXT  Ctx = (PGPU_CONTEXT)Context;
  INTN          Result;

  if ((GpuVa < EX_GPU_VA_START) || (GpuVa + (ULONGLONG)Pages * PAGE_SIZE > EX_GPU_VA_END)) {
    return STATUS_INVALID_PARAMETER;
  }
  ExAcquireFastMutex (&Ctx->Lock);
  Result = MaliPtMapPages (&Ctx->Pool, GpuVa, (CONST UINTN *)Pfn, Pages);
  if (Ctx->Up) {
    MaliAsFlush (&Ctx->Mali, 0, GpuVa, (ULONGLONG)Pages * PAGE_SIZE);
  }
  ExReleaseFastMutex (&Ctx->Lock);
  if (Result != 0) {
    OsLog ("exynosgpu: page tables full mapping %lu pages at 0x%08lx (%lu KiB used)", Pages, (ULONG)GpuVa,
           (ULONG)(Ctx->Pool.Used >> 10));
    return STATUS_INSUFFICIENT_RESOURCES;
  }
  return STATUS_SUCCESS;
}

static VOID
IfUnmapPages (PVOID Context, ULONGLONG GpuVa, ULONG Pages)
{
  PGPU_CONTEXT  Ctx = (PGPU_CONTEXT)Context;

  if ((GpuVa < EX_GPU_VA_START) || (GpuVa + (ULONGLONG)Pages * PAGE_SIZE > EX_GPU_VA_END)) {
    return;
  }
  ExAcquireFastMutex (&Ctx->Lock);
  MaliPtUnmap (&Ctx->Pool, GpuVa, Pages);
  if (Ctx->Up) {
    MaliAsFlush (&Ctx->Mali, 0, GpuVa, (ULONGLONG)Pages * PAGE_SIZE);
  }
  ExReleaseFastMutex (&Ctx->Lock);
}

/**
  One chain to completion: JS_STATUS (1 = done), or EX_GPU_TIMEOUT after
  a hard stop of the slot.
**/
static ULONG
IfRunChain (PVOID Context, ULONG Slot, ULONGLONG Chain, ULONG TimeoutMs)
{
  PGPU_CONTEXT   Ctx = (PGPU_CONTEXT)Context;
  LARGE_INTEGER  Timeout;
  LONG           Bits;
  ULONG          Status;

  if (Slot > 2) {
    return EX_GPU_TIMEOUT;
  }
  ExAcquireFastMutex (&Ctx->Lock);
  if (!Ctx->Up) {
    ExReleaseFastMutex (&Ctx->Lock);
    return EX_GPU_TIMEOUT;
  }
  InterlockedExchange (&Ctx->JobDone, 0);
  KeClearEvent (&Ctx->JobEvent);
  if (MaliJobStart (&Ctx->Mali, Slot, 0, Chain) != 0) {
    ExReleaseFastMutex (&Ctx->Lock);
    return EX_GPU_TIMEOUT;
  }
  Timeout.QuadPart = -10000LL * TimeoutMs;
  for (;;) {
    if (KeWaitForSingleObject (&Ctx->JobEvent, Executive, KernelMode, FALSE, &Timeout) == STATUS_TIMEOUT) {
      Bits = 0;
      break;
    }
    Bits = InterlockedExchange (&Ctx->JobDone, 0);
    if (Bits & ((1 << Slot) | (0x10000 << Slot))) {
      break;
    }
    KeClearEvent (&Ctx->JobEvent);   // another slot's bit: keep waiting
  }
  Ctx->Chains++;
  if (Bits & (1 << Slot)) {
    Status = 1;
  } else if (Bits & (0x10000 << Slot)) {
    Status = MmioRead32 (Ctx->Mali.Gpu + JS_STATUS (Slot));
  } else {
    MmioWrite32 (Ctx->Mali.Gpu + JS_COMMAND (Slot), JS_HARD_STOP);
    Status = EX_GPU_TIMEOUT;
  }
  if ((Status != 1) && (Ctx->ChainFaults++ < 20)) {
    OsLog ("exynosgpu: chain 0x%08lx on slot %lu: JS_STATUS 0x%08lx (%lu chains)", (ULONG)Chain, Slot, Status,
           Ctx->Chains);
    MaliLogFault (&Ctx->Mali, Slot, 0);
  }
  ExReleaseFastMutex (&Ctx->Lock);
  return Status;
}

static VOID
IfReference (PVOID Context)
{
  UNREFERENCED_PARAMETER (Context);
}

//
// ---------------------------------------------------------------------------
// PnP / power
// ---------------------------------------------------------------------------
//

static PVOID
MapPhys (ULONG Address, SIZE_T Size)
{
  PHYSICAL_ADDRESS  Pa;

  Pa.QuadPart = Address;
  return MmMapIoSpace (Pa, Size, MmNonCached);
}

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST Raw, WDFCMRESLIST Translated)
{
  PGPU_CONTEXT                     Ctx = GetGpu (Device);
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  WDF_IO_TARGET_OPEN_PARAMS        Open;
  PHYSICAL_ADDRESS                 Low;
  PHYSICAL_ADDRESS                 High;
  PHYSICAL_ADDRESS                 Boundary;
  ULONG                            Index;
  NTSTATUS                         Status;

  DECLARE_UNICODE_STRING_SIZE (Path, RESOURCE_HUB_PATH_SIZE);

  UNREFERENCED_PARAMETER (Raw);

  for (Index = 0; Index < WdfCmResourceListGetCount (Translated); Index++) {
    Desc = WdfCmResourceListGetDescriptor (Translated, Index);
    if ((Desc->Type == CmResourceTypeMemory) && (Ctx->Mali.Gpu == 0)) {
      Ctx->Mali.Gpu = (UINTN)MmMapIoSpace (Desc->u.Memory.Start, 0x5000, MmNonCached);
    } else if ((Desc->Type == CmResourceTypeConnection) &&
               (Desc->u.Connection.Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL) &&
               (Desc->u.Connection.Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C)) {
      Ctx->PmicId.LowPart  = Desc->u.Connection.IdLowPart;
      Ctx->PmicId.HighPart = Desc->u.Connection.IdHighPart;
    }
  }
  Ctx->Mali.Cmu = (UINTN)MapPhys (MALI_CMU_TOP_PHYS, 0x1000);
  Ctx->Mali.Pmu = (UINTN)MapPhys (MALI_PMU_G3D_PHYS, 0x1000);
  if ((Ctx->Mali.Gpu == 0) || (Ctx->Mali.Cmu == 0) || (Ctx->Mali.Pmu == 0)) {
    OsLog ("exynosgpu: resources missing (GPU %p, PMIC 0x%08lx)", (PVOID)Ctx->Mali.Gpu, Ctx->PmicId.LowPart);
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }

  if (Ctx->PmicId.QuadPart == 0) {
    OsLog ("exynosgpu: no PMIC connection: vdd_g3d through exynospmic");
    goto Memory;
  }
  Status = RESOURCE_HUB_CREATE_PATH_FROM_ID (&Path, Ctx->PmicId.LowPart, Ctx->PmicId.HighPart);
  if (NT_SUCCESS (Status) && (Ctx->Pmic == NULL)) {
    Status = WdfIoTargetCreate (Device, WDF_NO_OBJECT_ATTRIBUTES, &Ctx->Pmic);
    if (NT_SUCCESS (Status)) {
      WDF_IO_TARGET_OPEN_PARAMS_INIT_OPEN_BY_NAME (&Open, &Path, GENERIC_READ | GENERIC_WRITE);
      Status = WdfIoTargetOpen (Ctx->Pmic, &Open);
    }
  }
  OsLog ("exynosgpu: PMIC open - 0x%08lx", Status);
  if (!NT_SUCCESS (Status)) {
    //
    // The PMIC connection is exclusive and another driver (mxt1664s, key
    // backlight LDO) may hold it.  vdd_g3d then stays as the bootloader
    // left it (BUCK4 1025 mV, enough for GPU_MHZ).
    //
    if (Ctx->Pmic != NULL) {
      WdfObjectDelete (Ctx->Pmic);
      Ctx->Pmic = NULL;
    }
    OsLog ("exynosgpu: no PMIC, vdd_g3d left as set by the bootloader");
  }

Memory:
  //
  // GPU memory: non-cached, contiguous, below 4 GiB (1:1 GPU mapping).
  //
  Low.QuadPart      = 0;
  High.QuadPart     = 0xFFFFFFFF;
  Boundary.QuadPart = 0;
  if (Ctx->Buf == NULL) {
    Ctx->Buf = MmAllocateContiguousMemorySpecifyCache (BUF_BYTES, Low, High, Boundary, MmNonCached);
    Ctx->Pool.Va = MmAllocateContiguousMemorySpecifyCache (PT_BYTES, Low, High, Boundary, MmNonCached);
    if ((Ctx->Buf == NULL) || (Ctx->Pool.Va == NULL)) {
      return STATUS_INSUFFICIENT_RESOURCES;
    }
    Ctx->BufPa     = MmGetPhysicalAddress (Ctx->Buf);
    Ctx->Pool.Pa   = (UINT64)MmGetPhysicalAddress (Ctx->Pool.Va).QuadPart;
    Ctx->Pool.Size = PT_BYTES;
    RtlZeroMemory (Ctx->Buf, BUF_BYTES);
  }
  return STATUS_SUCCESS;
}

/**
  exynospmic is a separate driver and may start after the GPU: until it is
  there the GPU runs at the bootloader's vdd_g3d; then the level goes in.
**/
VOID
EvtPmicTimer (WDFTIMER Timer)
{
  PGPU_CONTEXT  Ctx = GetGpu (WdfTimerGetParentObject (Timer));

  if (Ctx->PmicIfOk || !Ctx->Up) {
    return;
  }
  Ctx->PmicIfOk = ExPmicConnect (&Ctx->PmicIf);
  if (!Ctx->PmicIfOk) {
    if (++Ctx->PmicTries < 60) {
      WdfTimerStart (Ctx->PmicTimer, WDF_REL_TIMEOUT_IN_MS (2000));
    }
    return;
  }
  OsLog ("exynosgpu: exynospmic connected after %lu tries", Ctx->PmicTries);
  ExAcquireFastMutex (&Ctx->Lock);
  SetVddG3d (Ctx, GPU_VDD_UV);
  ExReleaseFastMutex (&Ctx->Lock);
}

NTSTATUS
EvtD0EntryPostInterrupts (WDFDEVICE Device, WDF_POWER_DEVICE_STATE PreviousState)
{
  PGPU_CONTEXT  Ctx = GetGpu (Device);

  UNREFERENCED_PARAMETER (PreviousState);

  if ((Ctx->Pmic == NULL) && !Ctx->PmicIfOk && (Ctx->PmicId.QuadPart == 0)) {
    Ctx->PmicIfOk = ExPmicConnect (&Ctx->PmicIf);
    OsLog ("exynosgpu: exynospmic %s", Ctx->PmicIfOk ? "connected" : "not there yet: vdd_g3d as set by the bootloader until it is");
    if (!Ctx->PmicIfOk) {
      Ctx->PmicTries = 0;
      WdfTimerStart (Ctx->PmicTimer, WDF_REL_TIMEOUT_IN_MS (2000));
    }
  }
  if (((Ctx->Pmic != NULL) || Ctx->PmicIfOk) && !NT_SUCCESS (SetVddG3d (Ctx, GPU_VDD_UV))) {
    return STATUS_DEVICE_POWER_FAILURE;
  }
  if (MaliPowerUp (&Ctx->Mali, GPU_MHZ) != 0) {
    OsLog ("exynosgpu: power-up failed at step %lu", Ctx->Mali.Step);
    MaliLogState (&Ctx->Mali);
    MaliPowerDown (&Ctx->Mali);
    return STATUS_DEVICE_POWER_FAILURE;
  }
  MaliLogState (&Ctx->Mali);

  if ((MaliPtInit (&Ctx->Pool) != 0) ||
      (MaliPtMap (&Ctx->Pool, (UINT64)Ctx->BufPa.QuadPart, (UINT64)Ctx->BufPa.QuadPart, BUF_BYTES) != 0) ||
      (MaliAsEnable (&Ctx->Mali, 0, &Ctx->Pool) != 0)) {
    OsLog ("exynosgpu: address space 0 setup failed");
    MaliLogFault (&Ctx->Mali, 0, 0);
  }
  //
  // Interrupts on: job done / failed on every slot, MMU faults, GPU faults.
  //
  Ctx->Up = TRUE;
  MmioWrite32 (Ctx->Mali.Gpu + JOB_INT_MASK, 0xFFFFFFFF);
  MmioWrite32 (Ctx->Mali.Gpu + MMU_INT_MASK, 0xFFFFFFFF);
  MmioWrite32 (Ctx->Mali.Gpu + GPU_INT_MASK, 0x81);              // fault, multiple fault
  SelfTest (Ctx);
  //
  // The self-test buffer was mapped 1:1; it leaves address space 0 so the
  // range belongs to exynoswddm alone.
  //
  MaliPtUnmap (&Ctx->Pool, (UINT64)Ctx->BufPa.QuadPart, BUF_BYTES / PAGE_SIZE);
  MaliAsFlush (&Ctx->Mali, 0, (UINT64)Ctx->BufPa.QuadPart, BUF_BYTES);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtD0ExitPreInterrupts (WDFDEVICE Device, WDF_POWER_DEVICE_STATE TargetState)
{
  PGPU_CONTEXT  Ctx = GetGpu (Device);

  UNREFERENCED_PARAMETER (TargetState);

  WdfTimerStop (Ctx->PmicTimer, TRUE);
  if (Ctx->Up) {
    Ctx->Up = FALSE;
    MaliAsDisable (&Ctx->Mali, 0);
    MaliPowerDown (&Ctx->Mali);
    OsLog ("exynosgpu: GPU off");
  }
  return STATUS_SUCCESS;
}

NTSTATUS
EvtReleaseHardware (WDFDEVICE Device, WDFCMRESLIST Translated)
{
  PGPU_CONTEXT  Ctx = GetGpu (Device);

  UNREFERENCED_PARAMETER (Translated);

  if (Ctx->Mali.Gpu != 0) { MmUnmapIoSpace ((PVOID)Ctx->Mali.Gpu, 0x5000); Ctx->Mali.Gpu = 0; }
  if (Ctx->Mali.Cmu != 0) { MmUnmapIoSpace ((PVOID)Ctx->Mali.Cmu, 0x1000); Ctx->Mali.Cmu = 0; }
  if (Ctx->Mali.Pmu != 0) { MmUnmapIoSpace ((PVOID)Ctx->Mali.Pmu, 0x1000); Ctx->Mali.Pmu = 0; }
  if (Ctx->Buf != NULL) { MmFreeContiguousMemorySpecifyCache (Ctx->Buf, BUF_BYTES, MmNonCached); Ctx->Buf = NULL; }
  if (Ctx->Pool.Va != NULL) { MmFreeContiguousMemorySpecifyCache (Ctx->Pool.Va, PT_BYTES, MmNonCached); Ctx->Pool.Va = NULL; }
  return STATUS_SUCCESS;
}

NTSTATUS
EvtDeviceAdd (WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
  WDF_PNPPOWER_EVENT_CALLBACKS  PnpPower;
  WDF_OBJECT_ATTRIBUTES         Attributes;
  WDF_INTERRUPT_CONFIG          InterruptConfig;
  WDFDEVICE                     Device;
  PGPU_CONTEXT                  Ctx;
  ULONG                         Index;
  NTSTATUS                      Status;

  UNREFERENCED_PARAMETER (Driver);

  WDF_PNPPOWER_EVENT_CALLBACKS_INIT (&PnpPower);
  PnpPower.EvtDevicePrepareHardware                = EvtPrepareHardware;
  PnpPower.EvtDeviceReleaseHardware                = EvtReleaseHardware;
  PnpPower.EvtDeviceD0EntryPostInterruptsEnabled   = EvtD0EntryPostInterrupts;
  PnpPower.EvtDeviceD0ExitPreInterruptsDisabled    = EvtD0ExitPreInterrupts;
  WdfDeviceInitSetPnpPowerEventCallbacks (DeviceInit, &PnpPower);

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, GPU_CONTEXT);
  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Ctx = GetGpu (Device);
  KeInitializeEvent (&Ctx->JobEvent, NotificationEvent, FALSE);
  ExInitializeFastMutex (&Ctx->Lock);
  {
    WDF_TIMER_CONFIG       TimerConfig;
    WDF_OBJECT_ATTRIBUTES  TimerAttributes;

    WDF_TIMER_CONFIG_INIT (&TimerConfig, EvtPmicTimer);
    TimerConfig.AutomaticSerialization = FALSE;
    WDF_OBJECT_ATTRIBUTES_INIT (&TimerAttributes);
    TimerAttributes.ParentObject    = Device;
    TimerAttributes.ExecutionLevel  = WdfExecutionLevelPassive;
    Status = WdfTimerCreate (&TimerConfig, &TimerAttributes, &Ctx->PmicTimer);
    if (!NT_SUCCESS (Status)) {
      return Status;
    }
  }

  //
  // The direct-call interface for exynoswddm.
  //
  {
    WDF_QUERY_INTERFACE_CONFIG  Qi;
    static EX_GPU_INTERFACE     If;

    RtlZeroMemory (&If, sizeof (If));
    If.Header.Size                 = sizeof (If);
    If.Header.Version              = EX_GPU_IF_VERSION;
    If.Header.Context              = Ctx;
    If.Header.InterfaceReference   = IfReference;
    If.Header.InterfaceDereference = IfReference;
    If.GetInfo                     = IfGetInfo;
    If.MapPages                    = IfMapPages;
    If.UnmapPages                  = IfUnmapPages;
    If.RunChain                    = IfRunChain;
    WDF_QUERY_INTERFACE_CONFIG_INIT (&Qi, (PINTERFACE)&If, &GUID_EXYNOS_GPU_INTERFACE, NULL);
    Status = WdfDeviceAddQueryInterface (Device, &Qi);
    if (NT_SUCCESS (Status)) {
      Status = WdfDeviceCreateDeviceInterface (Device, &GUID_EXYNOS_GPU_INTERFACE, NULL);
    }
    OsLog ("exynosgpu: interface for exynoswddm - 0x%08lx", Status);
    if (!NT_SUCCESS (Status)) {
      return Status;
    }
  }

  //
  // Job, MMU, GPU interrupts, in DSDT order; one ISR checks all three
  // status registers.
  //
  for (Index = 0; Index < 3; Index++) {
    WDF_INTERRUPT_CONFIG_INIT (&InterruptConfig, EvtIsr, EvtDpc);
    Status = WdfInterruptCreate (Device, &InterruptConfig, WDF_NO_OBJECT_ATTRIBUTES, &Ctx->Interrupt[Index]);
    if (!NT_SUCCESS (Status)) {
      OsLog ("exynosgpu: interrupt %lu - 0x%08lx", Index, Status);
      return Status;
    }
  }
  OsLog ("exynosgpu: device added");
  return STATUS_SUCCESS;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("exynosgpu: DriverEntry (built " __DATE__ " " __TIME__ ")");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
