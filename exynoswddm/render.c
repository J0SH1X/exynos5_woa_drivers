/** @file
  exynoswddm render side (WDDM 1.2 full driver, milestone 2: the aperture
  model the Mesa user-mode driver expects, ExMw.h).

    - one aperture segment (APERTURE_BASE, nominal): allocations are system
      memory pages; on DXGK_OPERATION_MAP_APERTURE_SEGMENT their pages are
      mapped at the allocation's fixed GPU address (exynosgpu's address
      space 0, common/ExGpuIf.h) and, on first CPU use, write-combined into
      the kernel (presents, flips).  Map / unmap / fill run on the CPU while
      the paging buffer is built, so paging buffers stay empty;
    - DMA buffers in system memory: EX_CMD commands (ExCmd.h) - presents
      (blt / colour fill between allocations), GPU job chains from the
      UMD's render command buffers (EXMW_COMMAND);
    - SubmitCommand queues the range to a worker thread that runs it in
      submission order (job chains through exynosgpu, to completion) and
      reports DMA_COMPLETED for its fence;
    - flips: the primary is copied into the back one of two contiguous
      scan-out buffers (firmware carve-out) and FIMD1 flips to it at
      vsync; blts into the primary on screen also update the front buffer;
    - escapes: EXMW_ESCAPE_* (allocation GPU address, GPU parameters,
      counters for the UMD's idle wait, scan-out, caps).

  EXW_GPU_MODE 0: the UMD is told "present only" (GPU_INFO Count != 0,
  Valid == 0): it renders on the CPU (softpipe) and presents through the
  runtime; buffers still get GPU addresses and mappings when exynosgpu is
  there.  1: GPU parameters for panfrost.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include "exynoswddm.h"
#include "../../uefi/Exynos5420Pkg/Include/ExynosGpuCarveout.h"

#ifndef EXW_GPU_MODE
#define EXW_GPU_MODE   0
#endif

#define CARVEOUT_SKIP  0x10000          // header page(s) kept out
#define TAG_ALLOC      'lAxE'
#define TAG_DEVICE     'vDxE'
#define TAG_CONTEXT    'tCxE'
#define TAG_VA         'aVxE'

#define EX_DEVICE_MAGIC   0x56445845u     // 'EXDV'
#define EX_CONTEXT_MAGIC  0x58435845u     // 'EXCX'

typedef struct {
  ULONG    Magic;
  ADAPTER  *A;
  BOOLEAN  System;
} EX_DEVICE;

typedef struct {
  ULONG      Magic;
  EX_DEVICE  *Dev;
  ULONG      Node;
} EX_CONTEXT;

/**
  The adapter behind a Render / Present handle: a context (MultiEngineAware)
  or, should dxgkrnl ever hand us a device, that.
**/
static ADAPTER *
AdapterOf (HANDLE h)
{
  if (*(ULONG *)h == EX_CONTEXT_MAGIC) {
    return ((EX_CONTEXT *)h)->Dev->A;
  }
  return ((EX_DEVICE *)h)->A;
}

static ULONG  mRenderLogs;

#define RLOG(...)  do { if (mRenderLogs < 2000) { mRenderLogs++; OsLog (__VA_ARGS__); } } while (0)

static ULONG
SurfacePitch (ULONG Width)
{
  return (Width * 4 + 63) & ~63u;
}

//
// ---------------------------------------------------------------------------
// exynosgpu
// ---------------------------------------------------------------------------
//

static VOID GpuMap (ADAPTER *A, EX_ALLOCATION *X);

/**
  Find exynosgpu's interface (PASSIVE_LEVEL).  exynosgpu may start after
  us: retried by the callers until found.
**/
static BOOLEAN
GpuConnect (ADAPTER *A)
{
  PWSTR              Links = NULL;
  UNICODE_STRING     Name;
  PFILE_OBJECT       File;
  PDEVICE_OBJECT     Device;
  PDEVICE_OBJECT     Top;
  KEVENT             Event;
  IO_STATUS_BLOCK    Iosb;
  PIRP               Irp;
  PIO_STACK_LOCATION Stack;
  NTSTATUS           Status;

  if (A->GpuOk) {
    return TRUE;
  }
  if ((KeGetCurrentIrql () != PASSIVE_LEVEL) || (A->GpuTries > 1000)) {
    return FALSE;
  }
  A->GpuTries++;
  Status = IoGetDeviceInterfaces (&GUID_EXYNOS_GPU_INTERFACE, NULL, 0, &Links);
  if (!NT_SUCCESS (Status) || (Links == NULL) || (Links[0] == 0)) {
    if (Links != NULL) {
      ExFreePool (Links);
    }
    if ((A->GpuTries == 1) || ((A->GpuTries % 100) == 0)) {
      OsLog ("exynoswddm: exynosgpu not there yet (try %lu, 0x%08lx)", A->GpuTries, Status);
    }
    return FALSE;
  }
  RtlInitUnicodeString (&Name, Links);
  Status = IoGetDeviceObjectPointer (&Name, FILE_READ_DATA, &File, &Device);
  ExFreePool (Links);
  if (!NT_SUCCESS (Status)) {
    return FALSE;
  }
  Top = IoGetAttachedDeviceReference (Device);
  KeInitializeEvent (&Event, NotificationEvent, FALSE);
  Irp = IoBuildSynchronousFsdRequest (IRP_MJ_PNP, Top, NULL, 0, NULL, &Event, &Iosb);
  if (Irp != NULL) {
    Irp->IoStatus.Status = STATUS_NOT_SUPPORTED;
    Stack = IoGetNextIrpStackLocation (Irp);
    Stack->MinorFunction                                   = IRP_MN_QUERY_INTERFACE;
    Stack->Parameters.QueryInterface.InterfaceType         = &GUID_EXYNOS_GPU_INTERFACE;
    Stack->Parameters.QueryInterface.Size                  = sizeof (A->Gpu);
    Stack->Parameters.QueryInterface.Version               = EX_GPU_IF_VERSION;
    Stack->Parameters.QueryInterface.Interface             = (PINTERFACE)&A->Gpu;
    Stack->Parameters.QueryInterface.InterfaceSpecificData = NULL;
    Status = IoCallDriver (Top, Irp);
    if (Status == STATUS_PENDING) {
      KeWaitForSingleObject (&Event, Executive, KernelMode, FALSE, NULL);
      Status = Iosb.Status;
    }
  } else {
    Status = STATUS_INSUFFICIENT_RESOURCES;
  }
  ObDereferenceObject (Top);
  ObDereferenceObject (File);
  if (!NT_SUCCESS (Status) || (A->Gpu.RunChain == NULL)) {
    OsLog ("exynoswddm: exynosgpu interface - 0x%08lx", Status);
    return FALSE;
  }
  A->GpuOk = TRUE;
  {
    EX_GPU_INFO  Info;

    A->Gpu.GetInfo (A->Gpu.Header.Context, &Info);
    OsLog ("exynoswddm: exynosgpu connected: GPU_ID 0x%08lx, up %lu, shaders 0x%lx", Info.GpuId, Info.Up,
           (ULONG)Info.ShaderPresent);
  }
  //
  // Allocations made resident before exynosgpu was there: map them now
  // (one at a time, the list lock released around each map).
  //
  for (;;) {
    KIRQL          Irql;
    PLIST_ENTRY    E;
    EX_ALLOCATION  *Todo = NULL;

    KeAcquireSpinLock (&A->AllocLock, &Irql);
    for (E = A->Allocs.Flink; E != &A->Allocs; E = E->Flink) {
      EX_ALLOCATION  *X = CONTAINING_RECORD (E, EX_ALLOCATION, Link);

      if ((X->Mdl != NULL) && !X->GpuMapped && !X->MapTried) {
        X->MapTried = TRUE;
        Todo = X;
        break;
      }
    }
    KeReleaseSpinLock (&A->AllocLock, Irql);
    if (Todo == NULL) {
      break;
    }
    GpuMap (A, Todo);
  }
  return TRUE;
}

static VOID
GpuMap (ADAPTER *A, EX_ALLOCATION *X)
{
  NTSTATUS  Status;
  ULONG     Pages;

  if ((X->Mdl == NULL) || X->GpuMapped || (X->GpuVa == 0) || !GpuConnect (A)) {
    return;
  }
  Pages  = min (X->Pages, (ULONG)(MmGetMdlByteCount (X->Mdl) >> PAGE_SHIFT) - X->MdlPage);
  Status = A->Gpu.MapPages (A->Gpu.Header.Context, X->GpuVa, MmGetMdlPfnArray (X->Mdl) + X->MdlPage, Pages);
  X->GpuMapped = NT_SUCCESS (Status);
  A->Maps++;
  if (!X->GpuMapped) {
    A->MapFails++;
  }
}

/**
  Allocation pages First .. First + Count - 1 (= MDL pages) at their GPU
  address.
**/
static VOID
GpuMapRange (ADAPTER *A, EX_ALLOCATION *X, ULONG First, ULONG Count)
{
  NTSTATUS  Status;

  if ((X->Mdl == NULL) || (X->GpuVa == 0) || (Count == 0) || !GpuConnect (A)) {
    return;
  }
  Status = A->Gpu.MapPages (A->Gpu.Header.Context, X->GpuVa + (ULONGLONG)First * PAGE_SIZE,
                            MmGetMdlPfnArray (X->Mdl) + First, Count);
  A->Maps++;
  if (NT_SUCCESS (Status)) {
    X->GpuMapped = TRUE;
  } else {
    A->MapFails++;
  }
}

static VOID
GpuUnmap (ADAPTER *A, EX_ALLOCATION *X)
{
  if (X->GpuMapped && A->GpuOk) {
    A->Gpu.UnmapPages (A->Gpu.Header.Context, X->GpuVa, X->Pages);
  }
  X->GpuMapped = FALSE;
  X->MapTried  = FALSE;
}

//
// ---------------------------------------------------------------------------
// CPU access to allocations (write-combined, as VidMm maps them for Lock:
// the segment is not cache coherent)
// ---------------------------------------------------------------------------
//

static PUCHAR
CpuVa (EX_ALLOCATION *X)
{
  ULONG  Bytes;
  PUCHAR Start;

  if (X->CpuVa != NULL) {
    return X->CpuVa;
  }
  if (X->Mdl == NULL) {
    return NULL;
  }
  Bytes = X->Pages << PAGE_SHIFT;
  Start = (PUCHAR)MmGetMdlVirtualAddress (X->Mdl) + ((SIZE_T)X->MdlPage << PAGE_SHIFT);
  X->CpuMdl = IoAllocateMdl (Start, Bytes, FALSE, FALSE, NULL);
  if (X->CpuMdl == NULL) {
    return NULL;
  }
  IoBuildPartialMdl (X->Mdl, X->CpuMdl, Start, Bytes);
  X->CpuVa = MmMapLockedPagesSpecifyCache (X->CpuMdl, KernelMode, MmWriteCombined, NULL, FALSE,
                                           NormalPagePriority | MdlMappingNoExecute);
  if (X->CpuVa == NULL) {
    IoFreeMdl (X->CpuMdl);
    X->CpuMdl = NULL;
  }
  return X->CpuVa;
}

static VOID
CpuUnmap (EX_ALLOCATION *X)
{
  if (X->CpuVa != NULL) {
    MmUnmapLockedPages (X->CpuVa, X->CpuMdl);
    X->CpuVa = NULL;
  }
  if (X->CpuMdl != NULL) {
    IoFreeMdl (X->CpuMdl);
    X->CpuMdl = NULL;
  }
}

static EX_ALLOCATION *
AllocBySegAddress (ADAPTER *A, ULONGLONG Address)
{
  KIRQL          Irql;
  PLIST_ENTRY    E;
  EX_ALLOCATION  *Found = NULL;

  KeAcquireSpinLock (&A->AllocLock, &Irql);
  for (E = A->Allocs.Flink; E != &A->Allocs; E = E->Flink) {
    EX_ALLOCATION  *X = CONTAINING_RECORD (E, EX_ALLOCATION, Link);

    if ((X->Mdl != NULL) && (X->SegAddress == Address)) {
      Found = X;
      break;
    }
  }
  KeReleaseSpinLock (&A->AllocLock, Irql);
  return Found;
}

//
// ---------------------------------------------------------------------------
// Caps, segment
// ---------------------------------------------------------------------------
//

PUCHAR
ScanoutVa (ADAPTER *A)
{
  if (A->ScanFront < 2) {
    return A->ScanBuf[A->ScanFront].Va;
  }
  return (A->ScanPa == A->Boot.Pa.LowPart) ? A->Boot.Va : A->Fb;
}

VOID
RenderFillCaps (ADAPTER *A, DXGK_DRIVERCAPS *Caps)
{
  UNREFERENCED_PARAMETER (A);

  Caps->HighestAcceptableAddress.QuadPart = 0xFFFFFFFF;
  Caps->MaxAllocationListSlotId           = 4096;
  Caps->ApertureSegmentCommitLimit        = (SIZE_T)APERTURE_BYTES;
  Caps->InterruptMessageNumber            = 0;
  Caps->NumberOfSwizzlingRanges           = 0;
  Caps->MaxOverlays                       = 0;
  Caps->MaxQueuedFlipOnVSync              = 1;
  Caps->FlipCaps.FlipOnVSyncMmIo          = 1;
  //
  // Contexts (DxgkDdiCreateContext): without this dxgkrnl runs the legacy
  // one-implicit-context-per-device model and the UMD's pfnCreateContextCb
  // fails with E_INVALIDARG (seen on RT 8.1).
  //
  Caps->SchedulingCaps.MultiEngineAware   = 1;
  Caps->GpuEngineTopology.NbAsymetricProcessingNodes = 1;
  Caps->PreemptionCaps.GraphicsPreemptionGranularity = D3DKMDT_GRAPHICS_PREEMPTION_DMA_BUFFER_BOUNDARY;
  Caps->PreemptionCaps.ComputePreemptionGranularity  = D3DKMDT_COMPUTE_PREEMPTION_DMA_BUFFER_BOUNDARY;
  Caps->PresentationCaps.NoScreenToScreenBlt = 1;
  Caps->PresentationCaps.NoOverlapScreenBlt  = 1;
}

static VOID
FillSegment (DXGK_SEGMENTFLAGS *Flags, PHYSICAL_ADDRESS *Base, PHYSICAL_ADDRESS *Cpu, SIZE_T *Size, SIZE_T *Commit)
{
  Base->QuadPart = (LONGLONG)APERTURE_BASE;
  Cpu->QuadPart  = 0;
  *Size          = (SIZE_T)APERTURE_BYTES;
  *Commit        = (SIZE_T)APERTURE_BYTES;
  Flags->Value          = 0;
  Flags->Aperture       = 1;
  Flags->CpuVisible     = 1;    // dxgkrnl refuses a shared primary otherwise
  Flags->CacheCoherent  = 0;    // the Mali does not snoop the CPU caches
}

NTSTATUS
RenderQueryAdapterInfo (ADAPTER *A, CONST DXGKARG_QUERYADAPTERINFO *Q)
{
  UNREFERENCED_PARAMETER (A);

  switch (Q->Type) {
    case DXGKQAITYPE_QUERYSEGMENT: {
      DXGK_QUERYSEGMENTOUT  *Out = (DXGK_QUERYSEGMENTOUT *)Q->pOutputData;

      if (Q->OutputDataSize < sizeof (*Out)) {
        return STATUS_BUFFER_TOO_SMALL;
      }
      if (Out->pSegmentDescriptor != NULL) {
        DXGK_SEGMENTDESCRIPTOR  *D = Out->pSegmentDescriptor;

        RtlZeroMemory (D, sizeof (*D));
        FillSegment (&D->Flags, &D->BaseAddress, &D->CpuTranslatedAddress, &D->Size, &D->CommitLimit);
        Out->PagingBufferSegmentId       = APERTURE_ID;
        Out->PagingBufferSize            = 10 * PAGE_SIZE;
        Out->PagingBufferPrivateDataSize = 0;
      }
      Out->NbSegment = 1;
      return STATUS_SUCCESS;
    }
    case DXGKQAITYPE_QUERYSEGMENT3: {
      DXGK_QUERYSEGMENTOUT3  *Out = (DXGK_QUERYSEGMENTOUT3 *)Q->pOutputData;

      if (Q->OutputDataSize < sizeof (*Out)) {
        return STATUS_BUFFER_TOO_SMALL;
      }
      if (Out->pSegmentDescriptor != NULL) {
        DXGK_SEGMENTDESCRIPTOR3  *D = Out->pSegmentDescriptor;

        RtlZeroMemory (D, sizeof (*D));
        FillSegment (&D->Flags, &D->BaseAddress, &D->CpuTranslatedAddress, &D->Size, &D->CommitLimit);
        Out->PagingBufferSegmentId       = APERTURE_ID;
        Out->PagingBufferSize            = 10 * PAGE_SIZE;
        Out->PagingBufferPrivateDataSize = 0;
      }
      Out->NbSegment = 1;
      return STATUS_SUCCESS;
    }
    case DXGKQAITYPE_UMDRIVERPRIVATE: {
      EX_UMD_CAPS  *Out = (EX_UMD_CAPS *)Q->pOutputData;

      if ((Out == NULL) || (Q->OutputDataSize < sizeof (*Out))) {
        return STATUS_BUFFER_TOO_SMALL;
      }
      RtlZeroMemory (Out, sizeof (*Out));
      Out->Magic         = EX_UMD_MAGIC;
      Out->Version       = EX_UMD_VERSION;
      Out->DmaBufferSize = EX_DMA_BUFFER_SIZE;
      Out->MaxRects      = EX_MAX_RECTS;
      return STATUS_SUCCESS;
    }
    default:
      return STATUS_NOT_SUPPORTED;
  }
}

//
// ---------------------------------------------------------------------------
// Execution (worker thread, PASSIVE_LEVEL)
// ---------------------------------------------------------------------------
//

/**
  The primary on screen changed under a blt: the same rectangle into the
  front scan-out buffer.
**/
static VOID
BltToFront (ADAPTER *A, CONST UCHAR *Src, ULONG SrcPitch, CONST EX_RECT *R)
{
  PUCHAR  Front;
  LONG    Y;
  LONG    Left = max (R->Left, 0);
  LONG    Right = min (R->Right, (LONG)WIDTH);

  if ((A->ScanAllocBuf >= 2) || (Right <= Left)) {
    return;
  }
  Front = A->ScanBuf[A->ScanAllocBuf].Va;     // the buffer this primary was flipped to (latched or not yet)
  for (Y = max (R->Top, 0); Y < min (R->Bottom, (LONG)HEIGHT); Y++) {
    RtlCopyMemory (Front + (SIZE_T)Y * WIDTH * 4 + Left * 4, Src + (SIZE_T)Y * SrcPitch + Left * 4,
                   (SIZE_T)(Right - Left) * 4);
  }
}

static VOID
ExecBlt (ADAPTER *A, CONST EX_CMD_BLT_ARGS *B, BOOLEAN Fill)
{
  EX_ALLOCATION  *DstX = (EX_ALLOCATION *)(ULONG_PTR)B->Dst.Address;
  EX_ALLOCATION  *SrcX = Fill ? NULL : (EX_ALLOCATION *)(ULONG_PTR)B->Src.Address;
  PUCHAR         Dst = (DstX != NULL) ? CpuVa (DstX) : NULL;
  PUCHAR         Src = (SrcX != NULL) ? CpuVa (SrcX) : NULL;
  ULONG          Index;

  if ((Dst == NULL) || (!Fill && (Src == NULL))) {
    if (A->BadCommands++ < 10) {
      OsLog ("exynoswddm: blt skipped: dst %p (pages %s, cpu %p) src %p (pages %s, cpu %p)", DstX,
             (DstX != NULL && DstX->Mdl != NULL) ? "mapped" : "none", Dst, SrcX,
             (SrcX != NULL && SrcX->Mdl != NULL) ? "mapped" : "none", Src);
    }
    return;
  }
  for (Index = 0; Index < B->RectCount; Index++) {
    EX_RECT  R = B->Rects[Index];
    LONG     Y;
    LONG     X;

    R.Left   = max (R.Left, 0);
    R.Top    = max (R.Top, 0);
    R.Right  = min (R.Right, (LONG)B->DstWidth);
    R.Bottom = min (R.Bottom, (LONG)B->DstHeight);
    if (!Fill) {
      R.Left   = max (R.Left, -B->OffsetX);
      R.Top    = max (R.Top, -B->OffsetY);
      R.Right  = min (R.Right, (LONG)B->SrcWidth - B->OffsetX);
      R.Bottom = min (R.Bottom, (LONG)B->SrcHeight - B->OffsetY);
    }
    if ((R.Right <= R.Left) || (R.Bottom <= R.Top)) {
      continue;
    }
    for (Y = R.Top; Y < R.Bottom; Y++) {
      UINT32  *D = (UINT32 *)(Dst + (SIZE_T)Y * B->DstPitch) + R.Left;

      if (Fill) {
        for (X = R.Left; X < R.Right; X++) {
          *D++ = B->Color;
        }
      } else {
        RtlCopyMemory (D, Src + (SIZE_T)(Y + B->OffsetY) * B->SrcPitch + (SIZE_T)(R.Left + B->OffsetX) * 4,
                       (SIZE_T)(R.Right - R.Left) * 4);
      }
    }
    if ((DstX == A->ScanAlloc) && !DstX->GpuPresented) {
      BltToFront (A, Dst, B->DstPitch, &R);
    }
  }
}

/**
  The CPU view of GPU address Va (allocation mapped, CpuLock held), or NULL.
**/
static PUCHAR
GpuVaToCpu (ADAPTER *A, ULONGLONG Va, ULONG Bytes)
{
  KIRQL          Irql;
  PLIST_ENTRY    E;
  EX_ALLOCATION  *Found = NULL;
  PUCHAR         Cpu;

  KeAcquireSpinLock (&A->AllocLock, &Irql);
  for (E = A->Allocs.Flink; E != &A->Allocs; E = E->Flink) {
    EX_ALLOCATION  *X = CONTAINING_RECORD (E, EX_ALLOCATION, Link);

    if ((Va >= X->GpuVa) && (Va + Bytes <= X->GpuVa + ((ULONGLONG)X->Pages << PAGE_SHIFT))) {
      Found = X;
      break;
    }
  }
  KeReleaseSpinLock (&A->AllocLock, Irql);
  if ((Found == NULL) || (Found->Mdl == NULL)) {
    return NULL;
  }
  Cpu = CpuVa (Found);
  return (Cpu != NULL) ? Cpu + (SIZE_T)(Va - Found->GpuVa) : NULL;
}

/**
  After a failed chain: each job header as the GPU left it (Midgard job
  header: exception status, first incomplete task, fault pointer, type,
  next job), and the 32 bytes the faulting job's fault pointer names.
**/
static VOID
DumpChain (ADAPTER *A, ULONGLONG Chain)
{
  ULONGLONG  Va = Chain;
  ULONG      N;

  ExAcquireFastMutex (&A->CpuLock);
  for (N = 0; (N < 12) && (Va != 0); N++) {
    PUCHAR     H = GpuVaToCpu (A, Va, 0x20);
    ULONG      Exc;
    ULONGLONG  Fault;
    ULONGLONG  Next;

    if (H == NULL) {
      RLOG ("exynoswddm:   job %lu at 0x%08lx: not mapped", N, (ULONG)Va);
      break;
    }
    Exc   = *(volatile ULONG *)H;
    Fault = *(volatile ULONGLONG *)(H + 8);
    Next  = *(volatile ULONGLONG *)(H + 0x18);
    RLOG ("exynoswddm:   job %lu at 0x%08lx: type %lu exception 0x%08lx first task 0x%08lx fault 0x%08lx%08lx next 0x%08lx",
          N, (ULONG)Va, (ULONG)(H[0x10] >> 1) & 0x7F, Exc, *(volatile ULONG *)(H + 4), (ULONG)(Fault >> 32),
          (ULONG)Fault, (ULONG)Next);
    if (((Exc & 0xFF) >= 0x40) && (Fault != 0)) {
      PUCHAR  F = GpuVaToCpu (A, Fault & ~0xFull, 32);

      {
        KIRQL        Irql2;
        PLIST_ENTRY  E2;

        KeAcquireSpinLock (&A->AllocLock, &Irql2);
        for (E2 = A->Allocs.Flink; E2 != &A->Allocs; E2 = E2->Flink) {
          EX_ALLOCATION  *Y = CONTAINING_RECORD (E2, EX_ALLOCATION, Link);

          if ((Fault >= Y->GpuVa) && (Fault < Y->GpuVa + ((ULONGLONG)Y->Pages << PAGE_SHIFT))) {
            RLOG ("exynoswddm:     in allocation %p: GPU 0x%08lx, %lu pages, flags 0x%lx, MDL %p, %s", Y, (ULONG)Y->GpuVa,
                  Y->Pages, Y->Info.Flags, Y->Mdl, Y->GpuMapped ? "GPU mapped" : "not GPU mapped");
            break;
          }
        }
        KeReleaseSpinLock (&A->AllocLock, Irql2);
      }
      if (F != NULL) {
        RLOG ("exynoswddm:     at 0x%08lx: %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx", (ULONG)(Fault & ~0xFull),
              ((volatile ULONG *)F)[0], ((volatile ULONG *)F)[1], ((volatile ULONG *)F)[2], ((volatile ULONG *)F)[3],
              ((volatile ULONG *)F)[4], ((volatile ULONG *)F)[5], ((volatile ULONG *)F)[6], ((volatile ULONG *)F)[7]);
      } else {
        RLOG ("exynoswddm:     fault address 0x%08lx is in no mapped allocation", (ULONG)Fault);
      }
    }
    Va = Next;
  }
  ExReleaseFastMutex (&A->CpuLock);
}

/**
  After a completed chain: for each fragment job (type 9) the single-target
  framebuffer descriptor (Midgard v4): colour write enable / format word,
  bounds, colour writeback address + row stride, the allocation that holds
  it and two pixels the GPU left there.
**/
static VOID
DumpFragments (ADAPTER *A, ULONGLONG Chain)
{
  ULONGLONG  Va = Chain;
  ULONG      N;

  ExAcquireFastMutex (&A->CpuLock);
  for (N = 0; (N < 12) && (Va != 0); N++) {
    PUCHAR  H = GpuVaToCpu (A, Va, 0x40);

    if (H == NULL) {
      break;
    }
    if (((H[0x10] >> 1) & 0x7F) == 9) {
      ULONGLONG  FbVa = *(volatile ULONGLONG *)(H + 0x28) & ~0x3Full;
      PUCHAR     Fb = GpuVaToCpu (A, FbVa, 0x80 + 16);

      if (Fb == NULL) {
        RLOG ("exynoswddm:   fragment job at 0x%08lx: framebuffer 0x%08lx not mapped", (ULONG)Va, (ULONG)FbVa);
      } else {
        ULONG      P0 = *(volatile ULONG *)(Fb + 32);
        ULONG      MaxXY = *(volatile ULONG *)(Fb + 44);
        ULONGLONG  Color = *(volatile ULONGLONG *)(Fb + 80);
        ULONG      Stride = *(volatile ULONG *)(Fb + 88);
        ULONG      W = (MaxXY & 0xFFFF) + 1;
        ULONG      Hh = (MaxXY >> 16) + 1;
        PUCHAR     C0 = GpuVaToCpu (A, Color, 4);
        PUCHAR     Cc = GpuVaToCpu (A, Color + (ULONGLONG)(Hh / 2) * Stride + (W / 2) * 4, 4);

        RLOG ("exynoswddm:   fragment job at 0x%08lx: fb 0x%08lx params0 0x%08lx (write %lu) %lux%lu, colour 0x%08lx stride %lu: (0,0) 0x%08lx centre 0x%08lx",
              (ULONG)Va, (ULONG)FbVa, P0, P0 >> 31, W, Hh, (ULONG)Color, Stride, C0 ? *(volatile ULONG *)C0 : 0xDEADDEAD,
              Cc ? *(volatile ULONG *)Cc : 0xDEADDEAD);
      }
    }
    Va = *(volatile ULONGLONG *)(H + 0x18);
  }
  ExReleaseFastMutex (&A->CpuLock);
}

static VOID
ExecJob (ADAPTER *A, CONST EX_CMD_GPU_JOB_ARGS *J)
{
  ULONG  Status;

  if (!GpuConnect (A)) {
    A->JobsFailed++;
    return;
  }
  Status = A->Gpu.RunChain (A->Gpu.Header.Context, J->Slot, J->Chain, GPU_JOB_TIMEOUT_MS);
  A->LastJsStatus = Status;
  if (Status == 1) {
    A->JobsDone++;
    if ((J->Slot == 0) && (A->FragDumps++ < 16)) {
      DumpFragments (A, J->Chain);
    }
  } else {
    A->JobsFailed++;
    if (A->JobsFailed <= 4) {
      DumpChain (A, J->Chain);
    }
  }
}

static VOID
Execute (ADAPTER *A, PUCHAR Dma, ULONG Bytes)
{
  ULONG  Offset = 0;

  while (Offset + sizeof (EX_CMD) <= Bytes) {
    EX_CMD  *C = (EX_CMD *)(Dma + Offset);

    if ((C->Size < sizeof (EX_CMD)) || (C->Size > Bytes - Offset)) {
      A->BadCommands++;
      return;
    }
    A->Commands++;
    switch (C->Op) {
      case EX_CMD_NOP:
        break;
      case EX_CMD_BLT:
      case EX_CMD_COLORFILL:
        if (C->Size >= FIELD_OFFSET (EX_CMD_BLT_ARGS, Rects)) {
          ExAcquireFastMutex (&A->CpuLock);
          ExecBlt (A, (CONST EX_CMD_BLT_ARGS *)C, C->Op == EX_CMD_COLORFILL);
          ExReleaseFastMutex (&A->CpuLock);
        }
        break;
      case EX_CMD_GPU_JOB:
        if (C->Size >= sizeof (EX_CMD_GPU_JOB_ARGS)) {
          ExecJob (A, (CONST EX_CMD_GPU_JOB_ARGS *)C);
        }
        break;
      default:
        A->BadCommands++;
        break;
    }
    Offset += C->Size;
  }
}

/**
  Flip: the primary into the back scan-out buffer, then FIMD1 to it at the
  next vsync, whose interrupt reports the primary as on screen.
**/
static NTSTATUS ScanoutMapTo (ADAPTER *A, ULONGLONG Va, ULONG Index);

static VOID
ExecFlip (ADAPTER *A, EX_ALLOCATION *X, ULONGLONG PrimaryAddress)
{
  ULONG   Back = (A->ScanFront == 0) ? 1 : 0;
  PUCHAR  Src = CpuVa (X);
  PUCHAR  Dst = A->ScanBuf[Back].Va;
  ULONG   Y;
  ULONG   Pitch = (X->Info.Pitch != 0) ? X->Info.Pitch : WIDTH * 4;

  if (X->GpuPresented && (A->ScanGpuVa != 0)) {
    //
    // The UMD's GPU blit has put the frame into the off-screen scan-out
    // buffer (and waited for it): show that one, no copy.  Once the vsync
    // has latched it, the blit target moves to the other buffer - before
    // the next job, the next blit, runs (this worker runs them in order).
    //
    ULONG          Shown = A->ScanGpuBack;
    ULONG          Waited;
    LARGE_INTEGER  Ms;

    A->ScanAlloc      = X;
    A->ScanAllocBuf   = Shown;
    A->PendingPrimary = PrimaryAddress;
    A->PendingPa      = A->ScanBuf[Shown].Pa.LowPart;
    A->Flips++;
    A->GpuFlips++;
    ArmVsync (A, A->ScanBuf[Shown].Pa.LowPart);
    Ms.QuadPart = -10000;                           // 1 ms
    for (Waited = 0; A->FlipPending && (Waited < 50); Waited++) {
      KeDelayExecutionThread (KernelMode, FALSE, &Ms);
    }
    if (A->FlipPending) {
      A->FlipWaitTimeouts++;                        // not latched (yet): keep writing the same buffer
      return;
    }
    if (NT_SUCCESS (ScanoutMapTo (A, A->ScanGpuVa, Shown ^ 1))) {
      A->ScanGpuBack = Shown ^ 1;
    }
    return;
  }
  if ((Src == NULL) || (Dst == NULL)) {
    A->BadCommands++;
    return;
  }
  for (Y = 0; Y < min (X->Info.Height, (ULONG)HEIGHT); Y++) {
    RtlCopyMemory (Dst + (SIZE_T)Y * WIDTH * 4, Src + (SIZE_T)Y * Pitch, (SIZE_T)min (X->Info.Width, (ULONG)WIDTH) * 4);
  }
  if (A->Flips < 8) {
    PUCHAR  C = Src + (SIZE_T)(min (X->Info.Height, (ULONG)HEIGHT) / 2) * Pitch + (SIZE_T)(min (X->Info.Width, (ULONG)WIDTH) / 2) * 4;

    RLOG ("exynoswddm: flip %lu from %p: center 0x%08lx, (0,0) 0x%08lx -> scan-out %lu", A->Flips, X,
          *(volatile ULONG *)C, *(volatile ULONG *)Src, Back);
  }
  A->ScanAlloc      = X;
  A->ScanAllocBuf   = Back;
  A->PendingPrimary = PrimaryAddress;
  A->PendingPa      = A->ScanBuf[Back].Pa.LowPart;
  A->Flips++;
  ArmVsync (A, A->ScanBuf[Back].Pa.LowPart);
}

typedef struct {
  ADAPTER  *A;
  ULONG    FenceId;
} COMPLETE_SYNC;

static BOOLEAN
CompleteSync (PVOID Context)
{
  COMPLETE_SYNC                    *S = (COMPLETE_SYNC *)Context;
  DXGKARGCB_NOTIFY_INTERRUPT_DATA  Notify;

  RtlZeroMemory (&Notify, sizeof (Notify));
  Notify.InterruptType                  = DXGK_INTERRUPT_DMA_COMPLETED;
  Notify.DmaCompleted.SubmissionFenceId = S->FenceId;
  S->A->Dxgk.DxgkCbNotifyInterrupt (S->A->Dxgk.DeviceHandle, &Notify);
  S->A->LastCompleted = S->FenceId;
  return TRUE;
}

static VOID
Complete (ADAPTER *A, ULONG FenceId)
{
  COMPLETE_SYNC  S;
  BOOLEAN        Ret;

  S.A       = A;
  S.FenceId = FenceId;
  A->Dxgk.DxgkCbSynchronizeExecution (A->Dxgk.DeviceHandle, CompleteSync, &S, 0, &Ret);
  A->Dxgk.DxgkCbQueueDpc (A->Dxgk.DeviceHandle);
}

static VOID HandlePreempt (ADAPTER *A);

static VOID
Worker (PVOID Context)
{
  ADAPTER     *A = (ADAPTER *)Context;
  SUBMISSION  S;
  KIRQL       Irql;
  BOOLEAN     Have;

  LARGE_INTEGER  Beat;
  ULONG          Seen = 0;

  Beat.QuadPart = -10000LL * 5000;
  for (;;) {
    if (KeWaitForSingleObject (&A->QueueEvent, Executive, KernelMode, FALSE, &Beat) == STATUS_TIMEOUT) {
      ULONG  Now = A->Presents + A->Submissions + A->Renders + A->Flips + A->AllocCount;

      if (Now != Seen) {
        Seen = Now;
        OsLog ("exynoswddm: [5 s] presents %lu (blt %lu fill %lu), submissions %lu, renders %lu, flips %lu (%lu no-copy, %lu unlatched), allocations %lu, "
               "fence %lu/%lu, jobs %lu (%lu failed), maps %lu (%lu failed), bad %lu", A->Presents, A->PresentBlts,
               A->PresentFills, A->Submissions, A->Renders, A->Flips, A->GpuFlips, A->FlipWaitTimeouts, A->AllocCount, A->LastCompleted, A->LastSubmitted,
               A->JobsDone, A->JobsFailed, A->Maps, A->MapFails, A->BadCommands);
      }
      continue;
    }
    for (;;) {
      if (A->PreemptPending) {
        HandlePreempt (A);
      }
      KeAcquireSpinLock (&A->QueueLock, &Irql);
      Have = (A->QueueHead != A->QueueTail);
      if (Have) {
        S = A->Queue[A->QueueHead % SUBMIT_QUEUE];
        InterlockedExchange (&A->Running, 1);
      }
      KeReleaseSpinLock (&A->QueueLock, Irql);
      if (!Have) {
        break;
      }
      if (S.Kind == SUB_DROPPED) {
        // preempted before it started: the scheduler resubmits it
      } else if (S.Kind == SUB_FLIP) {
        //
        // VidMm may have unmapped (paging thread) or destroyed the primary
        // since the flip was queued: under CpuLock, take the entry's
        // primary again (FreeAllocation clears it) and copy only while it
        // is still mapped.
        //
        ExAcquireFastMutex (&A->CpuLock);
        KeAcquireSpinLock (&A->QueueLock, &Irql);
        S.Primary = A->Queue[A->QueueHead % SUBMIT_QUEUE].Primary;
        KeReleaseSpinLock (&A->QueueLock, Irql);
        if ((S.Primary != NULL) && (S.Primary->Mdl != NULL)) {
          ExecFlip (A, S.Primary, S.PrimaryAddress);
        } else if (A->FlipsSkipped++ < 10) {
          RLOG ("exynoswddm: flip skipped, primary %p gone or not mapped", S.Primary);
        }
        ExReleaseFastMutex (&A->CpuLock);
      } else {
        if (S.Dma != NULL) {
          Execute (A, (PUCHAR)S.Dma, S.Bytes);
        }
        Complete (A, S.FenceId);
      }
      KeAcquireSpinLock (&A->QueueLock, &Irql);
      A->QueueHead++;
      InterlockedExchange (&A->Running, 0);
      KeReleaseSpinLock (&A->QueueLock, Irql);
    }
    if (A->WorkerStop) {
      PsTerminateSystemThread (STATUS_SUCCESS);
    }
  }
}

static BOOLEAN
Enqueue (ADAPTER *A, CONST SUBMISSION *S)
{
  KIRQL    Irql;
  BOOLEAN  Ok = FALSE;

  KeAcquireSpinLock (&A->QueueLock, &Irql);
  if (A->QueueTail - A->QueueHead < SUBMIT_QUEUE) {
    A->Queue[A->QueueTail % SUBMIT_QUEUE] = *S;
    A->QueueTail++;
    Ok = TRUE;
  }
  KeReleaseSpinLock (&A->QueueLock, Irql);
  KeSetEvent (&A->QueueEvent, IO_NO_INCREMENT, FALSE);
  return Ok;
}

NTSTATUS
RenderStart (ADAPTER *A)
{
  HANDLE    Thread;
  NTSTATUS  Status;

  //
  // The firmware carve-out (reserved memory, ExynosGpuCarveout.h) holds the
  // two scan-out buffers.
  //
  if (A->CarveVa == NULL) {
    PHYSICAL_ADDRESS     Pa;
    GPU_CARVEOUT_HEADER  *Hdr;
    BOOLEAN              Ok = FALSE;

    Pa.QuadPart = GPU_CARVEOUT_BASE;
    Hdr = MmMapIoSpace (Pa, PAGE_SIZE, MmNonCached);
    if (Hdr != NULL) {
      Ok = (Hdr->Magic == GPU_CARVEOUT_MAGIC) && (Hdr->Base == GPU_CARVEOUT_BASE) && (Hdr->Size == GPU_CARVEOUT_SIZE);
      MmUnmapIoSpace (Hdr, PAGE_SIZE);
    }
    if (!Ok) {
      OsLog ("exynoswddm: no GPU carve-out from the firmware (needs the carve-out UEFI image)");
      return STATUS_INSUFFICIENT_RESOURCES;
    }
    A->CarvePa.QuadPart = GPU_CARVEOUT_BASE + CARVEOUT_SKIP;
    A->CarveBytes       = 2 * (ULONG)FB_BYTES;
    A->CarveVa          = MmMapIoSpace (A->CarvePa, A->CarveBytes, MmWriteCombined);
    if (A->CarveVa == NULL) {
      return STATUS_INSUFFICIENT_RESOURCES;
    }
    A->ScanBuf[0].Va          = A->CarveVa;
    A->ScanBuf[0].Pa          = A->CarvePa;
    A->ScanBuf[1].Va          = A->CarveVa + FB_BYTES;
    A->ScanBuf[1].Pa.QuadPart = A->CarvePa.QuadPart + FB_BYTES;
    RtlCopyMemory (A->ScanBuf[0].Va, A->Boot.Va, FB_BYTES);
    RtlCopyMemory (A->ScanBuf[1].Va, A->Boot.Va, FB_BYTES);
  }
  if (A->VaBits == NULL) {
    A->VaBits = ExAllocatePoolWithTag (NonPagedPool, (VA_GRANULES + 31) / 32 * 4, TAG_VA);
    if (A->VaBits == NULL) {
      return STATUS_INSUFFICIENT_RESOURCES;
    }
    RtlInitializeBitMap (&A->VaMap, A->VaBits, VA_GRANULES);
    RtlClearAllBits (&A->VaMap);
    KeInitializeSpinLock (&A->AllocLock);
    InitializeListHead (&A->Allocs);
    ExInitializeFastMutex (&A->VaLock);
    ExInitializeFastMutex (&A->ShadowLock);
    ExInitializeFastMutex (&A->CpuLock);
    ExInitializeFastMutex (&A->ScanMapLock);
  }
  OsLog ("exynoswddm: aperture segment %lu MiB (nominal 0x%08lx), GPU addresses 0x%08lx..0x%08lx, scan-out buffers "
         "0x%08lx / 0x%08lx, %s", (ULONG)(APERTURE_BYTES >> 20), (ULONG)APERTURE_BASE, (ULONG)EX_GPU_VA_START,
         (ULONG)EX_GPU_VA_END, A->ScanBuf[0].Pa.LowPart, A->ScanBuf[1].Pa.LowPart,
         EXW_GPU_MODE ? "GPU mode" : "present-only mode (UMD renders on the CPU)");

  KeInitializeSpinLock (&A->QueueLock);
  KeInitializeEvent (&A->QueueEvent, SynchronizationEvent, FALSE);
  A->QueueHead     = 0;
  A->QueueTail     = 0;
  A->WorkerStop    = 0;
  A->LastCompleted = 0;
  A->LastSubmitted = 0;
  Status = PsCreateSystemThread (&Thread, THREAD_ALL_ACCESS, NULL, NULL, NULL, Worker, A);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  ObReferenceObjectByHandle (Thread, THREAD_ALL_ACCESS, NULL, KernelMode, (PVOID *)&A->Worker, NULL);
  ZwClose (Thread);
  GpuConnect (A);
  return STATUS_SUCCESS;
}

VOID
RenderStop (ADAPTER *A)
{
  if (A->Worker != NULL) {
    InterlockedExchange (&A->WorkerStop, 1);
    KeSetEvent (&A->QueueEvent, IO_NO_INCREMENT, FALSE);
    KeWaitForSingleObject (A->Worker, Executive, KernelMode, FALSE, NULL);
    ObDereferenceObject (A->Worker);
    A->Worker = NULL;
  }
  OsLog ("exynoswddm: render stop: %lu submissions, %lu commands, %lu bad, %lu jobs (%lu failed), %lu flips, "
         "%lu maps (%lu failed), last fence %lu", A->Submissions, A->Commands, A->BadCommands, A->JobsDone,
         A->JobsFailed, A->Flips, A->Maps, A->MapFails, A->LastCompleted);
}

VOID
RenderRelease (ADAPTER *A)
{
  if ((A->ScanGpuVa != 0) && A->GpuOk) {
    A->Gpu.UnmapPages (A->Gpu.Header.Context, A->ScanGpuVa, (ULONG)(FB_BYTES >> PAGE_SHIFT));
  }
  A->ScanGpuVa = 0;
  if (A->ScanPfn != NULL) {
    ExFreePoolWithTag (A->ScanPfn, TAG_VA);
    A->ScanPfn = NULL;
  }
  if (A->CarveVa != NULL) {
    MmUnmapIoSpace (A->CarveVa, A->CarveBytes);
    A->CarveVa = NULL;
  }
  if (A->GpuOk && (A->Gpu.Header.InterfaceDereference != NULL)) {
    A->Gpu.Header.InterfaceDereference (A->Gpu.Header.Context);
  }
  A->GpuOk = FALSE;
  if (A->VaBits != NULL) {
    ExFreePoolWithTag (A->VaBits, TAG_VA);
    A->VaBits = NULL;
  }
}

VOID
RenderDpc (ADAPTER *A)
{
  UNREFERENCED_PARAMETER (A);
}

//
// ---------------------------------------------------------------------------
// Devices, contexts
// ---------------------------------------------------------------------------
//

static NTSTATUS APIENTRY
RnCreateDevice (IN_CONST_HANDLE hAdapter, INOUT_PDXGKARG_CREATEDEVICE pCreateDevice)
{
  EX_DEVICE  *D = ExAllocatePoolWithTag (NonPagedPool, sizeof (EX_DEVICE), TAG_DEVICE);

  if (D == NULL) {
    return STATUS_NO_MEMORY;
  }
  D->Magic  = EX_DEVICE_MAGIC;
  D->A      = (ADAPTER *)hAdapter;
  D->System = (BOOLEAN)pCreateDevice->Flags.SystemDevice;
  pCreateDevice->hDevice = D;
  //
  // pInfo shares a union with Flags and is only an output for drivers that
  // are not multi-engine aware; this one is (DMA buffer / list sizes come
  // from CreateContext's ContextInfo).  For a system device the union
  // holds Flags.SystemDevice = 1: never touch it.
  //
  RLOG ("exynoswddm: CreateDevice %p%s", D, D->System ? " (system)" : "");
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnDestroyDevice (IN_CONST_HANDLE hDevice)
{
  ExFreePoolWithTag ((PVOID)hDevice, TAG_DEVICE);
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnCreateContext (IN_CONST_HANDLE hDevice, INOUT_PDXGKARG_CREATECONTEXT pCreateContext)
{
  EX_CONTEXT  *C = ExAllocatePoolWithTag (NonPagedPool, sizeof (EX_CONTEXT), TAG_CONTEXT);

  if (C == NULL) {
    return STATUS_NO_MEMORY;
  }
  C->Magic = EX_CONTEXT_MAGIC;
  C->Dev  = (EX_DEVICE *)hDevice;
  C->Node = pCreateContext->NodeOrdinal;
  pCreateContext->hContext = C;
  RtlZeroMemory (&pCreateContext->ContextInfo, sizeof (pCreateContext->ContextInfo));
  pCreateContext->ContextInfo.DmaBufferSize            = EX_DMA_BUFFER_SIZE;
  pCreateContext->ContextInfo.DmaBufferSegmentSet      = 0;     // system memory
  pCreateContext->ContextInfo.DmaBufferPrivateDataSize = 0;
  pCreateContext->ContextInfo.AllocationListSize       = EX_ALLOCATION_LIST;
  pCreateContext->ContextInfo.PatchLocationListSize    = EX_ALLOCATION_LIST;
  RLOG ("exynoswddm: CreateContext %p (device %p, node %lu, flags 0x%x)", C, hDevice, C->Node,
        pCreateContext->Flags.Value);
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnDestroyContext (IN_CONST_HANDLE hContext)
{
  ExFreePoolWithTag ((PVOID)hContext, TAG_CONTEXT);
  return STATUS_SUCCESS;
}

//
// ---------------------------------------------------------------------------
// Allocations
// ---------------------------------------------------------------------------
//

static NTSTATUS APIENTRY
RnGetStandardAllocationDriverData (IN_CONST_HANDLE hAdapter,
                                   INOUT_PDXGKARG_GETSTANDARDALLOCATIONDRIVERDATA pData)
{
  EXMW_ALLOCATION_INFO  I;

  UNREFERENCED_PARAMETER (hAdapter);

  RtlZeroMemory (&I, sizeof (I));
  I.Version = EXMW_ABI_VERSION;
  I.Flags   = EXMW_ALLOC_NOEXEC;
  switch (pData->StandardAllocationType) {
    case D3DKMDT_STANDARDALLOCATION_SHAREDPRIMARYSURFACE:
      I.Flags |= EXMW_ALLOC_PRIMARY;
      I.Width  = pData->pCreateSharedPrimarySurfaceData->Width;
      I.Height = pData->pCreateSharedPrimarySurfaceData->Height;
      I.Format = pData->pCreateSharedPrimarySurfaceData->Format;
      break;
    case D3DKMDT_STANDARDALLOCATION_SHADOWSURFACE:
      I.Width  = pData->pCreateShadowSurfaceData->Width;
      I.Height = pData->pCreateShadowSurfaceData->Height;
      I.Format = pData->pCreateShadowSurfaceData->Format;
      pData->pCreateShadowSurfaceData->Pitch = SurfacePitch (I.Width);
      break;
    case D3DKMDT_STANDARDALLOCATION_STAGINGSURFACE:
      I.Width  = pData->pCreateStagingSurfaceData->Width;
      I.Height = pData->pCreateStagingSurfaceData->Height;
      I.Format = D3DDDIFMT_A8R8G8B8;
      pData->pCreateStagingSurfaceData->Pitch = SurfacePitch (I.Width);
      break;
    default:
      RLOG ("exynoswddm: standard allocation type %u not supported", (ULONG)pData->StandardAllocationType);
      return STATUS_NOT_SUPPORTED;
  }
  I.Pitch = SurfacePitch (I.Width);
  I.Size  = (UINT64)I.Pitch * I.Height;
  pData->ResourcePrivateDriverDataSize = 0;
  if (pData->pAllocationPrivateDriverData == NULL) {
    pData->AllocationPrivateDriverDataSize = sizeof (I);
    return STATUS_SUCCESS;
  }
  if (pData->AllocationPrivateDriverDataSize < sizeof (I)) {
    return STATUS_BUFFER_TOO_SMALL;
  }
  RtlCopyMemory (pData->pAllocationPrivateDriverData, &I, sizeof (I));
  return STATUS_SUCCESS;
}

static VOID
FreeAllocation (ADAPTER *A, EX_ALLOCATION *X)
{
  KIRQL  Irql;
  ULONG  Q;

  //
  // Queued flips must not reach a freed allocation (the worker re-reads
  // the entry under CpuLock).
  //
  ExAcquireFastMutex (&A->CpuLock);
  KeAcquireSpinLock (&A->QueueLock, &Irql);
  for (Q = A->QueueHead; Q != A->QueueTail; Q++) {
    if ((A->Queue[Q % SUBMIT_QUEUE].Kind == SUB_FLIP) && (A->Queue[Q % SUBMIT_QUEUE].Primary == X)) {
      A->Queue[Q % SUBMIT_QUEUE].Primary = NULL;
    }
  }
  KeReleaseSpinLock (&A->QueueLock, Irql);
  ExReleaseFastMutex (&A->CpuLock);

  if (X->Shadow != NULL) {
    ExAcquireFastMutex (&A->ShadowLock);
    ExFreePoolWithTag (X->Shadow, 'dSxE');
    X->Shadow      = NULL;
    X->ShadowBytes = 0;
    ExReleaseFastMutex (&A->ShadowLock);
  }

  if (A->ScanAlloc == X) {
    A->ScanAlloc = NULL;
  }
  GpuUnmap (A, X);
  CpuUnmap (X);
  KeAcquireSpinLock (&A->AllocLock, &Irql);
  RemoveEntryList (&X->Link);
  KeReleaseSpinLock (&A->AllocLock, Irql);
  if (X->VaCount != 0) {
    ExAcquireFastMutex (&A->VaLock);
    RtlClearBits (&A->VaMap, X->VaFirst, X->VaCount);
    ExReleaseFastMutex (&A->VaLock);
  }
  A->AllocCount--;
  ExFreePoolWithTag (X, TAG_ALLOC);
}

static NTSTATUS APIENTRY
RnCreateAllocation (IN_CONST_HANDLE hAdapter, INOUT_PDXGKARG_CREATEALLOCATION pCreateAllocation)
{
  ADAPTER  *A = (ADAPTER *)hAdapter;
  ULONG    Index;

  for (Index = 0; Index < pCreateAllocation->NumAllocations; Index++) {
    DXGK_ALLOCATIONINFO        *I = &pCreateAllocation->pAllocationInfo[Index];
    CONST EXMW_ALLOCATION_INFO *In = (CONST EXMW_ALLOCATION_INFO *)I->pPrivateDriverData;
    EX_ALLOCATION              *X;
    ULONG                      Granules;
    ULONG                      First;
    KIRQL                      Irql;

    if ((In == NULL) || (I->PrivateDriverDataSize < sizeof (*In)) || (In->Version != EXMW_ABI_VERSION) ||
        (In->Size == 0) || (In->Size > APERTURE_BYTES)) {
      RLOG ("exynoswddm: CreateAllocation %lu: bad private data (%p, %u bytes)", Index, In, I->PrivateDriverDataSize);
      while (Index-- > 0) {
        FreeAllocation (A, (EX_ALLOCATION *)pCreateAllocation->pAllocationInfo[Index].hAllocation);
      }
      return STATUS_INVALID_PARAMETER;
    }
    X = ExAllocatePoolWithTag (NonPagedPool, sizeof (*X), TAG_ALLOC);
    if (X == NULL) {
      return STATUS_NO_MEMORY;
    }
    RtlZeroMemory (X, sizeof (*X));
    X->Info  = *In;
    X->Pages = (ULONG)((In->Size + PAGE_SIZE - 1) >> PAGE_SHIFT);

    //
    // Its GPU address for good.
    //
    Granules = (ULONG)(((ULONGLONG)X->Pages * PAGE_SIZE + VA_GRANULE - 1) / VA_GRANULE);
    ExAcquireFastMutex (&A->VaLock);
    if ((In->Flags & EXMW_ALLOC_NOEXEC) || (Granules > EXEC_WINDOW_GRANULES)) {
      First = RtlFindClearBitsAndSet (&A->VaMap, Granules, 0);
    } else {
      //
      // Executable (shader code): inside one 16 MB window - Midgard's
      // shader program counter is 24 bits, the upper bits come from the
      // descriptor (Linux panfrost_gem does the same).  First = 0xFFFFFFFF:
      // no window has room.
      //
      ULONG  Hint = 0;

      First = 0xFFFFFFFF;
      for (;;) {
        ULONG  Try = RtlFindClearBits (&A->VaMap, Granules, Hint);

        if ((Try == 0xFFFFFFFF) || (Try < Hint)) {
          break;                                   // none (RtlFindClearBits wraps around)
        }
        if ((Try / EXEC_WINDOW_GRANULES) == ((Try + Granules - 1) / EXEC_WINDOW_GRANULES)) {
          RtlSetBits (&A->VaMap, Try, Granules);
          First = Try;
          break;
        }
        Hint = (Try / EXEC_WINDOW_GRANULES + 1) * EXEC_WINDOW_GRANULES;
        if (Hint >= VA_GRANULES) {
          break;
        }
      }
    }
    ExReleaseFastMutex (&A->VaLock);
    if (First == 0xFFFFFFFF) {
      RLOG ("exynoswddm: GPU address space full (%lu pages)", X->Pages);
      ExFreePoolWithTag (X, TAG_ALLOC);
      return STATUS_GRAPHICS_NO_VIDEO_MEMORY;
    }
    X->VaFirst = First;
    X->VaCount = Granules;
    X->GpuVa   = EX_GPU_VA_START + (ULONGLONG)First * VA_GRANULE;
    if (!(In->Flags & EXMW_ALLOC_NOEXEC) && (A->ExecLogs++ < 30)) {
      RLOG ("exynoswddm: executable allocation %p: %lu pages at GPU 0x%08lx..0x%08lx", X, X->Pages, (ULONG)X->GpuVa,
            (ULONG)(X->GpuVa + ((ULONGLONG)X->Pages << PAGE_SHIFT) - 1));
    }
    KeAcquireSpinLock (&A->AllocLock, &Irql);
    InsertTailList (&A->Allocs, &X->Link);
    KeReleaseSpinLock (&A->AllocLock, Irql);
    A->AllocCount++;

    I->hAllocation                 = X;
    I->Alignment                   = 0;
    I->Size                        = (SIZE_T)X->Pages << PAGE_SHIFT;
    I->PitchAlignedSize            = 0;
    I->HintedBank.Value            = 0;
    I->PreferredSegment.Value      = 0;
    I->PreferredSegment.SegmentId0 = APERTURE_ID;
    I->SupportedReadSegmentSet     = 1u << (APERTURE_ID - 1);
    I->SupportedWriteSegmentSet    = 1u << (APERTURE_ID - 1);
    I->EvictionSegmentSet          = 0;
    I->MaximumRenamingListLength   = 0;
    I->pAllocationUsageHint        = NULL;
    I->Flags.Value                 = 0;
    I->Flags.CpuVisible            = 1;
    I->AllocationPriority          = D3DDDI_ALLOCATIONPRIORITY_NORMAL;
    if ((A->AllocCount <= 8) || (In->Flags & EXMW_ALLOC_PRIMARY)) {
      RLOG ("exynoswddm: CreateAllocation %p flags 0x%x %ux%u pitch %u, %lu pages at GPU 0x%08lx", X, In->Flags,
            In->Width, In->Height, In->Pitch, X->Pages, (ULONG)X->GpuVa);
    }
  }
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnDestroyAllocation (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_DESTROYALLOCATION pDestroyAllocation)
{
  ULONG  Index;

  for (Index = 0; Index < pDestroyAllocation->NumAllocations; Index++) {
    if (pDestroyAllocation->pAllocationList[Index] != NULL) {
      FreeAllocation ((ADAPTER *)hAdapter, (EX_ALLOCATION *)pDestroyAllocation->pAllocationList[Index]);
    }
  }
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnDescribeAllocation (IN_CONST_HANDLE hAdapter, INOUT_PDXGKARG_DESCRIBEALLOCATION pDescribeAllocation)
{
  EX_ALLOCATION  *X = (EX_ALLOCATION *)pDescribeAllocation->hAllocation;

  UNREFERENCED_PARAMETER (hAdapter);

  pDescribeAllocation->Width                              = X->Info.Width;
  pDescribeAllocation->Height                             = X->Info.Height;
  pDescribeAllocation->Format                             = (D3DDDIFORMAT)X->Info.Format;
  pDescribeAllocation->MultisampleMethod.NumSamples       = 0;
  pDescribeAllocation->MultisampleMethod.NumQualityLevels = 0;
  //
  // Exactly the VidPN mode's VSyncFreq: SetDisplayMode (DXGI fullscreen,
  // DWM) compares the primary's description with the current mode and
  // fails with a mode change on any difference (60/1 vs 60.02 Hz was one).
  //
  ExRefreshRate (&pDescribeAllocation->RefreshRate);
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnOpenAllocation (IN_CONST_HANDLE hDevice, IN_CONST_PDXGKARG_OPENALLOCATION pOpenAllocation)
{
  EX_DEVICE  *D = (EX_DEVICE *)hDevice;
  ULONG      Index;

  for (Index = 0; Index < pOpenAllocation->NumAllocations; Index++) {
    DXGK_OPENALLOCATIONINFO  *O = &pOpenAllocation->pOpenAllocation[Index];
    DXGKARGCB_GETHANDLEDATA  Get;

    RtlZeroMemory (&Get, sizeof (Get));
    Get.hObject = O->hAllocation;
    Get.Type    = DXGK_HANDLE_ALLOCATION;
    O->hDeviceSpecificAllocation = D->A->Dxgk.DxgkCbGetHandleData (&Get);
  }
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnCloseAllocation (IN_CONST_HANDLE hDevice, IN_CONST_PDXGKARG_CLOSEALLOCATION pCloseAllocation)
{
  UNREFERENCED_PARAMETER (hDevice);
  UNREFERENCED_PARAMETER (pCloseAllocation);
  return STATUS_SUCCESS;
}

//
// ---------------------------------------------------------------------------
// Paging: everything on the CPU while the paging buffer is built
// ---------------------------------------------------------------------------
//

static NTSTATUS APIENTRY
RnBuildPagingBuffer (IN_CONST_HANDLE hAdapter, IN_PDXGKARG_BUILDPAGINGBUFFER pBuildPagingBuffer)
{
  ADAPTER  *A = (ADAPTER *)hAdapter;

  if ((ULONG)pBuildPagingBuffer->Operation < 16) {
    A->PagingOps[pBuildPagingBuffer->Operation]++;
  }
  if (A->PagingLogs < 40) {
    A->PagingLogs++;
    OsLog ("exynoswddm: paging op %u: alloc %p%s", (ULONG)pBuildPagingBuffer->Operation,
           (pBuildPagingBuffer->Operation == DXGK_OPERATION_MAP_APERTURE_SEGMENT) ? pBuildPagingBuffer->MapApertureSegment.hAllocation :
           (pBuildPagingBuffer->Operation == DXGK_OPERATION_UNMAP_APERTURE_SEGMENT) ? pBuildPagingBuffer->UnmapApertureSegment.hAllocation :
           (pBuildPagingBuffer->Operation == DXGK_OPERATION_FILL) ? pBuildPagingBuffer->Fill.hAllocation :
           (pBuildPagingBuffer->Operation == DXGK_OPERATION_TRANSFER) ? pBuildPagingBuffer->Transfer.hAllocation : NULL,
           (pBuildPagingBuffer->Operation == DXGK_OPERATION_TRANSFER) ? " (transfer)" : "");
  }
  ExAcquireFastMutex (&A->CpuLock);
  switch (pBuildPagingBuffer->Operation) {
    case DXGK_OPERATION_MAP_APERTURE_SEGMENT: {
      EX_ALLOCATION  *X = (EX_ALLOCATION *)pBuildPagingBuffer->MapApertureSegment.hAllocation;

      if (X == NULL) {
        break;
      }
      {
        //
        // pMdl describes the whole allocation (MDL page n = allocation page
        // n); this operation maps NumberOfPages of them from MdlOffset - a
        // big allocation can come in pieces.  The old code mapped the whole
        // allocation from MdlOffset to its GPU address 0: shifted pages,
        // and the GPU executed other memory than the shader (0x51).
        //
        PMDL   Mdl   = pBuildPagingBuffer->MapApertureSegment.pMdl;
        ULONG  First = (ULONG)pBuildPagingBuffer->MapApertureSegment.MdlOffset;
        ULONG  Count = (ULONG)pBuildPagingBuffer->MapApertureSegment.NumberOfPages;
        ULONG  MdlPages = (ULONG)(MmGetMdlByteCount (Mdl) >> PAGE_SHIFT);

        if (X->Mdl != Mdl) {
          GpuUnmap (A, X);                       // new backing pages
          CpuUnmap (X);
          X->Mdl = Mdl;
        }
        X->MdlPage = 0;
        if (First == 0) {
          X->SegAddress = APERTURE_BASE + (ULONGLONG)pBuildPagingBuffer->MapApertureSegment.OffsetInPages * PAGE_SIZE;
        }
        if ((First < X->Pages) && (First < MdlPages)) {
          Count = min (Count, min (X->Pages, MdlPages) - First);
          GpuMapRange (A, X, First, Count);
        }
        if ((A->PagingLogs < 40) || (((First != 0) || (Count != X->Pages)) && (A->PartialMaps++ < 30))) {
          OsLog ("exynoswddm:   map %p: %lu pages from MDL page %lu (allocation %lu pages, MDL %lu) at segment page %lu, GPU 0x%08lx %s",
                 X, Count, First, X->Pages, MdlPages,
                 (ULONG)pBuildPagingBuffer->MapApertureSegment.OffsetInPages, (ULONG)(X->GpuVa + (ULONGLONG)First * PAGE_SIZE),
                 X->GpuMapped ? "mapped" : "not mapped");
        }
      }
      break;
    }
    case DXGK_OPERATION_UNMAP_APERTURE_SEGMENT: {
      EX_ALLOCATION  *X = (EX_ALLOCATION *)pBuildPagingBuffer->UnmapApertureSegment.hAllocation;

      if (X == NULL) {
        break;
      }
      if (A->ScanAlloc == X) {
        A->ScanAlloc = NULL;
      }
      GpuUnmap (A, X);
      CpuUnmap (X);
      X->Mdl = NULL;
      A->Unmaps++;
      break;
    }
    case DXGK_OPERATION_FILL: {
      EX_ALLOCATION  *X = (EX_ALLOCATION *)pBuildPagingBuffer->Fill.hAllocation;
      PUCHAR         Va = (X != NULL) ? CpuVa (X) : NULL;

      if (Va != NULL) {
        SIZE_T  N;
        SIZE_T  Bytes = min (pBuildPagingBuffer->Fill.FillSize, (SIZE_T)X->Pages << PAGE_SHIFT);

        for (N = 0; N + 4 <= Bytes; N += 4) {
          *(UINT32 *)(Va + N) = pBuildPagingBuffer->Fill.FillPattern;
        }
      }
      break;
    }
    default:
      // transfer / discard: aperture allocations live in their system pages
      break;
  }
  ExReleaseFastMutex (&A->CpuLock);
  return STATUS_SUCCESS;
}

//
// ---------------------------------------------------------------------------
// DMA buffers
// ---------------------------------------------------------------------------
//

/**
  UMD command buffer (EXMW_COMMAND list) -> EX_CMD_GPU_JOB commands.
**/
static NTSTATUS APIENTRY
RnRender (IN_CONST_HANDLE hContext, INOUT_PDXGKARG_RENDER pRender)
{
  CONST EXMW_COMMAND  *In = (CONST EXMW_COMMAND *)pRender->pCommand;
  ULONG               Count = pRender->CommandLength / sizeof (EXMW_COMMAND);
  PUCHAR              Out = (PUCHAR)pRender->pDmaBuffer;
  ULONG               N;
  ADAPTER             *A = AdapterOf (hContext);

  if ((ULONGLONG)Count * sizeof (EX_CMD_GPU_JOB_ARGS) > pRender->DmaSize) {
    return STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER;
  }
  for (N = 0; N < Count; N++) {
    EX_CMD_GPU_JOB_ARGS  *J = (EX_CMD_GPU_JOB_ARGS *)Out;

    if ((In[N].Type != EXMW_CMD_JOB) || (In[N].Slot > 1)) {
      return STATUS_INVALID_PARAMETER;
    }
    J->Hdr.Op   = EX_CMD_GPU_JOB;
    J->Hdr.Size = sizeof (*J);
    J->Chain    = In[N].Chain;
    J->Slot     = In[N].Slot;
    J->Reserved = 0;
    Out += sizeof (*J);
  }
  pRender->pDmaBuffer = Out;
  //
  // Nothing to patch (fixed GPU addresses), but VidMm only makes resident
  // (maps into the aperture, hence the Mali MMU) the allocations a patch
  // location names.  The UMD (malikm) sends none ("the KMD patches every
  // entry"): one per allocation-list entry, at DMA offset 0.
  //
  if (pRender->AllocationListSize > pRender->PatchLocationListOutSize) {
    if (A->PatchShort++ < 5) {
      RLOG ("exynoswddm: render: %u allocations, room for %u patch locations", pRender->AllocationListSize,
            pRender->PatchLocationListOutSize);
    }
  }
  for (N = 0; (N < pRender->AllocationListSize) && (N < pRender->PatchLocationListOutSize); N++) {
    D3DDDI_PATCHLOCATIONLIST  *P = &pRender->pPatchLocationListOut[N];

    RtlZeroMemory (P, sizeof (*P));
    P->AllocationIndex = N;
    P->SlotId          = N;
    P->PatchOffset     = 0;
  }
  pRender->pPatchLocationListOut += N;
  A->Renders++;
  if ((A->Renders <= 3) || ((A->Renders % 500) == 0)) {
    RLOG ("exynoswddm: render %lu: %lu job chains, %u allocations", A->Renders, Count, pRender->AllocationListSize);
  }
  return STATUS_SUCCESS;
}

/**
  Present: Blt / ColorFill between allocations on the CPU (allocation
  pointers straight from the allocation list), Flip: nothing to copy here
  (SetVidPnSourceAddress queues the flip copy).
**/
static NTSTATUS APIENTRY
RnPresent (IN_CONST_HANDLE hContext, INOUT_PDXGKARG_PRESENT pPresent)
{
  PUCHAR           Out = (PUCHAR)pPresent->pDmaBuffer;
  EX_CMD_BLT_ARGS  *B = (EX_CMD_BLT_ARGS *)Out;
  EX_ALLOCATION    *Src;
  EX_ALLOCATION    *Dst;
  ULONG            Rects;
  ULONG            Need;
  ULONG            Index;
  ADAPTER          *A = AdapterOf (hContext);

  if (!pPresent->Flags.Blt && !pPresent->Flags.ColorFill) {
    EX_CMD  *C = (EX_CMD *)Out;

    //
    // A flip (FlipOnVSyncMmIo): dxgkrnl may give no DMA buffer at all -
    // the flip itself comes through DxgkDdiSetVidPnSourceAddress.  Asking
    // for a bigger buffer here failed every DWM present
    // (STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER in DxgKrnl's present event).
    //
    if ((Out == NULL) || (pPresent->DmaSize < sizeof (*C))) {
      return STATUS_SUCCESS;
    }
    C->Op   = EX_CMD_NOP;
    C->Size = sizeof (*C);
    pPresent->pDmaBuffer = Out + sizeof (*C);
    return STATUS_SUCCESS;
  }
  Dst = (EX_ALLOCATION *)pPresent->pAllocationList[DXGK_PRESENT_DESTINATION_INDEX].hDeviceSpecificAllocation;
  Src = pPresent->Flags.ColorFill ? NULL :
        (EX_ALLOCATION *)pPresent->pAllocationList[DXGK_PRESENT_SOURCE_INDEX].hDeviceSpecificAllocation;
  if ((Dst == NULL) || (!pPresent->Flags.ColorFill && (Src == NULL))) {
    return STATUS_INVALID_PARAMETER;
  }
  Rects = (pPresent->SubRectCnt != 0) ? min (pPresent->SubRectCnt, EX_MAX_RECTS) : 1;
  Need  = (FIELD_OFFSET (EX_CMD_BLT_ARGS, Rects) + Rects * sizeof (EX_RECT) + 7) & ~7u;
  if (pPresent->DmaSize < Need) {
    return STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER;
  }
  RtlZeroMemory (B, Need);
  B->Hdr.Op      = pPresent->Flags.ColorFill ? EX_CMD_COLORFILL : EX_CMD_BLT;
  B->Hdr.Size    = Need;
  B->Dst.Index   = EX_NO_INDEX;
  B->Dst.Address = (UINT64)(ULONG_PTR)Dst;
  B->DstPitch    = (Dst->Info.Pitch != 0) ? Dst->Info.Pitch : SurfacePitch (Dst->Info.Width);
  B->DstWidth    = Dst->Info.Width;
  B->DstHeight   = Dst->Info.Height;
  if (Src != NULL) {
    B->Src.Index   = EX_NO_INDEX;
    B->Src.Address = (UINT64)(ULONG_PTR)Src;
    B->SrcPitch    = (Src->Info.Pitch != 0) ? Src->Info.Pitch : SurfacePitch (Src->Info.Width);
    B->SrcWidth    = Src->Info.Width;
    B->SrcHeight   = Src->Info.Height;
  }
  B->OffsetX   = pPresent->SrcRect.left - pPresent->DstRect.left;
  B->OffsetY   = pPresent->SrcRect.top - pPresent->DstRect.top;
  B->Color     = pPresent->Color;
  B->RectCount = Rects;
  for (Index = 0; Index < Rects; Index++) {
    CONST RECT  *R = (pPresent->SubRectCnt != 0) ? &pPresent->pDstSubRects[Index] : &pPresent->DstRect;

    B->Rects[Index].Left   = R->left;
    B->Rects[Index].Top    = R->top;
    B->Rects[Index].Right  = R->right;
    B->Rects[Index].Bottom = R->bottom;
  }
  if (pPresent->SubRectCnt > EX_MAX_RECTS) {
    B->RectCount       = 1;
    B->Rects[0].Left   = pPresent->DstRect.left;
    B->Rects[0].Top    = pPresent->DstRect.top;
    B->Rects[0].Right  = pPresent->DstRect.right;
    B->Rects[0].Bottom = pPresent->DstRect.bottom;
  }
  //
  // One patch location per allocation the copy touches: that is what
  // makes VidMm page them in (map them) for this submission.
  //
  for (Index = DXGK_PRESENT_SOURCE_INDEX; Index <= DXGK_PRESENT_DESTINATION_INDEX; Index++) {
    if ((pPresent->PatchLocationListOutSize == 0) ||
        (pPresent->pAllocationList[Index].hDeviceSpecificAllocation == NULL)) {
      continue;
    }
    RtlZeroMemory (pPresent->pPatchLocationListOut, sizeof (*pPresent->pPatchLocationListOut));
    pPresent->pPatchLocationListOut->AllocationIndex = Index;
    pPresent->pPatchLocationListOut->SlotId          = Index;
    pPresent->pPatchLocationListOut++;
    pPresent->PatchLocationListOutSize--;
  }
  pPresent->pDmaBuffer = Out + Need;
  if (pPresent->Flags.ColorFill) {
    A->PresentFills++;
  } else {
    A->PresentBlts++;
  }
  if ((A->Presents++ < 3) || ((A->Presents % 500) == 0)) {
    RLOG ("exynoswddm: present %lu %s %ux%u -> %ux%u, %lu rects", A->Presents,
          pPresent->Flags.ColorFill ? "fill" : "blt", (Src != NULL) ? Src->Info.Width : 0,
          (Src != NULL) ? Src->Info.Height : 0, Dst->Info.Width, Dst->Info.Height, Rects);
  }
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnPatch (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_PATCH pPatch)
{
  UNREFERENCED_PARAMETER (hAdapter);
  UNREFERENCED_PARAMETER (pPatch);
  return STATUS_SUCCESS;
}

/**
  DMA buffers live in system memory (segment 0): VidMm hands over only
  their physical address; the CPU address is the system mapping of that
  (physically contiguous) memory, checked at both ends.
**/
static PVOID
DmaVa (PHYSICAL_ADDRESS Pa, ULONG Start, ULONG Bytes)
{
  PHYSICAL_ADDRESS  At;
  PUCHAR            Va;

  At.QuadPart = Pa.QuadPart + Start;
  Va = (PUCHAR)MmGetVirtualForPhysical (At);
  if ((Va == NULL) || (Bytes == 0)) {
    return Va;
  }
  if (MmGetPhysicalAddress (Va + Bytes - 1).QuadPart != At.QuadPart + Bytes - 1) {
    RLOG ("exynoswddm: DMA buffer at 0x%08lx (+%lu bytes) not contiguous in the system mapping", At.LowPart, Bytes);
    return NULL;
  }
  return Va;
}

static NTSTATUS APIENTRY
RnSubmitCommand (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_SUBMITCOMMAND pSubmitCommand)
{
  ADAPTER     *A = (ADAPTER *)hAdapter;
  SUBMISSION  S;

  RtlZeroMemory (&S, sizeof (S));
  S.Kind    = SUB_DMA;
  S.Bytes   = pSubmitCommand->DmaBufferSubmissionEndOffset - pSubmitCommand->DmaBufferSubmissionStartOffset;
  S.Dma     = (S.Bytes != 0) ? DmaVa (pSubmitCommand->DmaBufferPhysicalAddress,
                                      pSubmitCommand->DmaBufferSubmissionStartOffset, S.Bytes) : NULL;
  S.FenceId = pSubmitCommand->SubmissionFenceId;
  S.Paging  = (BOOLEAN)pSubmitCommand->Flags.Paging;
  if ((S.Dma == NULL) && (S.Bytes != 0)) {
    RLOG ("exynoswddm: submission without a CPU address (0x%08lx, segment %u)",
          pSubmitCommand->DmaBufferPhysicalAddress.LowPart, pSubmitCommand->DmaBufferSegmentId);
  }
  if (!Enqueue (A, &S)) {
    return STATUS_INSUFFICIENT_RESOURCES;
  }
  A->Submissions++;
  A->LastSubmitted = S.FenceId;
  if ((A->Submissions <= 3) || ((A->Submissions % 500) == 0)) {
    RLOG ("exynoswddm: submission %lu: fence %lu, %lu bytes%s", A->Submissions, S.FenceId, S.Bytes,
          S.Paging ? " (paging)" : "");
  }
  return STATUS_SUCCESS;
}

typedef struct {
  ADAPTER  *A;
  ULONG    PreemptionFenceId;
} PREEMPT_SYNC;

static BOOLEAN
PreemptSync (PVOID Context)
{
  PREEMPT_SYNC                     *P = (PREEMPT_SYNC *)Context;
  DXGKARGCB_NOTIFY_INTERRUPT_DATA  Notify;

  RtlZeroMemory (&Notify, sizeof (Notify));
  Notify.InterruptType                     = DXGK_INTERRUPT_DMA_PREEMPTED;
  Notify.DmaPreempted.PreemptionFenceId    = P->PreemptionFenceId;
  Notify.DmaPreempted.LastCompletedFenceId = P->A->LastCompleted;
  P->A->Dxgk.DxgkCbNotifyInterrupt (P->A->Dxgk.DeviceHandle, &Notify);
  return TRUE;
}

/**
  Worker thread, between submissions (DMA-buffer granularity): the
  submissions not started yet are dropped - the scheduler resubmits them -
  then the preemption is reported with the fence that really completed
  last.  Reporting at once while the queue kept running completed fences
  the scheduler had written off (bugcheck 0x119, 1).
**/
static VOID
HandlePreempt (ADAPTER *A)
{
  PREEMPT_SYNC  P;
  BOOLEAN       Ret;
  KIRQL         Irql;
  ULONG         Q;
  ULONG         Dropped = 0;

  KeAcquireSpinLock (&A->QueueLock, &Irql);
  for (Q = A->QueueHead; Q != A->QueueTail; Q++) {
    if (A->Queue[Q % SUBMIT_QUEUE].Kind == SUB_DMA) {
      A->Queue[Q % SUBMIT_QUEUE].Kind = SUB_DROPPED;
      Dropped++;
    }
  }
  P.PreemptionFenceId = A->PreemptFence;
  InterlockedExchange (&A->PreemptPending, 0);
  KeReleaseSpinLock (&A->QueueLock, Irql);
  P.A = A;
  A->Dxgk.DxgkCbSynchronizeExecution (A->Dxgk.DeviceHandle, PreemptSync, &P, 0, &Ret);
  A->Dxgk.DxgkCbQueueDpc (A->Dxgk.DeviceHandle);
  if (A->Preemptions++ < 10) {
    RLOG ("exynoswddm: preempted (fence %lu): %lu queued submissions dropped, last completed %lu", P.PreemptionFenceId,
          Dropped, A->LastCompleted);
  }
}

static NTSTATUS APIENTRY
RnPreemptCommand (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_PREEMPTCOMMAND pPreemptCommand)
{
  ADAPTER  *A = (ADAPTER *)hAdapter;

  A->PreemptFence = pPreemptCommand->PreemptionFenceId;
  InterlockedExchange (&A->PreemptPending, 1);
  KeSetEvent (&A->QueueEvent, IO_NO_INCREMENT, FALSE);     // the worker reports it at the next boundary
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnQueryCurrentFence (IN_CONST_HANDLE hAdapter, INOUT_PDXGKARG_QUERYCURRENTFENCE pCurrentFence)
{
  pCurrentFence->CurrentFence = ((ADAPTER *)hAdapter)->LastCompleted;
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnResetFromTimeout (IN_CONST_HANDLE hAdapter)
{
  ADAPTER  *A = (ADAPTER *)hAdapter;

  OsLog ("exynoswddm: reset from timeout (submitted %lu, completed %lu, queue %lu..%lu, running %ld, last JS 0x%08lx)",
         A->LastSubmitted, A->LastCompleted, A->QueueHead, A->QueueTail, A->Running, A->LastJsStatus);
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnRestartFromTimeout (IN_CONST_HANDLE hAdapter)
{
  UNREFERENCED_PARAMETER (hAdapter);
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnCollectDbgInfo (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_COLLECTDBGINFO pCollectDbgInfo)
{
  UNREFERENCED_PARAMETER (hAdapter);

  if ((pCollectDbgInfo->pBuffer != NULL) && (pCollectDbgInfo->BufferSize != 0)) {
    RtlZeroMemory (pCollectDbgInfo->pBuffer, pCollectDbgInfo->BufferSize);
  }
  return STATUS_SUCCESS;
}

//
// ---------------------------------------------------------------------------
// Escapes (ExMw.h)
// ---------------------------------------------------------------------------
//

static VOID
GpuParams (ADAPTER *A, EXMW_ESCAPE_GPU_INFO_DATA *G)
{
  EX_GPU_INFO  I;
  ULONG        N;

  G->Count = EXMW_GPU_PARAMS;
  G->Valid = 0;
  RtlZeroMemory (G->Value, sizeof (G->Value));
  if (!EXW_GPU_MODE || !GpuConnect (A)) {
    return;                                    // present only
  }
  A->Gpu.GetInfo (A->Gpu.Header.Context, &I);
  if (!I.Up) {
    return;
  }
  G->Value[EXMW_PARAM_GPU_PROD_ID]          = I.GpuId >> 16;
  G->Value[EXMW_PARAM_GPU_REVISION]         = I.GpuId & 0xFFFF;
  G->Value[EXMW_PARAM_SHADER_PRESENT]       = I.ShaderPresent;
  G->Value[EXMW_PARAM_TILER_PRESENT]        = I.TilerPresent;
  G->Value[EXMW_PARAM_L2_PRESENT]           = I.L2Present;
  G->Value[EXMW_PARAM_STACK_PRESENT]        = I.StackPresent;
  G->Value[EXMW_PARAM_AS_PRESENT]           = I.AsPresent;
  G->Value[EXMW_PARAM_JS_PRESENT]           = I.JsPresent;
  G->Value[EXMW_PARAM_L2_FEATURES]          = I.L2Features;
  G->Value[EXMW_PARAM_CORE_FEATURES]        = I.CoreFeatures;
  G->Value[EXMW_PARAM_TILER_FEATURES]       = I.TilerFeatures;
  G->Value[EXMW_PARAM_MEM_FEATURES]         = I.MemFeatures;
  G->Value[EXMW_PARAM_MMU_FEATURES]         = I.MmuFeatures;
  G->Value[EXMW_PARAM_THREAD_FEATURES]      = I.ThreadFeatures;
  G->Value[EXMW_PARAM_MAX_THREADS]          = I.ThreadMaxThreads;
  G->Value[EXMW_PARAM_THREAD_MAX_WORKGROUP] = I.ThreadMaxWorkgroupSize;
  G->Value[EXMW_PARAM_THREAD_MAX_BARRIER]   = I.ThreadMaxBarrierSize;
  G->Value[EXMW_PARAM_COHERENCY_FEATURES]   = I.CoherencyFeatures;
  for (N = 0; N < 4; N++) {
    G->Value[EXMW_PARAM_TEXTURE_FEATURES0 + N] = I.TextureFeatures[N];
  }
  for (N = 0; N < 16; N++) {
    G->Value[EXMW_PARAM_JS_FEATURES0 + N] = I.JsFeatures[N];
  }
  G->Value[EXMW_PARAM_NR_CORE_GROUPS]   = 1;
  G->Value[EXMW_PARAM_THREAD_TLS_ALLOC] = I.ThreadTlsAlloc;
  G->Valid = (1ull << (EXMW_PARAM_THREAD_TLS_ALLOC + 1)) - 1;     // 0 .. THREAD_TLS_ALLOC
}

static EX_ALLOCATION *
AllocFromHandle (ADAPTER *A, UINT32 Handle)
{
  DXGKARGCB_GETHANDLEDATA  Get;

  RtlZeroMemory (&Get, sizeof (Get));
  Get.hObject = Handle;
  Get.Type    = DXGK_HANDLE_ALLOCATION;
  return (EX_ALLOCATION *)A->Dxgk.DxgkCbGetHandleData (&Get);
}

/**
  ScanBuf[0] (physically contiguous, in the firmware carve-out) mapped into
  the GPU address space once, for the UMD's GPU blit of DWM's frames
  (exynos-woa H4b); 0 if it cannot be.
**/
/**
  ScanGpuVa onto ScanBuf[Index] (both physically contiguous, in the
  firmware carve-out).
**/
static NTSTATUS
ScanoutMapTo (ADAPTER *A, ULONGLONG Va, ULONG Index)
{
  ULONG  Pages = (ULONG)(FB_BYTES >> PAGE_SHIFT);
  ULONG  I;

  for (I = 0; I < Pages; I++) {
    A->ScanPfn[I] = (PFN_NUMBER)((A->ScanBuf[Index].Pa.QuadPart >> PAGE_SHIFT) + I);
  }
  return A->Gpu.MapPages (A->Gpu.Header.Context, Va, A->ScanPfn, Pages);
}

/**
  The scan-out in the GPU address space, for the UMD's GPU blit of DWM's
  frames (exynos-woa H4b), mapped once onto the scan-out buffer that is
  off screen; every no-copy flip shows that buffer and moves the mapping
  to the other one (ExecFlip), so the blit never writes what the panel
  reads.  0 if it cannot be mapped.
**/
static ULONGLONG
ScanoutGpuVa (ADAPTER *A)
{
  ULONG        Pages = (ULONG)(FB_BYTES >> PAGE_SHIFT);
  ULONG        Granules = (ULONG)((FB_BYTES + VA_GRANULE - 1) / VA_GRANULE);
  ULONG        First;
  ULONG        Back;
  ULONGLONG    Va;
  NTSTATUS     Status;

  if ((A->ScanGpuVa != 0) || !EXW_GPU_MODE || (A->CarveVa == NULL) || !GpuConnect (A)) {
    return A->ScanGpuVa;
  }
  ExAcquireFastMutex (&A->ScanMapLock);            // one mapping, whichever process asks first
  if (A->ScanGpuVa != 0) {
    ExReleaseFastMutex (&A->ScanMapLock);
    return A->ScanGpuVa;
  }
  if (A->ScanPfn == NULL) {
    A->ScanPfn = ExAllocatePoolWithTag (NonPagedPool, Pages * sizeof (PFN_NUMBER), TAG_VA);
  }
  if (A->ScanPfn == NULL) {
    ExReleaseFastMutex (&A->ScanMapLock);
    return 0;
  }
  ExAcquireFastMutex (&A->VaLock);
  First = RtlFindClearBitsAndSet (&A->VaMap, Granules, 0);
  ExReleaseFastMutex (&A->VaLock);
  if (First == 0xFFFFFFFF) {
    ExReleaseFastMutex (&A->ScanMapLock);
    return 0;
  }
  Va     = EX_GPU_VA_START + (ULONGLONG)First * VA_GRANULE;
  Back   = (A->ScanFront == 0) ? 1 : 0;
  Status = ScanoutMapTo (A, Va, Back);
  OsLog ("exynoswddm: scan-out buffer %lu (0x%08lx, %lu pages) at GPU 0x%08lx - 0x%08lx", Back,
         A->ScanBuf[Back].Pa.LowPart, Pages, (ULONG)Va, Status);
  if (!NT_SUCCESS (Status)) {
    ExAcquireFastMutex (&A->VaLock);
    RtlClearBits (&A->VaMap, First, Granules);
    ExReleaseFastMutex (&A->VaLock);
    ExReleaseFastMutex (&A->ScanMapLock);
    return 0;
  }
  A->ScanVaFirst = First;
  A->ScanGpuBack = Back;
  A->ScanGpuVa   = Va;
  ExReleaseFastMutex (&A->ScanMapLock);
  return Va;
}

static NTSTATUS APIENTRY
RnEscape (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_ESCAPE pEscape)
{
  ADAPTER  *A = (ADAPTER *)hAdapter;
  UINT32   Code;
  ULONG    Size = pEscape->PrivateDriverDataSize;
  PVOID    Data = pEscape->pPrivateDriverData;

  if ((Data == NULL) || (Size < sizeof (UINT32))) {
    return STATUS_INVALID_PARAMETER;
  }
  Code = *(UINT32 *)Data;
  switch (Code) {
    case EXMW_ESCAPE_ALLOC_INFO: {
      EXMW_ESCAPE_ALLOC_INFO_DATA  *E = (EXMW_ESCAPE_ALLOC_INFO_DATA *)Data;
      EX_ALLOCATION                *X;

      if (Size < sizeof (*E)) {
        return STATUS_BUFFER_TOO_SMALL;
      }
      X = AllocFromHandle (A, E->hAllocation);
      if (X == NULL) {
        return STATUS_INVALID_HANDLE;
      }
      E->GpuVa = X->GpuVa;
      E->Size  = (UINT64)X->Pages << PAGE_SHIFT;
      return STATUS_SUCCESS;
    }
    case EXMW_ESCAPE_GPU_INFO: {
      EXMW_ESCAPE_GPU_INFO_DATA  *G = (EXMW_ESCAPE_GPU_INFO_DATA *)Data;
      static ULONG               Logs;

      if (Size < sizeof (*G)) {
        return STATUS_BUFFER_TOO_SMALL;
      }
      GpuParams (A, G);
      if (Logs++ < 5) {
        OsLog ("exynoswddm: UMD asks for the GPU: %s", (G->Valid != 0) ? "GPU mode" : "present only");
      }
      return STATUS_SUCCESS;
    }
    case EXMW_ESCAPE_STATS: {
      EXMW_ESCAPE_STATS_DATA  *S = (EXMW_ESCAPE_STATS_DATA *)Data;

      if (Size < FIELD_OFFSET (EXMW_ESCAPE_STATS_DATA, JobsDone)) {
        return STATUS_BUFFER_TOO_SMALL;
      }
      RtlZeroMemory ((PUCHAR)S + sizeof (UINT32), min (Size, sizeof (*S)) - sizeof (UINT32));
      S->GpuUp              = A->GpuOk;
      S->Renders            = A->Renders;
      S->Presents           = A->Presents;
      S->Submits            = A->Submissions;
      S->LastSubmittedFence = A->LastSubmitted;
      S->LastCompletedFence = A->LastCompleted;
      S->Queued             = A->QueueTail - A->QueueHead;
      S->Running            = (UINT32)A->Running;
      if (Size >= sizeof (*S)) {
        S->JobsDone     = A->JobsDone;
        S->JobsFailed   = A->JobsFailed;
        S->LastJsStatus = A->LastJsStatus;
        S->Maps         = A->Maps;
        S->MapFails     = A->MapFails;
        S->Unmaps       = A->Unmaps;
        S->Allocations  = A->AllocCount;
        S->PresentBlts  = A->PresentBlts;
        S->PresentFills = A->PresentFills;
      }
      return STATUS_SUCCESS;
    }
    case EXMW_ESCAPE_SCANOUT: {
      EXMW_ESCAPE_SCANOUT_DATA  *S = (EXMW_ESCAPE_SCANOUT_DATA *)Data;

      if (Size < sizeof (*S)) {
        return STATUS_BUFFER_TOO_SMALL;
      }
      S->GpuVa  = ScanoutGpuVa (A);            // 0: not mapped, presents stay CPU copies
      if ((S->GpuVa != 0) && (S->hAllocation != 0)) {
        EX_ALLOCATION  *X = AllocFromHandle (A, S->hAllocation);

        if (X == NULL) {
          return STATUS_INVALID_HANDLE;
        }
        if (!X->GpuPresented) {
          X->GpuPresented = TRUE;
          RLOG ("exynoswddm: primary %p (0x%x) is GPU-presented, scan-out GPU 0x%08lx", X, S->hAllocation,
                (ULONG)S->GpuVa);
        }
      }
      S->Width  = WIDTH;
      S->Height = HEIGHT;
      S->Pitch  = WIDTH * 4;
      S->Format = D3DDDIFMT_A8R8G8B8;
      return STATUS_SUCCESS;
    }
    case EXMW_ESCAPE_CAPS: {
      EXMW_ESCAPE_CAPS_DATA  *C = (EXMW_ESCAPE_CAPS_DATA *)Data;

      if (Size < sizeof (*C)) {
        return STATUS_BUFFER_TOO_SMALL;
      }
      C->Flags = 0;
      return STATUS_SUCCESS;
    }
    case EXW_ESCAPE_SHARED_PUBLISH:
    case EXW_ESCAPE_SHARED_FETCH: {
      EXW_ESCAPE_SHARED_DATA  *E = (EXW_ESCAPE_SHARED_DATA *)Data;
      PUCHAR                  Pixels = (PUCHAR)(E + 1);
      EX_ALLOCATION           *X;
      NTSTATUS                Status = STATUS_SUCCESS;

      if ((Size < sizeof (*E)) || (E->Bytes > Size - sizeof (*E)) || (E->Bytes > EXW_SHARED_CHUNK)) {
        return STATUS_INVALID_PARAMETER;
      }
      X = AllocFromHandle (A, E->hAllocation);
      if (X == NULL) {
        return STATUS_INVALID_HANDLE;
      }
      ExAcquireFastMutex (&A->ShadowLock);
      if (Code == EXW_ESCAPE_SHARED_PUBLISH) {
        if ((E->Total == 0) || (E->Total > (X->Pages << PAGE_SHIFT)) ||
            (E->Offset > E->Total) || (E->Bytes > E->Total - E->Offset)) {
          Status = STATUS_INVALID_PARAMETER;
        } else {
          if ((X->Shadow != NULL) && (X->ShadowBytes != E->Total)) {
            ExFreePoolWithTag (X->Shadow, 'dSxE');
            X->Shadow = NULL;
          }
          if (X->Shadow == NULL) {
            X->Shadow      = ExAllocatePoolWithTag (PagedPool, E->Total, 'dSxE');
            X->ShadowBytes = (X->Shadow != NULL) ? E->Total : 0;
            if (X->Shadow != NULL) {
              RtlZeroMemory (X->Shadow, E->Total);
            }
          }
          if (X->Shadow == NULL) {
            Status = STATUS_INSUFFICIENT_RESOURCES;
          } else {
            RtlCopyMemory (X->Shadow + E->Offset, Pixels, E->Bytes);
            if (E->Flags & EXW_SHARED_LAST) {
              X->ShadowGen++;
              if (X->ShadowGen == 0) {
                X->ShadowGen = 1;
              }
              if ((A->ShadowPublishes++ < 5) || ((A->ShadowPublishes % 500) == 0)) {
                RLOG ("exynoswddm: shared publish %p %lu bytes, generation %lu (%lu publishes)", X, X->ShadowBytes,
                      X->ShadowGen, A->ShadowPublishes);
              }
            }
          }
        }
        E->Gen = X->ShadowGen;
      } else {
        E->Total = X->ShadowBytes;
        E->Gen   = (X->Shadow != NULL) ? X->ShadowGen : 0;
        if ((X->Shadow == NULL) || (E->Offset >= X->ShadowBytes)) {
          E->Bytes = 0;
        } else {
          E->Bytes = min (E->Bytes, X->ShadowBytes - E->Offset);
          RtlCopyMemory (Pixels, X->Shadow + E->Offset, E->Bytes);
          if ((E->Offset == 0) && ((A->ShadowFetches++ < 5) || ((A->ShadowFetches % 500) == 0))) {
            RLOG ("exynoswddm: shared fetch %p generation %lu (%lu fetches)", X, E->Gen, A->ShadowFetches);
          }
        }
      }
      ExReleaseFastMutex (&A->ShadowLock);
      return Status;
    }
    default:
      return STATUS_NOT_SUPPORTED;
  }
}

static NTSTATUS APIENTRY
RnSetPalette (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_SETPALETTE pSetPalette)
{
  UNREFERENCED_PARAMETER (hAdapter);
  UNREFERENCED_PARAMETER (pSetPalette);
  return STATUS_SUCCESS;
}

/**
  Flip to a primary: queued to the worker (after everything submitted
  before it), which copies it into the back scan-out buffer and flips.
**/
static NTSTATUS APIENTRY
RnSetVidPnSourceAddress (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_SETVIDPNSOURCEADDRESS pSetVidPnSourceAddress)
{
  ADAPTER        *A = (ADAPTER *)hAdapter;
  ULONGLONG      Address = (ULONGLONG)pSetVidPnSourceAddress->PrimaryAddress.QuadPart;
  EX_ALLOCATION  *X = (EX_ALLOCATION *)pSetVidPnSourceAddress->hAllocation;
  SUBMISSION     S;

  if ((X == NULL) || (X->Mdl == NULL)) {
    X = AllocBySegAddress (A, Address);
  }
  if ((pSetVidPnSourceAddress->PrimarySegment != APERTURE_ID) || (X == NULL)) {
    RLOG ("exynoswddm: SetVidPnSourceAddress segment %u address 0x%08lx: no resident primary",
          pSetVidPnSourceAddress->PrimarySegment, (ULONG)Address);
    return STATUS_SUCCESS;
  }
  if (A->Flips < 3) {
    RLOG ("exynoswddm: flip to primary %p (0x%08lx)%s", X, (ULONG)Address,
          pSetVidPnSourceAddress->Flags.ModeChange ? " (mode change)" : "");
  }
  RtlZeroMemory (&S, sizeof (S));
  S.Kind           = SUB_FLIP;
  S.Primary        = X;
  S.PrimaryAddress = Address;
  Enqueue (A, &S);
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnRecommendVidPnTopology (IN_CONST_HANDLE hAdapter,
                          IN_CONST_PDXGKARG_RECOMMENDVIDPNTOPOLOGY_CONST pRecommendVidPnTopology)
{
  UNREFERENCED_PARAMETER (hAdapter);
  UNREFERENCED_PARAMETER (pRecommendVidPnTopology);
  return STATUS_GRAPHICS_NO_RECOMMENDED_VIDPN_TOPOLOGY;
}

static NTSTATUS APIENTRY
RnStopCapture (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_STOPCAPTURE pStopCapture)
{
  UNREFERENCED_PARAMETER (hAdapter);
  UNREFERENCED_PARAMETER (pStopCapture);
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnAcquireSwizzlingRange (IN_CONST_HANDLE hAdapter, INOUT_PDXGKARG_ACQUIRESWIZZLINGRANGE pAcquireSwizzlingRange)
{
  UNREFERENCED_PARAMETER (hAdapter);
  UNREFERENCED_PARAMETER (pAcquireSwizzlingRange);
  return STATUS_NOT_SUPPORTED;
}

static NTSTATUS APIENTRY
RnReleaseSwizzlingRange (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_RELEASESWIZZLINGRANGE pReleaseSwizzlingRange)
{
  UNREFERENCED_PARAMETER (hAdapter);
  UNREFERENCED_PARAMETER (pReleaseSwizzlingRange);
  return STATUS_NOT_SUPPORTED;
}

static NTSTATUS APIENTRY
RnNotifyAcpiEvent (IN_CONST_PVOID MiniportDeviceContext, IN_DXGK_EVENT_TYPE EventType, IN_ULONG Event,
                   IN_PVOID Argument, OUT_PULONG AcpiFlags)
{
  UNREFERENCED_PARAMETER (MiniportDeviceContext);
  UNREFERENCED_PARAMETER (EventType);
  UNREFERENCED_PARAMETER (Event);
  UNREFERENCED_PARAMETER (Argument);
  *AcpiFlags = 0;
  return STATUS_SUCCESS;
}

static VOID APIENTRY
RnControlEtwLogging (IN_BOOLEAN Enable, IN_ULONG Flags, IN_UCHAR Level)
{
  UNREFERENCED_PARAMETER (Enable);
  UNREFERENCED_PARAMETER (Flags);
  UNREFERENCED_PARAMETER (Level);
}

//
// Windows 8 scheduling DDIs (contexts / MultiEngineAware): one node, one
// engine, executed by the worker in order; nothing to cancel or reset.
//
static NTSTATUS APIENTRY
RnQueryDependentEngineGroup (IN_CONST_HANDLE hAdapter, INOUT_DXGKARG_QUERYDEPENDENTENGINEGROUP pQuery)
{
  UNREFERENCED_PARAMETER (hAdapter);
  pQuery->DependentNodeOrdinalMask = 0;
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnQueryEngineStatus (IN_CONST_HANDLE hAdapter, INOUT_PDXGKARG_QUERYENGINESTATUS pQuery)
{
  UNREFERENCED_PARAMETER (hAdapter);
  pQuery->EngineStatus.Responsive = 1;
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnResetEngine (IN_CONST_HANDLE hAdapter, INOUT_PDXGKARG_RESETENGINE pReset)
{
  ADAPTER  *A = (ADAPTER *)hAdapter;

  OsLog ("exynoswddm: reset engine (node %u): last completed fence %lu", pReset->NodeOrdinal, A->LastCompleted);
  pReset->LastAbortedFenceId = A->LastCompleted;
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
RnCancelCommand (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_CANCELCOMMAND pCancel)
{
  UNREFERENCED_PARAMETER (hAdapter);
  UNREFERENCED_PARAMETER (pCancel);
  return STATUS_SUCCESS;
}

VOID
RenderInitData (DRIVER_INITIALIZATION_DATA *Init)
{
  Init->DxgkDdiQueryDependentEngineGroup         = RnQueryDependentEngineGroup;
  Init->DxgkDdiQueryEngineStatus                 = RnQueryEngineStatus;
  Init->DxgkDdiResetEngine                       = RnResetEngine;
  Init->DxgkDdiCancelCommand                     = RnCancelCommand;
  Init->DxgkDdiNotifyAcpiEvent                   = RnNotifyAcpiEvent;
  Init->DxgkDdiControlEtwLogging                 = RnControlEtwLogging;
  Init->DxgkDdiCreateDevice                      = RnCreateDevice;
  Init->DxgkDdiDestroyDevice                     = RnDestroyDevice;
  Init->DxgkDdiCreateContext                     = RnCreateContext;
  Init->DxgkDdiDestroyContext                    = RnDestroyContext;
  Init->DxgkDdiCreateAllocation                  = RnCreateAllocation;
  Init->DxgkDdiDestroyAllocation                 = RnDestroyAllocation;
  Init->DxgkDdiDescribeAllocation                = RnDescribeAllocation;
  Init->DxgkDdiGetStandardAllocationDriverData   = RnGetStandardAllocationDriverData;
  Init->DxgkDdiOpenAllocation                    = RnOpenAllocation;
  Init->DxgkDdiCloseAllocation                   = RnCloseAllocation;
  Init->DxgkDdiAcquireSwizzlingRange             = RnAcquireSwizzlingRange;
  Init->DxgkDdiReleaseSwizzlingRange             = RnReleaseSwizzlingRange;
  Init->DxgkDdiBuildPagingBuffer                 = RnBuildPagingBuffer;
  Init->DxgkDdiRender                            = RnRender;
  Init->DxgkDdiPresent                           = RnPresent;
  Init->DxgkDdiPatch                             = RnPatch;
  Init->DxgkDdiSubmitCommand                     = RnSubmitCommand;
  Init->DxgkDdiPreemptCommand                    = RnPreemptCommand;
  Init->DxgkDdiQueryCurrentFence                 = RnQueryCurrentFence;
  Init->DxgkDdiResetFromTimeout                  = RnResetFromTimeout;
  Init->DxgkDdiRestartFromTimeout                = RnRestartFromTimeout;
  Init->DxgkDdiCollectDbgInfo                    = RnCollectDbgInfo;
  Init->DxgkDdiEscape                            = RnEscape;
  Init->DxgkDdiSetPalette                        = RnSetPalette;
  Init->DxgkDdiSetVidPnSourceAddress             = RnSetVidPnSourceAddress;
  Init->DxgkDdiRecommendVidPnTopology            = RnRecommendVidPnTopology;
  Init->DxgkDdiStopCapture                       = RnStopCapture;
}
