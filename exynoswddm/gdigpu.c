/** @file
  GDI on the Mali-T628: the render-pass engine (see gdigpu_layout.h).

  exynoswddm keeps GDIGPU_RESERVED_BASE..END out of its GPU address
  allocator; once exynosgpu is connected this file maps
    - the scratch (64 KB, physically contiguous, write-combined) holding
      the template from gdigpu_blob.h, at GDIGPU_BASE, and
    - 16 MB of pages for the polygon list and the tiler heap above it.
  A pass is patched into the scratch (gdigpu_build.c) and run as two job
  chains, JS1 (tiler) then JS0 (fragment), each to completion, from the
  worker thread (the same thread that runs the UMD's jobs, so passes and
  application rendering never overlap).

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include "exynoswddm.h"
#include "gdigpu_build.h"
#include "gdigpu_blob.h"

#define TAG_GDIGPU          'gGxE'
#define GDIGPU_TIMEOUT_MS   500
#define POLY_HEAP_BYTES     (GDIGPU_POLY_SIZE + GDIGPU_HEAP_SIZE)

C_ASSERT (sizeof (mGdiGpuBlob) <= GDIGPU_SCRATCH_SIZE);
C_ASSERT (GDIGPU_RESERVED_END <= EX_GPU_VA_END);
C_ASSERT (GDIGPU_TEMP_BASE >= GDIGPU_BASE + GDIGPU_SCRATCH_SIZE);
C_ASSERT (GDIGPU_TEMP_BASE + GDIGPU_TEMP_SIZE <= GDIGPU_POLY_BASE);
C_ASSERT ((GDIGPU_BASE / (16u << 20)) == ((GDIGPU_BASE + GDIGPU_SCRATCH_SIZE - 1) / (16u << 20)));   // one exec window

VOID
GdiGpuReserveVa (ADAPTER *A)
{
  RtlSetBits (&A->VaMap, (ULONG)((GDIGPU_RESERVED_BASE - EX_GPU_VA_START) / VA_GRANULE),
              (ULONG)((GDIGPU_RESERVED_END - GDIGPU_RESERVED_BASE) / VA_GRANULE));
}

VOID
GdiGpuRelease (ADAPTER *A)
{
  if (A->GdiGpuMapped && A->GpuOk) {
    A->Gpu.UnmapPages (A->Gpu.Header.Context, GDIGPU_BASE, GDIGPU_SCRATCH_SIZE >> PAGE_SHIFT);
    A->Gpu.UnmapPages (A->Gpu.Header.Context, GDIGPU_POLY_BASE, POLY_HEAP_BYTES >> PAGE_SHIFT);
    if (A->GdiTempMdl != NULL) {
      A->Gpu.UnmapPages (A->Gpu.Header.Context, GDIGPU_TEMP_BASE, GDIGPU_TEMP_SIZE >> PAGE_SHIFT);
    }
  }
  A->GdiGpuMapped = FALSE;
  A->GdiGpuOk     = FALSE;
  if (A->GdiScratch != NULL) {
    MmFreeContiguousMemorySpecifyCache (A->GdiScratch, GDIGPU_SCRATCH_SIZE, MmWriteCombined);
    A->GdiScratch = NULL;
  }
  if (A->GdiPolyMdl != NULL) {
    MmFreePagesFromMdl (A->GdiPolyMdl);
    ExFreePool (A->GdiPolyMdl);
    A->GdiPolyMdl = NULL;
  }
  if (A->GdiTempMdl != NULL) {
    MmFreePagesFromMdl (A->GdiTempMdl);
    ExFreePool (A->GdiTempMdl);
    A->GdiTempMdl = NULL;
  }
}

/**
  Called once exynosgpu is connected (PASSIVE_LEVEL).  Failure only means
  GDI keeps running on the CPU.
**/
VOID
GdiGpuStart (ADAPTER *A)
{
  PHYSICAL_ADDRESS  Low;
  PHYSICAL_ADDRESS  High;
  PHYSICAL_ADDRESS  Skip;
  PHYSICAL_ADDRESS  Pa;
  PFN_NUMBER        Pfn[GDIGPU_SCRATCH_SIZE >> PAGE_SHIFT];
  ULONG             I;
  NTSTATUS          Status;

  if (A->GdiGpuOk || A->GdiGpuTried || !gGdiGpu || !A->GpuOk) {
    return;
  }
  A->GdiGpuTried = TRUE;
  Low.QuadPart  = 0;
  High.QuadPart = 0xFFFFFFFF;
  Skip.QuadPart = 0;
  A->GdiScratch = MmAllocateContiguousMemorySpecifyCache (GDIGPU_SCRATCH_SIZE, Low, High, Skip, MmWriteCombined);
  if (A->GdiScratch == NULL) {
    OsLog ("exynoswddm: GDI GPU: no scratch memory");
    goto Fail;
  }
  RtlZeroMemory (A->GdiScratch, GDIGPU_SCRATCH_SIZE);
  RtlCopyMemory (A->GdiScratch, mGdiGpuBlob, sizeof (mGdiGpuBlob));
  Pa = MmGetPhysicalAddress (A->GdiScratch);
  for (I = 0; I < (GDIGPU_SCRATCH_SIZE >> PAGE_SHIFT); I++) {
    Pfn[I] = (PFN_NUMBER)(Pa.QuadPart >> PAGE_SHIFT) + I;
  }
  //
  // Polygon list + tiler heap: GPU only.  Zeroed by the memory manager
  // through a cached mapping: clean those lines out before the GPU writes.
  //
  A->GdiPolyMdl = MmAllocatePagesForMdlEx (Low, High, Skip, POLY_HEAP_BYTES, MmCached, MM_ALLOCATE_FULLY_REQUIRED);
  if ((A->GdiPolyMdl == NULL) || (MmGetMdlByteCount (A->GdiPolyMdl) < POLY_HEAP_BYTES)) {
    OsLog ("exynoswddm: GDI GPU: no polygon list memory");
    goto Fail;
  }
  FlushMdlPages (A->GdiPolyMdl, 0, POLY_HEAP_BYTES >> PAGE_SHIFT);
  __builtin_arm_dsb (15);
  Status = A->Gpu.MapPages (A->Gpu.Header.Context, GDIGPU_BASE, Pfn, GDIGPU_SCRATCH_SIZE >> PAGE_SHIFT);
  if (NT_SUCCESS (Status)) {
    Status = A->Gpu.MapPages (A->Gpu.Header.Context, GDIGPU_POLY_BASE, MmGetMdlPfnArray (A->GdiPolyMdl),
                              POLY_HEAP_BYTES >> PAGE_SHIFT);
  }
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynoswddm: GDI GPU: mapping failed 0x%08lx", (ULONG)Status);
    goto Fail;
  }
  A->GdiGpuMapped = TRUE;
  A->GdiGpuOk     = TRUE;
  //
  // Optional: the temp of copies within one surface.  Without it those stay
  // on the CPU.
  //
  A->GdiTempMdl = MmAllocatePagesForMdlEx (Low, High, Skip, GDIGPU_TEMP_SIZE, MmCached, MM_ALLOCATE_FULLY_REQUIRED);
  if ((A->GdiTempMdl != NULL) && (MmGetMdlByteCount (A->GdiTempMdl) < GDIGPU_TEMP_SIZE)) {
    MmFreePagesFromMdl (A->GdiTempMdl);
    ExFreePool (A->GdiTempMdl);
    A->GdiTempMdl = NULL;
  }
  if (A->GdiTempMdl != NULL) {
    FlushMdlPages (A->GdiTempMdl, 0, GDIGPU_TEMP_SIZE >> PAGE_SHIFT);
    __builtin_arm_dsb (15);
    Status = A->Gpu.MapPages (A->Gpu.Header.Context, GDIGPU_TEMP_BASE, MmGetMdlPfnArray (A->GdiTempMdl),
                              GDIGPU_TEMP_SIZE >> PAGE_SHIFT);
    if (!NT_SUCCESS (Status)) {
      OsLog ("exynoswddm: GDI GPU: temp mapping failed 0x%08lx - copies within a surface stay on the CPU", (ULONG)Status);
      MmFreePagesFromMdl (A->GdiTempMdl);
      ExFreePool (A->GdiTempMdl);
      A->GdiTempMdl = NULL;
    }
  }
  OsLog ("exynoswddm: GDI GPU: scratch 0x%08lx (PA 0x%08lx), polygon list + heap %lu MB at 0x%08lx, temp %s", (ULONG)GDIGPU_BASE,
         Pa.LowPart, (ULONG)(POLY_HEAP_BYTES >> 20), (ULONG)GDIGPU_POLY_BASE, (A->GdiTempMdl != NULL) ? "mapped" : "none");
  return;

Fail:
  GdiGpuRelease (A);
}

/**
  Runs pass P (worker thread).  TRUE: done on the GPU.  FALSE: not run or
  failed - the caller does it on the CPU (fill and copy between different
  surfaces are idempotent, so a partly done pass is simply redone).
**/
BOOLEAN
GdiGpuRun (ADAPTER *A, CONST GDIGPU_PASS *P)
{
  ULONG  Status;

  if (!A->GdiGpuOk || (GdiGpuBuild (A->GdiScratch, &mGdiGpuSlots[0][0][0], P) != 0)) {
    return FALSE;
  }
  __builtin_arm_dsb (15);                     // descriptors (write-combined) in memory before the kick
  Status = A->Gpu.RunChain (A->Gpu.Header.Context, 1, GDIGPU_BASE + GO_WV, GDIGPU_TIMEOUT_MS);
  if (Status == 1) {
    Status = A->Gpu.RunChain (A->Gpu.Header.Context, 0, GDIGPU_BASE + GO_FRAG, GDIGPU_TIMEOUT_MS);
  }
  if (Status != 1) {
    if (A->GdiGpuFails < 3) {
      ULONG  J;

      for (J = 0; J < 3 + P->DrawCount; J++) {
        ULONG          Off = (J == 0) ? GO_WV : (J == 1) ? GO_TILER_PRE : (J == 2) ? GO_FRAG : GO_SLOT (J - 3);
        volatile ULONG *H = (volatile ULONG *)(A->GdiScratch + Off);

        OsLog ("exynoswddm: GDI GPU:   %s %lu: status 0x%08lx, first task 0x%08lx, fault 0x%08lx%08lx",
               (J == 0) ? "write-value" : (J == 1) ? "preload" : (J == 2) ? "fragment" : "draw", (J >= 3) ? J - 3 : 0,
               H[0], H[1], H[3], H[2]);
      }
    }
    A->GdiGpuFails++;
    if (A->GdiGpuFails <= 10) {
      OsLog ("exynoswddm: GDI GPU: pass %ux%u (%u draws, first op %u) failed, JS_STATUS 0x%lx%s", P->FbW, P->FbH,
             P->DrawCount, P->Draws[0].Op, Status, (A->GdiGpuFails >= 10) ? " - GPU GDI off" : "");
    }
    if (A->GdiGpuFails >= 10) {
      A->GdiGpuOk = FALSE;                    // keep the desktop working: CPU from now on
    }
    return FALSE;
  }
  A->GdiGpuPasses++;
  return TRUE;
}
