/** @file
  exynoswddm internal: the adapter, shared by the display side
  (exynoswddm.c, from exynosdisp2) and the render side (render.c).

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef __EXYNOSWDDM_H__
#define __EXYNOSWDDM_H__

#include <ntddk.h>
#include <dispmprt.h>
#include "ExynosEnv.h"
#include "../../uefi/Exynos5420Pkg/Drivers/Display/ExynosDisplay.h"
#include "fimdext.h"
#include "ExCmd.h"
#include "ExMw.h"
#include "../common/ExGpuIf.h"

#define WIDTH          EXYNOS_DISP_WIDTH
#define HEIGHT         EXYNOS_DISP_HEIGHT
#define FB_BYTES       ((SIZE_T)WIDTH * HEIGHT * 4)
#define MAX_RECTS      32

//
// One aperture segment: allocations live in system memory pages, mapped
// into the GPU (exynosgpu, address space 0) at a fixed address each while
// VidMm keeps them resident.  APERTURE_BASE is nominal (the segment
// address VidMm reports for flips = APERTURE_BASE + offset).
//
#define APERTURE_ID        1
#define APERTURE_BASE      0x10000000ull
#define APERTURE_BYTES     (1024ull * 1024 * 1024)
#define VA_GRANULE         0x10000u                         // GPU address granule (64 KiB)
#define VA_GRANULES        ((ULONG)((EX_GPU_VA_END - EX_GPU_VA_START) / VA_GRANULE))
#define EXEC_WINDOW_GRANULES  ((ULONG)((16u << 20) / VA_GRANULE))   // executable allocations stay in one 16 MB window
                                                                  // (EX_GPU_VA_START is 16 MB aligned)
#define SUBMIT_QUEUE       256
#define GPU_JOB_TIMEOUT_MS 500                   // well under the 2 s TDR: a faulting job fails, no timeout recovery

typedef struct {
  PUCHAR                  Va;
  PHYSICAL_ADDRESS        Pa;
} SCANOUT;

typedef struct _EX_ALLOCATION  EX_ALLOCATION;

#define SUB_DMA   0                      // a DMA buffer range, then its fence
#define SUB_FLIP  1                      // copy a primary to the back scan-out buffer, flip
#define SUB_DROPPED 2                    // a DMA submission dropped by a preemption

typedef struct {
  ULONG                   Kind;
  PVOID                   Dma;           // CPU address of the submitted range
  ULONG                   Bytes;
  ULONG                   FenceId;
  BOOLEAN                 Paging;
  EX_ALLOCATION           *Primary;      // SUB_FLIP
  ULONGLONG               PrimaryAddress;
} SUBMISSION;

//
// An allocation (driver handle).  GpuVa is reserved for its whole life;
// Mdl / SegAddress / CpuVa are valid while it is mapped in the aperture.
//
struct _EX_ALLOCATION {
  LIST_ENTRY              Link;
  EXMW_ALLOCATION_INFO    Info;
  ULONG                   Pages;
  ULONGLONG               GpuVa;
  ULONG                   VaFirst;       // granule index
  ULONG                   VaCount;
  PMDL                    Mdl;           // VidMm's, while mapped
  ULONG                   MdlPage;
  ULONGLONG               SegAddress;    // APERTURE_BASE + offset, while mapped
  BOOLEAN                 GpuMapped;
  BOOLEAN                 MapTried;      // GpuConnect's catch-up pass
  BOOLEAN                 GpuPresented;  // the UMD blits its frames into the scan-out on the GPU (no CPU copy)
  PMDL                    CpuMdl;        // partial MDL for the CPU mapping
  PUCHAR                  CpuVa;         // write-combined, while mapped (lazy)
  PUCHAR                  Shadow;        // present-only: the creator's last published frame (paged pool)
  ULONG                   ShadowBytes;
  ULONG                   ShadowGen;
};

typedef struct _ADAPTER {
  DEVICE_OBJECT           *Pdo;
  DXGKRNL_INTERFACE       Dxgk;
  EXYNOS_DISPLAY          Disp;
  PUCHAR                  Fb;            // firmware framebuffer, Pitch * HEIGHT bytes
  UINT32                  Pitch;
  PHYSICAL_ADDRESS        FbPhys;
  BOOLEAN                 PanelOn;
  BOOLEAN                 SourceVisible;
  BOOLEAN                 NeedFull;
  BOOLEAN                 LogNextPresent;
  ULONG                   RotatedPresentLogs;
  BOOLEAN                 LastPanelShaped;
  D3DKMDT_VIDPN_PRESENT_PATH_ROTATION  Rotation;
  UCHAR                   Level;         // brightness, percent
  ULONG                   Presents;
  UINTN                   Combiner;
  //
  // Scanout: the boot picture buffer until Windows flips to a primary
  // allocation (DxgkDdiSetVidPnSourceAddress), then segment memory.
  //
  SCANOUT                 Boot;
  volatile ULONG          ScanPa;        // on screen (latched)
  volatile ULONG          PendingPa;     // programmed, latched at the next vsync (0: none)
  volatile ULONGLONG      ScanPrimary;   // segment address of the primary on screen (0: none) - CRTC vsync reports it
  volatile ULONGLONG      PendingPrimary;
  EX_ALLOCATION * volatile ScanAlloc;    // primary on screen (blts into it are copied to the front buffer too)
  volatile ULONG          ScanAllocBuf;  // the scan-out buffer ScanAlloc was flipped to (2: none)
  SCANOUT                 ScanBuf[2];    // in the firmware carve-out: flipped scan-out buffers
  ULONG                   ScanFront;     // index into ScanBuf of the buffer on screen, or 2: Boot / firmware
  volatile LONG           FlipPending;
  volatile LONG           VsyncToWindows;
  ULONG                   Vsyncs;
  SCANOUT                 Cursor;        // 64 x 64 ARGB
  UINT32                  Shape[FIMD_CURSOR_SIZE * FIMD_CURSOR_SIZE];
  INT32                   CursorX;
  INT32                   CursorY;
  BOOLEAN                 CursorVisible;
  //
  // Render side (render.c).
  //
  PUCHAR                  CarveVa;       // firmware carve-out (scan-out buffers)
  PHYSICAL_ADDRESS        CarvePa;
  ULONG                   CarveBytes;
  ULONGLONG               ScanGpuVa;     // ScanBuf[0] in the GPU address space (0: not mapped yet)
  ULONG                   ScanVaFirst;   // its VaMap granules
  ULONG                   ScanGpuBack;   // the ScanBuf ScanGpuVa maps now (the UMD's blit target, off screen)
  PFN_NUMBER              *ScanPfn;      // FB_BYTES / PAGE_SIZE entries, for remapping ScanGpuVa
  ULONG                   FlipWaitTimeouts;
  ULONG                   GpuFlips;      // flips of GPU-presented primaries (no copy)
  FAST_MUTEX              ScanMapLock;
  KSPIN_LOCK              AllocLock;     // the allocation list
  LIST_ENTRY              Allocs;
  FAST_MUTEX              VaLock;
  FAST_MUTEX              ShadowLock;    // EX_ALLOCATION Shadow*
  FAST_MUTEX              CpuLock;       // CPU mappings (CpuVa / Mdl) vs. the worker's copies and VidMm's paging
  ULONG                   FlipsSkipped;
  ULONG                   ShadowPublishes;
  ULONG                   ShadowFetches;
  RTL_BITMAP              VaMap;
  PULONG                  VaBits;
  EX_GPU_INTERFACE        Gpu;
  BOOLEAN                 GpuOk;
  ULONG                   GpuTries;
  volatile LONG           Running;
  ULONG                   Renders;
  ULONG                   PatchShort;
  ULONG                   PartialMaps;
  ULONG                   ExecLogs;
  ULONG                   FragDumps;
  volatile LONG           PreemptPending;
  ULONG                   PreemptFence;
  ULONG                   Preemptions;
  ULONG                   PresentBlts;
  ULONG                   PresentFills;
  ULONG                   Maps;
  ULONG                   MapFails;
  ULONG                   Unmaps;
  ULONG                   AllocCount;
  ULONG                   JobsDone;
  ULONG                   JobsFailed;
  ULONG                   LastJsStatus;
  ULONG                   Flips;
  ULONG                   PagingOps[16];
  ULONG                   PagingLogs;
  KSPIN_LOCK              QueueLock;
  SUBMISSION              Queue[SUBMIT_QUEUE];
  ULONG                   QueueHead;     // next to execute
  ULONG                   QueueTail;     // next free
  KEVENT                  QueueEvent;
  PKTHREAD                Worker;
  volatile LONG           WorkerStop;
  volatile ULONG          LastSubmitted;
  volatile ULONG          LastCompleted;
  volatile ULONG          CompletedToReport;
  ULONG                   Submissions;
  ULONG                   Commands;
  ULONG                   BadCommands;
  ULONG                   UmdQueries;
  ULONG                   UmdReports;
  UCHAR                   UmdDigit[17];
} ADAPTER;

//
// Segment address <-> CPU address.
//

//
// render.c
//
NTSTATUS RenderStart (ADAPTER *A);
VOID     RenderStop (ADAPTER *A);
VOID     RenderRelease (ADAPTER *A);
NTSTATUS RenderQueryAdapterInfo (ADAPTER *A, CONST DXGKARG_QUERYADAPTERINFO *Q);
VOID     RenderFillCaps (ADAPTER *A, DXGK_DRIVERCAPS *Caps);
VOID     RenderDpc (ADAPTER *A);
VOID     RenderInitData (DRIVER_INITIALIZATION_DATA *Init);
PUCHAR   ScanoutVa (ADAPTER *A);

//
// ddtrace.c
//
VOID     DdTraceInit (DRIVER_INITIALIZATION_DATA *Init);

//
// oslogfile.c
//
VOID     OsLogFileStart (VOID);
VOID     OsLogFileStop (VOID);

//
// exynoswddm.c
//
VOID     ArmVsync (ADAPTER *A, ULONG Phys);
VOID     ExRefreshRate (D3DDDI_RATIONAL *Rate);

#endif // __EXYNOSWDDM_H__
