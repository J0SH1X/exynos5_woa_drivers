/** @file
  The private data exynoswddm exchanges with its user-mode driver (Mesa's
  d3d10umd with the panfrost / softpipe backends and the Windows "malikm"
  layer from exynos-woa / wddm-mali-g52).  Written for exynoswddm from the
  layout that user-mode layer uses (its maliwddm_abi.h, ABI version 1):
  every structure here must stay byte-compatible with it.

    - allocation private data: size, flags, surface geometry;
    - a render's command buffer: Mali job chains (slot + first job);
    - escapes: allocation GPU address, GPU parameters (panfrost's DRM
      GET_PARAM numbering), counters (the UMD's idle wait reads the fence
      and queue fields), scan-out buffer, caps.

  Model: every allocation has a GPU address for its whole life; its pages
  are mapped there (Mali MMU, through exynosgpu) while VidMm keeps it
  resident in the aperture segment.  Nothing is patched.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef __EX_MW_H__
#define __EX_MW_H__

#define EXMW_ABI_VERSION        1

// Allocation flags
#define EXMW_ALLOC_NOEXEC       0x1     // not shader code
#define EXMW_ALLOC_PRIMARY      0x2     // scan-out capable
#define EXMW_ALLOC_CPU_ONDEMAND 0x4     // (WDDM 2 hint; ignored here)
// Kernel-only (standard allocations the KMD describes itself; the UMD never sets these):
#define EXMW_ALLOC_KM_GDI       0x100   // a GDI surface (D3DKMDT_STANDARDALLOCATION_GDISURFACE)
#define EXMW_ALLOC_KM_GDITYPE(f) (((f) >> 12) & 0xF)   // its D3DKMDT_GDISURFACETYPE
#define EXMW_ALLOC_KM_GDITYPE_SET(t) (((UINT32)(t) & 0xF) << 12)

#pragma pack (push, 8)

typedef struct {
  UINT32  Version;                      // EXMW_ABI_VERSION
  UINT32  Flags;                        // EXMW_ALLOC_*
  UINT64  Size;                         // bytes
  UINT32  Width;                        // runtime surfaces; 0 for buffers
  UINT32  Height;
  UINT32  Pitch;
  UINT32  Format;                       // D3DDDIFORMAT
} EXMW_ALLOCATION_INFO;                 // 32 bytes

// Render command buffer: EXMW_COMMAND entries
#define EXMW_CMD_JOB            1
#define EXMW_MAX_COMMANDS       32

typedef struct {
  UINT32  Type;                         // EXMW_CMD_JOB
  UINT32  Slot;                         // 0 fragment, 1 vertex / tiler / compute
  UINT64  Chain;                        // GPU address of the first job header
} EXMW_COMMAND;                         // 16 bytes

// Escapes: the private data starts with the code
#define EXMW_ESCAPE_ALLOC_INFO  1
#define EXMW_ESCAPE_GPU_INFO    2
#define EXMW_ESCAPE_STATS       3
#define EXMW_ESCAPE_SCANOUT     4
#define EXMW_ESCAPE_CLOCK       5
#define EXMW_ESCAPE_PANEL_POWER 6
#define EXMW_ESCAPE_CAPS        7

typedef struct {
  UINT32  Code;
  UINT32  hAllocation;                  // in: D3DKMT allocation handle
  UINT64  GpuVa;                        // out
  UINT64  Size;                         // out
} EXMW_ESCAPE_ALLOC_INFO_DATA;

// GPU parameters by panfrost DRM_PANFROST_PARAM_* number
#define EXMW_GPU_PARAMS         45
#define EXMW_PARAM_GPU_PROD_ID          0
#define EXMW_PARAM_GPU_REVISION         1
#define EXMW_PARAM_SHADER_PRESENT       2
#define EXMW_PARAM_TILER_PRESENT        3
#define EXMW_PARAM_L2_PRESENT           4
#define EXMW_PARAM_STACK_PRESENT        5
#define EXMW_PARAM_AS_PRESENT           6
#define EXMW_PARAM_JS_PRESENT           7
#define EXMW_PARAM_L2_FEATURES          8
#define EXMW_PARAM_CORE_FEATURES        9
#define EXMW_PARAM_TILER_FEATURES       10
#define EXMW_PARAM_MEM_FEATURES         11
#define EXMW_PARAM_MMU_FEATURES         12
#define EXMW_PARAM_THREAD_FEATURES      13
#define EXMW_PARAM_MAX_THREADS          14
#define EXMW_PARAM_THREAD_MAX_WORKGROUP 15
#define EXMW_PARAM_THREAD_MAX_BARRIER   16
#define EXMW_PARAM_COHERENCY_FEATURES   17
#define EXMW_PARAM_TEXTURE_FEATURES0    18          // .. 21
#define EXMW_PARAM_JS_FEATURES0         22          // .. 37
#define EXMW_PARAM_NR_CORE_GROUPS       38
#define EXMW_PARAM_THREAD_TLS_ALLOC     39
#define EXMW_PARAM_AFBC_FEATURES        40
#define EXMW_PARAM_SYSTEM_TIMESTAMP     41
#define EXMW_PARAM_SYSTEM_TIMESTAMP_FREQUENCY 42

//
// Count != 0 and Valid == 0: "present only" (no GPU for the UMD: it
// renders on the CPU, buffers and presents still go through this
// driver).  Valid bit n: Value[n] is meaningful (GPU mode).
//
typedef struct {
  UINT32  Code;
  UINT32  Count;
  UINT64  Value[EXMW_GPU_PARAMS];
  UINT64  Valid;
} EXMW_ESCAPE_GPU_INFO_DATA;

// Counters; the UMD's idle wait compares the fences and Queued / Running
typedef struct {
  UINT32  Code;
  UINT32  GpuUp;
  UINT32  Renders;
  UINT32  Presents;
  UINT32  Submits;
  UINT32  LastSubmittedFence;
  UINT32  LastCompletedFence;
  UINT32  Queued;
  UINT32  Running;
  UINT32  JobsDone;
  UINT32  JobsFailed;
  UINT32  JobsTimedOut;
  UINT32  LastJsStatus;
  UINT32  LastFaultStatus;
  UINT32  Irqs;
  UINT32  Resets;
  UINT64  LastFaultAddress;
  UINT32  PagingOps[16];
  UINT32  Maps;
  UINT32  MapFails;
  UINT32  Unmaps;
  UINT32  LastMapPages;
  UINT32  LastMapStatus;
  UINT32  Allocations;
  UINT64  LastMapVa;
  UINT64  LastAllocVa;
  UINT32  Patches;
  UINT32  PatchAllocations;
  UINT32  PatchPatchLocations;
  UINT32  PatchSegment0;
  UINT64  PatchAddress0;
  UINT32  PresentBlts;
  UINT32  PresentFills;
  UINT32  PresentSkips;
  UINT32  PresentGpu;
} EXMW_ESCAPE_STATS_DATA;

// Scan-out buffer in the GPU address space (GpuVa 0: not mapped, the UMD
// presents through the runtime and this driver copies on the CPU)
typedef struct {
  UINT32  Code;
  UINT32  hAllocation;                  // in: allocation now GPU-presented, or 0
  UINT64  GpuVa;                        // out
  UINT32  Width;
  UINT32  Height;
  UINT32  Pitch;
  UINT32  Format;                       // out
} EXMW_ESCAPE_SCANOUT_DATA;

#define EXMW_CAPS_DEFERRED_PRESENT 0x1u
#define EXMW_CAPS_REDIRECTED_PRESENT 0x2u     // windowed (DWM-redirected) presents through the runtime reach DWM:
                                               // the UMD needs no GDI fallback
typedef struct {
  UINT32  Code;
  UINT32  Flags;                        // out: EXMW_CAPS_*
} EXMW_ESCAPE_CAPS_DATA;

//
// exynoswddm's own escapes (not in the maliwddm ABI): pixels of a shared
// allocation, for present-only mode.  An opener (DWM) may not lock a
// shared allocation, so its creator publishes each frame into a shadow
// the driver keeps, and openers fetch from it.  The pixels travel in the
// escape's private data (dxgkrnl copies it in and out), EXW_SHARED_CHUNK
// bytes at a time.
//
#define EXW_ESCAPE_SHARED_PUBLISH  0x101
#define EXW_ESCAPE_SHARED_FETCH    0x102
#define EXW_SHARED_CHUNK           32768
#define EXW_SHARED_LAST            0x1      // PUBLISH: last chunk of a frame

typedef struct {
  UINT32  Code;
  UINT32  hAllocation;                  // in: D3DKMT allocation handle
  UINT32  Offset;                       // in: byte offset of this chunk
  UINT32  Bytes;                        // in (publish) / in-out (fetch): chunk bytes
  UINT32  Total;                        // publish in: frame bytes; fetch out: shadow bytes (0: none)
  UINT32  Gen;                          // out: frame generation (0: none published)
  UINT32  Flags;                        // EXW_SHARED_*
  UINT32  Reserved;
  // UCHAR Data[Bytes] follows
} EXW_ESCAPE_SHARED_DATA;

#pragma pack (pop)

C_ASSERT (sizeof (EXMW_ALLOCATION_INFO) == 32);
C_ASSERT (sizeof (EXMW_COMMAND) == 16);
C_ASSERT (sizeof (EXW_ESCAPE_SHARED_DATA) == 32);
C_ASSERT (FIELD_OFFSET (EXMW_ESCAPE_STATS_DATA, LastCompletedFence) == 24);
C_ASSERT (FIELD_OFFSET (EXMW_ESCAPE_STATS_DATA, Running) == 32);

#endif // __EX_MW_H__
