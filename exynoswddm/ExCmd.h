/** @file
  exynoswddm private formats, shared by the kernel-mode driver
  (exynoswddm.sys) and the user-mode driver (exynosumd.dll):

    - allocation private data (UMD CreateResource / standard allocations
      -> DxgkDdiCreateAllocation),
    - DMA buffer commands (UMD command buffers, DxgkDdiRender /
      DxgkDdiPresent / DxgkDdiBuildPagingBuffer -> executed at submission),
    - the UMD's adapter query (DXGKQAITYPE_UMDRIVERPRIVATE) and log escape.

  Addresses in commands are memory segment addresses (the segment's
  physical address range); allocation references (EX_REF) are resolved by
  DxgkDdiPatch from the submission's allocation list.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef __EX_CMD_H__
#define __EX_CMD_H__

#define EX_ALLOC_MAGIC       0x43414558u    // 'EXAC'
#define EX_UMD_MAGIC         0x444D5545u    // 'EUMD'

//
// Allocation private data.
//
#define EX_ALLOC_TYPE_SURFACE        0       // UMD resource
#define EX_ALLOC_TYPE_PRIMARY        1       // shared primary (scanout)
#define EX_ALLOC_TYPE_SHADOW         2       // kernel GDI shadow
#define EX_ALLOC_TYPE_STAGING        3       // kernel staging
#define EX_ALLOC_TYPE_GDI            4       // GDI surface
#define EX_ALLOC_TYPE_BUFFER         5       // UMD linear buffer (vertex / index / constant)

typedef struct {
  UINT32  Magic;          // EX_ALLOC_MAGIC
  UINT32  Type;           // EX_ALLOC_TYPE_*
  UINT32  Width;
  UINT32  Height;
  UINT32  Pitch;          // bytes; 0 for buffers
  UINT32  Format;         // D3DDDIFORMAT
  UINT32  Size;           // bytes (buffers: the size; surfaces: Pitch * Height)
  UINT32  VidPnSourceId;
} EX_ALLOC_DESC;

//
// DMA buffer commands: a sequence of EX_CMD headers, each followed by its
// payload, Size = header + payload bytes (multiple of 8).
//
#define EX_CMD_NOP           0
#define EX_CMD_COPY          1       // segment -> segment, linear
#define EX_CMD_FILL          2       // segment, 32-bit pattern
#define EX_CMD_PAGE_IN       3       // MDL -> segment (paging)
#define EX_CMD_PAGE_OUT      4       // segment -> MDL (paging)
#define EX_CMD_BLT           5       // 32 bpp rectangle copy, segment -> segment
#define EX_CMD_COLORFILL     6       // 32 bpp rectangle fill
#define EX_CMD_GPU_JOB       7       // Mali job chain (GPU VA), slot (later)
#define EX_CMD_GDI           8       // GDI hardware acceleration operation (DxgkDdiRenderKm), EX_CMD_GDI_ARGS

typedef struct {
  UINT32  Op;
  UINT32  Size;
} EX_CMD;

//
// Allocation references: Index is an allocation-list index (EX_NO_INDEX:
// Address is an absolute segment address).  DxgkDdiPatch sets Address =
// the allocation's segment address + Offset for the submission.
//
#define EX_NO_INDEX  0xFFFFFFFFu

typedef struct {
  UINT32  Index;
  UINT32  Reserved;
  UINT64  Offset;
  UINT64  Address;        // patched
} EX_REF;

typedef struct {
  EX_CMD  Hdr;
  EX_REF  Src;
  EX_REF  Dst;
  UINT64  Bytes;
} EX_CMD_COPY_ARGS;

typedef struct {
  EX_CMD  Hdr;
  EX_REF  Dst;
  UINT64  Bytes;
  UINT32  Pattern;
  UINT32  Reserved;
} EX_CMD_FILL_ARGS;

typedef struct {
  EX_CMD  Hdr;
  UINT64  Mdl;            // PMDL (kernel only: built by DxgkDdiBuildPagingBuffer)
  UINT64  MdlByteOffset;
  UINT64  Segment;        // segment address
  UINT64  Bytes;
} EX_CMD_PAGE_ARGS;

typedef struct {
  INT32   Left;
  INT32   Top;
  INT32   Right;
  INT32   Bottom;
} EX_RECT;

typedef struct {
  EX_CMD   Hdr;
  EX_REF   Src;
  EX_REF   Dst;
  UINT32   SrcPitch;
  UINT32   DstPitch;
  UINT32   SrcWidth;      // for clipping
  UINT32   SrcHeight;
  UINT32   DstWidth;
  UINT32   DstHeight;
  INT32    OffsetX;       // dst = src + offset
  INT32    OffsetY;
  UINT32   Color;         // EX_CMD_COLORFILL
  UINT32   RectCount;
  EX_RECT  Rects[1];      // destination rectangles, RectCount of them
} EX_CMD_BLT_ARGS;

typedef struct {
  EX_CMD  Hdr;
  UINT64  Chain;          // GPU VA of the first job
  UINT32  Slot;
  UINT32  Reserved;
} EX_CMD_GPU_JOB_ARGS;

//
// EX_CMD_GDI: one DXGK_GDIOP_* operation over RectCount destination
// rectangles (a DXGK_RENDERKM_COMMAND with many sub-rectangles becomes
// several of these).  Allocations are EX_ALLOCATION pointers (kernel only:
// built by DxgkDdiRenderKm, executed in the same driver); 0 = none.
//
#define EX_GDI_BITBLT          1      // DXGK_RENDERKM_OPERATION values
#define EX_GDI_COLORFILL       2
#define EX_GDI_ALPHABLEND      3
#define EX_GDI_STRETCHBLT      4
#define EX_GDI_TRANSPARENTBLT  6
#define EX_GDI_CLEARTYPEBLEND  7

#define EX_GDI_F_HASALPHA      0x1    // AlphaBlend: SourceHasAlpha
#define EX_GDI_F_HONORALPHA    0x2    // TransparentBlt: Flags.HonorAlpha
#define EX_GDI_F_MIRRORX       0x4    // StretchBlt
#define EX_GDI_F_MIRRORY       0x8

#define EX_GDI_MAX_RECTS       32     // rectangles per EX_CMD_GDI

typedef struct {
  EX_CMD   Hdr;
  UINT32   GdiOp;         // EX_GDI_*
  UINT32   RectCount;
  UINT64   Dst;           // EX_ALLOCATION *
  UINT64   Src;           // EX_ALLOCATION * (BitBlt / StretchBlt / AlphaBlend / TransparentBlt)
  UINT64   Alpha;         // EX_ALLOCATION * (ClearTypeBlend: coverage)
  UINT64   Gamma;         // EX_ALLOCATION * (ClearTypeBlend: gamma tables), 0 = none
  UINT32   DstPitch;      // from the command (CPU-visible staging / existing sysmem), 0 = the allocation's
  UINT32   SrcPitch;      // same for the source / the alpha surface
  EX_RECT  SrcRect;
  EX_RECT  DstRect;
  UINT32   Rop;           // DXGK_GDIROP_BITBLT / DXGK_GDIROP_COLORFILL; StretchBlt: Mode
  UINT32   Rop3;
  UINT32   Color;
  UINT32   Color2;        // ClearTypeBlend
  UINT32   GammaIndex;    // ClearTypeBlend: Gamma (D3DKM_INVALID_GAMMA_INDEX = none)
  UINT32   Flags;         // EX_GDI_F_*
  UINT32   ConstantAlpha; // AlphaBlend
  INT32    OffsetX;       // ClearTypeBlend: DstToAlphaOffsetX / Y
  INT32    OffsetY;
  UINT32   Reserved;
  EX_RECT  Rects[1];      // RectCount destination rectangles
} EX_CMD_GDI_ARGS;

//
// DXGKQAITYPE_UMDRIVERPRIVATE: input EX_UMD_QUERY, output EX_UMD_CAPS.
// Op 1 also writes Text (a UMD log line) to the driver's OS log.
//
#define EX_UMD_QUERY_CAPS    0
#define EX_UMD_QUERY_LOG     1

typedef struct {
  UINT32  Magic;          // EX_UMD_MAGIC
  UINT32  Op;
  CHAR    Text[120];
} EX_UMD_QUERY;

typedef struct {
  UINT32  Magic;
  UINT32  Version;        // EX_UMD_VERSION
  UINT32  SegmentSize;
  UINT32  DmaBufferSize;
  UINT32  MaxRects;       // EX_CMD_BLT rectangles per command
  UINT32  GpuId;          // 0 until the GPU side is in
} EX_UMD_CAPS;

#define EX_UMD_VERSION       1
#define EX_UMD_DIGIT_BASE    0x200     // size-encoded report, see exynosumd.c Log
#define EX_UMD_DIGITS        17
#define EX_DMA_BUFFER_SIZE   (64 * 1024)
#define EX_PAGING_BUFFER_SIZE (64 * 1024)
#define EX_ALLOCATION_LIST   256
#define EX_PATCH_LIST        1024
#define EX_MAX_RECTS         64

#endif // __EX_CMD_H__
