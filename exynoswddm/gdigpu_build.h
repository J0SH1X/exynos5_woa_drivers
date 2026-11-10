/** @file
  GDI on the Mali-T628: per-pass patching of the template (gdigpu_blob.h)
  in the scratch.  Portable C (no kernel headers, no floating point): built
  into exynoswddm.sys and into the host generator, which checks it against
  a pass packed entirely with Mesa's packers.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef __GDIGPU_BUILD_H__
#define __GDIGPU_BUILD_H__

#include <stdint.h>
#include "gdigpu_layout.h"

typedef struct {
  int  Left, Top, Right, Bottom;        // framebuffer coordinates, exclusive right / bottom
} GDIGPU_RECT;

typedef struct {
  unsigned     Op;                      // GDIGPU_OP_*
  uint64_t     SrcVa;                   // copy / alpha: GPU address of source pixel (0, 0) (64-byte aligned)
  uint32_t     SrcPitch;
  unsigned     SrcW, SrcH;              // source texture size
  int          SrcDx, SrcDy;            // source (cleartype: coverage) = framebuffer + (SrcDx, SrcDy)
  uint32_t     Color;                   // fill, cleartype foreground: 0xAARRGGBB
  uint32_t     Color2;                  // cleartype: the colour where the coverage is 255
  uint64_t     GammaVa;                 // cleartype: the gamma row (64-byte aligned, 512 bytes)
  unsigned     ConstantAlpha;           // alpha: 0..255
  unsigned     FirstRect;               // the draw's rectangles: Rects[FirstRect ..], FirstRect even,
  unsigned     RectCount;               // after the previous draw's
} GDIGPU_DRAW;

typedef struct _GDIGPU_PASS {
  uint64_t     FbVa;                    // GPU address of framebuffer pixel (0, 0): 64-byte aligned
  uint32_t     FbPitch;                 // bytes per destination row (multiple of 64)
  unsigned     FbW, FbH;                // framebuffer size, 1..GDIGPU_MAX_DIM
  unsigned     DrawCount;               // 1..GDIGPU_MAX_DRAWS, executed in order
  GDIGPU_DRAW  Draws[GDIGPU_MAX_DRAWS];
  GDIGPU_RECT  Rects[GDIGPU_MAX_RECTS]; // inside 0..FbW x 0..FbH, non-empty
} GDIGPU_PASS;

/**
  Tiler polygon list sizes for a FbW x FbH framebuffer with GDIGPU_MASK
  (Mesa pan_tiler.c, hierarchical).
**/
unsigned
GdiGpuHeaderSize (
  unsigned  W,
  unsigned  H
  );

unsigned
GdiGpuFullSize (
  unsigned  W,
  unsigned  H
  );

/**
  IEEE-754 single bits of an integer (exact for |V| < 2^24).
**/
uint32_t
GdiGpuFloatBits (
  int  V
  );

/**
  Writes every per-pass field, the draws' slots (copied from Slots =
  mGdiGpuSlots[GDIGPU_MAX_DRAWS][GDIGPU_OPS][SL_SIZE]) and the vertex data
  of pass P into Scratch (the CPU view of GDIGPU_BASE, already holding the
  template).  Returns 0, or -1 when P cannot run on the GPU (then nothing
  was written).
**/
int
GdiGpuBuild (
  uint8_t            *Scratch,
  const uint8_t      *Slots,
  const GDIGPU_PASS  *P
  );

#endif // __GDIGPU_BUILD_H__
