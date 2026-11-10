/** @file
  GDI hardware acceleration: the pixel operations of DxgkDdiRenderKm
  (DXGK_GDIOP_*), executed on the CPU.

  Portable C on purpose (no kernel headers): the same file is built into
  exynoswddm.sys and into the host test (tools/gditest), which checks every
  operation against a straightforward reference.

  Every function clips its rectangle against both surfaces and against the
  bytes the surface really has (GDI_SURF.Bytes): a wrong rectangle or pitch
  from above must never become a wild kernel write.

  Semantics follow the WDK documentation of DXGK_GDIARG_BITBLT / COLORFILL /
  STRETCHBLT / ALPHABLEND / TRANSPARENTBLT / CLEARTYPEBLEND.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef __GDI_OPS_H__
#define __GDI_OPS_H__

typedef struct {
  int  Left;
  int  Top;
  int  Right;                   // exclusive
  int  Bottom;                  // exclusive
} GDI_RECT;                     // same layout as RECT and EX_RECT

typedef struct {
  unsigned char  *Base;         // row 0, pixel 0
  unsigned int   Pitch;         // bytes per row
  unsigned int   Width;         // pixels
  unsigned int   Height;
  unsigned int   Bpp;           // bytes per pixel: 4 (A8R8G8B8 / X8R8G8B8) or 1 (A8)
  unsigned int   Bytes;         // bytes addressable from Base
} GDI_SURF;

// DXGK_GDIROP_BITBLT
#define GDI_BLT_SRCCOPY     1
#define GDI_BLT_SRCINVERT   2
#define GDI_BLT_SRCAND      3
#define GDI_BLT_SRCOR       4
#define GDI_BLT_ROP3        5

// DXGK_GDIROP_COLORFILL
#define GDI_FILL_PATCOPY    1
#define GDI_FILL_PATINVERT  2
#define GDI_FILL_PDXN       3
#define GDI_FILL_DSTINVERT  4
#define GDI_FILL_PATAND     5
#define GDI_FILL_PATOR      6
#define GDI_FILL_ROP3       7

// StretchBlt modes (wingdi.h)
#define GDI_STRETCH_BLACKONWHITE  1     // shrinking: AND the source pixels
#define GDI_STRETCH_WHITEONBLACK  2     // shrinking: OR the source pixels
#define GDI_STRETCH_COLORONCOLOR  3     // nearest

#define GDI_GAMMA_INVALID   0xFFFFFFFFu // D3DKM_INVALID_GAMMA_INDEX
#define GDI_GAMMA_WIDTH     512         // gamma[256] then inverse gamma[256] per row
#define GDI_GAMMA_ROWS      16

/**
  Makes Width / Height fit Pitch and Bytes; 0 if nothing usable is left.
**/
int
GdiSurfValidate (
  GDI_SURF  *S
  );

/**
  The ROP3 Rop of pattern P, source S and destination D (32 bits at once).
**/
unsigned int
GdiRop3 (
  unsigned int  Rop,
  unsigned int  P,
  unsigned int  S,
  unsigned int  D
  );

void
GdiColorFill (
  const GDI_SURF  *Dst,
  const GDI_RECT  *Rect,
  unsigned int    Color,
  unsigned int    Rop,
  unsigned int    Rop3
  );

/**
  Dst[x, y] = Rop (Src[x + Dx, y + Dy], Dst[x, y]) over Rect (destination
  coordinates).  Src and Dst may be the same surface with overlapping
  rectangles (a scroll): rows and pixels are walked in the safe order.
**/
void
GdiBitBlt (
  const GDI_SURF  *Dst,
  const GDI_SURF  *Src,
  const GDI_RECT  *Rect,
  int             Dx,
  int             Dy,
  unsigned int    Rop,
  unsigned int    Rop3
  );

void
GdiStretchBlt (
  const GDI_SURF  *Dst,
  const GDI_SURF  *Src,
  const GDI_RECT  *Rect,          // destination sub-rectangle
  const GDI_RECT  *SrcRect,
  const GDI_RECT  *DstRect,
  unsigned int    Mode,
  int             MirrorX,
  int             MirrorY
  );

void
GdiAlphaBlend (
  const GDI_SURF  *Dst,
  const GDI_SURF  *Src,
  const GDI_RECT  *Rect,
  const GDI_RECT  *SrcRect,
  const GDI_RECT  *DstRect,
  unsigned int    ConstantAlpha,
  int             SourceHasAlpha
  );

void
GdiTransparentBlt (
  const GDI_SURF  *Dst,
  const GDI_SURF  *Src,
  const GDI_RECT  *Rect,
  const GDI_RECT  *SrcRect,
  const GDI_RECT  *DstRect,
  unsigned int    Color,
  int             HonorAlpha
  );

/**
  ClearType / antialiased text.  Alpha[x + OffX, y + OffY] holds the
  coverage of destination pixel (x, y) in its R, G and B bytes.  Gamma: a
  row of GammaSurf (A8, GDI_GAMMA_WIDTH wide) or GDI_GAMMA_INVALID.
**/
void
GdiClearTypeBlend (
  const GDI_SURF  *Dst,
  const GDI_SURF  *Alpha,
  const GDI_SURF  *GammaSurf,     // may be NULL with GDI_GAMMA_INVALID
  const GDI_RECT  *Rect,
  int             OffX,
  int             OffY,
  unsigned int    Color,          // gamma corrected foreground
  unsigned int    Color2,         // foreground as is
  unsigned int    Gamma
  );

#endif // __GDI_OPS_H__
