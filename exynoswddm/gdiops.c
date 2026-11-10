/** @file
  GDI hardware acceleration: the pixel operations (see gdiops.h).

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <string.h>
#include "gdiops.h"

#define GDI_MAX_DIM   16384      // keeps the coordinate arithmetic inside 32 bits

//
// Read-modify-write operations work on row chunks copied into these
// buffers: the surfaces are mapped write-combined (uncached), where one
// 4-byte read costs a full memory round trip, while memcpy reads in bursts.
//
#define GDI_CHUNK     256        // pixels; 1 KB per buffer on the (kernel) stack

static unsigned int
Div255 (unsigned int V)          // round (V / 255) for V <= 255 * 255
{
  V += 128;
  return (V + (V >> 8)) >> 8;
}

static unsigned int
Sat255 (unsigned int V)
{
  return (V > 255) ? 255 : V;
}

int
GdiSurfValidate (GDI_SURF *S)
{
  unsigned int  Row;
  unsigned int  MaxH;

  if ((S->Base == NULL) || ((S->Bpp != 4) && (S->Bpp != 1)) || (S->Pitch == 0) || (S->Width == 0) || (S->Height == 0)) {
    return 0;
  }
  if ((S->Bpp == 4) && (((S->Pitch & 3) != 0) || ((((size_t)S->Base) & 3) != 0))) {
    return 0;                    // 32-bit pixels are read and written as words
  }
  if (S->Width > GDI_MAX_DIM) {
    S->Width = GDI_MAX_DIM;
  }
  if (S->Height > GDI_MAX_DIM) {
    S->Height = GDI_MAX_DIM;
  }
  if (S->Width > S->Pitch / S->Bpp) {
    S->Width = S->Pitch / S->Bpp;
  }
  if (S->Width == 0) {
    return 0;
  }
  Row = S->Width * S->Bpp;
  if (S->Bytes < Row) {
    return 0;
  }
  MaxH = (S->Bytes - Row) / S->Pitch + 1;
  if (S->Height > MaxH) {
    S->Height = MaxH;
  }
  return 1;
}

unsigned int
GdiRop3 (unsigned int Rop, unsigned int P, unsigned int S, unsigned int D)
{
  unsigned int  R = 0;

  //
  // Bit (P << 2 | S << 1 | D) of the ROP code is the result for that input.
  //
  if (Rop & 0x01) { R |= ~P & ~S & ~D; }
  if (Rop & 0x02) { R |= ~P & ~S &  D; }
  if (Rop & 0x04) { R |= ~P &  S & ~D; }
  if (Rop & 0x08) { R |= ~P &  S &  D; }
  if (Rop & 0x10) { R |=  P & ~S & ~D; }
  if (Rop & 0x20) { R |=  P & ~S &  D; }
  if (Rop & 0x40) { R |=  P &  S & ~D; }
  if (Rop & 0x80) { R |=  P &  S &  D; }
  return R;
}

/**
  Rect inside the surface; 0 if empty.
**/
static int
ClipTo (const GDI_SURF *S, const GDI_RECT *In, GDI_RECT *Out)
{
  *Out = *In;
  if (Out->Left < 0) { Out->Left = 0; }
  if (Out->Top < 0) { Out->Top = 0; }
  if (Out->Right > (int)S->Width) { Out->Right = (int)S->Width; }
  if (Out->Bottom > (int)S->Height) { Out->Bottom = (int)S->Height; }
  return (Out->Right > Out->Left) && (Out->Bottom > Out->Top);
}

static unsigned int *
Row32 (const GDI_SURF *S, int Y)
{
  return (unsigned int *)(S->Base + (size_t)Y * S->Pitch);
}

static unsigned int
FillOp (unsigned int Rop, unsigned int Rop3, unsigned int C, unsigned int D)
{
  switch (Rop) {
    case GDI_FILL_PATCOPY:    return C;
    case GDI_FILL_PATINVERT:  return D ^ C;
    case GDI_FILL_PDXN:       return ~(C ^ D);
    case GDI_FILL_DSTINVERT:  return ~D;
    case GDI_FILL_PATAND:     return D & C;
    case GDI_FILL_PATOR:      return D | C;
    case GDI_FILL_ROP3:       return GdiRop3 (Rop3, C, 0, D);
    default:                  return D;
  }
}

void
GdiColorFill (const GDI_SURF *Dst, const GDI_RECT *Rect, unsigned int Color, unsigned int Rop, unsigned int Rop3)
{
  GDI_RECT      R;
  int           X;
  int           Y;
  int           W;
  unsigned int  Db[GDI_CHUNK];

  if (!ClipTo (Dst, Rect, &R)) {
    return;
  }
  W = R.Right - R.Left;
  for (Y = R.Top; Y < R.Bottom; Y++) {
    if (Dst->Bpp == 1) {
      unsigned char  *D = Dst->Base + (size_t)Y * Dst->Pitch + R.Left;

      if (Rop == GDI_FILL_PATCOPY) {
        memset (D, (int)(Color & 0xFF), (size_t)W);
      } else {
        for (X = 0; X < W; X++) {
          D[X] = (unsigned char)FillOp (Rop, Rop3, Color & 0xFF, D[X]);
        }
      }
    } else {
      unsigned int  *D = Row32 (Dst, Y) + R.Left;

      if (Rop == GDI_FILL_PATCOPY) {
        for (X = 0; X < W; X++) {
          D[X] = Color;
        }
      } else {
        int  X0;

        for (X0 = 0; X0 < W; X0 += GDI_CHUNK) {
          int  N = (W - X0 < GDI_CHUNK) ? (W - X0) : GDI_CHUNK;

          memcpy (Db, D + X0, (size_t)N * 4);
          for (X = 0; X < N; X++) {
            Db[X] = FillOp (Rop, Rop3, Color, Db[X]);
          }
          memcpy (D + X0, Db, (size_t)N * 4);
        }
      }
    }
  }
}

static unsigned int
BltOp (unsigned int Rop, unsigned int Rop3, unsigned int S, unsigned int D)
{
  switch (Rop) {
    case GDI_BLT_SRCCOPY:    return S;
    case GDI_BLT_SRCINVERT:  return D ^ S;
    case GDI_BLT_SRCAND:     return D & S;
    case GDI_BLT_SRCOR:      return D | S;
    case GDI_BLT_ROP3:       return GdiRop3 (Rop3, 0, S, D);
    default:                 return D;
  }
}

void
GdiBitBlt (const GDI_SURF *Dst, const GDI_SURF *Src, const GDI_RECT *Rect, int Dx, int Dy, unsigned int Rop,
           unsigned int Rop3)
{
  GDI_RECT      R;
  int           W;
  int           H;
  int           I;
  int           X;
  int           Same = (Dst->Base == Src->Base);
  unsigned int  Sb[GDI_CHUNK];
  unsigned int  Db[GDI_CHUNK];
  int       RowsUp;
  int       PixelsBack;

  if (Dst->Bpp != Src->Bpp) {
    return;
  }
  if ((Dx > GDI_MAX_DIM) || (Dx < -GDI_MAX_DIM) || (Dy > GDI_MAX_DIM) || (Dy < -GDI_MAX_DIM)) {
    return;
  }
  if (!ClipTo (Dst, Rect, &R)) {
    return;
  }
  //
  // The source of every destination pixel must exist.
  //
  if (R.Left + Dx < 0) { R.Left = -Dx; }
  if (R.Top + Dy < 0) { R.Top = -Dy; }
  if (R.Right + Dx > (int)Src->Width) { R.Right = (int)Src->Width - Dx; }
  if (R.Bottom + Dy > (int)Src->Height) { R.Bottom = (int)Src->Height - Dy; }
  W = R.Right - R.Left;
  H = R.Bottom - R.Top;
  if ((W <= 0) || (H <= 0)) {
    return;
  }
  //
  // Same surface: never read what this operation has already written.  The
  // source row of row y is y + Dy: going down is safe for Dy > 0, going up
  // for Dy < 0; inside one row (Dy == 0) the direction follows Dx.
  //
  RowsUp     = Same && (Dy < 0);
  PixelsBack = Same && (Dy == 0) && (Dx < 0);
  for (I = 0; I < H; I++) {
    int  Y = RowsUp ? (R.Bottom - 1 - I) : (R.Top + I);

    if (Dst->Bpp == 1) {
      unsigned char        *D = Dst->Base + (size_t)Y * Dst->Pitch + R.Left;
      const unsigned char  *S = Src->Base + (size_t)(Y + Dy) * Src->Pitch + (R.Left + Dx);

      if (Rop == GDI_BLT_SRCCOPY) {
        memmove (D, S, (size_t)W);
      } else if (PixelsBack) {
        for (X = W - 1; X >= 0; X--) {
          D[X] = (unsigned char)BltOp (Rop, Rop3, S[X], D[X]);
        }
      } else {
        for (X = 0; X < W; X++) {
          D[X] = (unsigned char)BltOp (Rop, Rop3, S[X], D[X]);
        }
      }
    } else {
      unsigned int        *D = Row32 (Dst, Y) + R.Left;
      const unsigned int  *S = Row32 (Src, Y + Dy) + (R.Left + Dx);

      if (Rop == GDI_BLT_SRCCOPY) {
        if (Same && (Dy == 0)) {
          memmove (D, S, (size_t)W * 4);
        } else {
          memcpy (D, S, (size_t)W * 4);
        }
      } else if (PixelsBack) {
        for (X = W - 1; X >= 0; X--) {
          D[X] = BltOp (Rop, Rop3, S[X], D[X]);
        }
      } else if (Same && (Dy == 0)) {
        for (X = 0; X < W; X++) {                // same row, source ahead: pixel by pixel keeps the order
          D[X] = BltOp (Rop, Rop3, S[X], D[X]);
        }
      } else {
        int  X0;

        for (X0 = 0; X0 < W; X0 += GDI_CHUNK) {
          int  N = (W - X0 < GDI_CHUNK) ? (W - X0) : GDI_CHUNK;

          memcpy (Sb, S + X0, (size_t)N * 4);
          memcpy (Db, D + X0, (size_t)N * 4);
          for (X = 0; X < N; X++) {
            Db[X] = BltOp (Rop, Rop3, Sb[X], Db[X]);
          }
          memcpy (D + X0, Db, (size_t)N * 4);
        }
      }
    }
  }
}

//
// Destination -> source coordinate of a stretch, one axis:
//   s = truncate ((d - DstLo + 0.5) * SrcSize / DstSize) + SrcLo
// (the WDK's formula for COLORONCOLOR), clamped into the source surface.
//
typedef struct {
  int  SrcLo;
  int  SrcSize;
  int  DstLo;
  int  DstSize;
  int  Mirror;
  int  Limit;                    // source surface size on this axis
} GDI_AXIS;

static int
AxisInit (GDI_AXIS *A, int SrcLo, int SrcHi, int DstLo, int DstHi, int Mirror, unsigned int Limit)
{
  A->SrcLo   = SrcLo;
  A->SrcSize = SrcHi - SrcLo;
  A->DstLo   = DstLo;
  A->DstSize = DstHi - DstLo;
  A->Mirror  = Mirror;
  A->Limit   = (int)Limit;
  return (A->SrcSize > 0) && (A->DstSize > 0) && (A->SrcSize <= GDI_MAX_DIM) && (A->DstSize <= GDI_MAX_DIM) &&
         (SrcLo > -GDI_MAX_DIM) && (SrcLo < GDI_MAX_DIM);
}

static int
AxisIndex (const GDI_AXIS *A, int D)
{
  int  I = D - A->DstLo;

  if (I < 0) { I = 0; }
  if (I >= A->DstSize) { I = A->DstSize - 1; }
  return I;
}

/**
  Source offset (0 .. SrcSize - 1) -> source coordinate.  A mirror flips in
  source space, s = SrcHi - 1 - offset, as software GDI does (flipping the
  destination index instead differs by one pixel wherever (i + 0.5) * Ws /
  Wd is a whole number - seen on the tablet as stripes every 35 pixels).
**/
static int
AxisSource (const GDI_AXIS *A, int Offset)
{
  return A->Mirror ? (A->SrcLo + A->SrcSize - 1 - Offset) : (A->SrcLo + Offset);
}

static int
AxisClamp (const GDI_AXIS *A, int S)
{
  if (S < 0) { S = 0; }
  if (S >= A->Limit) { S = A->Limit - 1; }
  return S;
}

static int
AxisMap (const GDI_AXIS *A, int D)
{
  unsigned int  I = (unsigned int)AxisIndex (A, D);

  if (A->SrcSize == A->DstSize) {
    return AxisClamp (A, AxisSource (A, (int)I));
  }
  return AxisClamp (A, AxisSource (A, (int)(((2 * I + 1) * (unsigned int)A->SrcSize) / (2 * (unsigned int)A->DstSize))));
}

/**
  The source span [*Lo, *Hi) that shrinks into destination coordinate D
  (BLACKONWHITE / WHITEONBLACK combine it); one pixel when not shrinking.
**/
static void
AxisSpan (const GDI_AXIS *A, int D, int *Lo, int *Hi)
{
  unsigned int  I = (unsigned int)AxisIndex (A, D);

  if (A->SrcSize <= A->DstSize) {
    *Lo = AxisMap (A, D);
    *Hi = *Lo + 1;
    return;
  }
  {
    int  O0 = (int)((I * (unsigned int)A->SrcSize) / (unsigned int)A->DstSize);             // offsets [O0, O1)
    int  O1 = (int)(((I + 1) * (unsigned int)A->SrcSize) / (unsigned int)A->DstSize);
    int  S0 = A->Mirror ? AxisSource (A, O1 - 1) : AxisSource (A, O0);                       // lowest coordinate
    int  S1 = A->Mirror ? AxisSource (A, O0) : AxisSource (A, O1 - 1);                       // highest

    *Lo = AxisClamp (A, S0);
    *Hi = AxisClamp (A, S1) + 1;
  }
  if (*Hi <= *Lo) {
    *Hi = *Lo + 1;
  }
}

static void SourceSpan (const GDI_AXIS *Ax, const unsigned int *S, int X0, int N, unsigned int *Out);

void
GdiStretchBlt (const GDI_SURF *Dst, const GDI_SURF *Src, const GDI_RECT *Rect, const GDI_RECT *SrcRect,
               const GDI_RECT *DstRect, unsigned int Mode, int MirrorX, int MirrorY)
{
  GDI_RECT      R;
  GDI_AXIS      Ax;
  GDI_AXIS      Ay;
  int           X;
  int           Y;
  int           Combine;
  unsigned int  Ob[GDI_CHUNK];

  if ((Dst->Bpp != 4) || (Src->Bpp != 4) || !ClipTo (Dst, Rect, &R)) {
    return;
  }
  if (!AxisInit (&Ax, SrcRect->Left, SrcRect->Right, DstRect->Left, DstRect->Right, MirrorX, Src->Width) ||
      !AxisInit (&Ay, SrcRect->Top, SrcRect->Bottom, DstRect->Top, DstRect->Bottom, MirrorY, Src->Height)) {
    return;
  }
  Combine = ((Mode == GDI_STRETCH_BLACKONWHITE) || (Mode == GDI_STRETCH_WHITEONBLACK)) &&
            ((Ax.SrcSize > Ax.DstSize) || (Ay.SrcSize > Ay.DstSize));
  if (!Combine) {
    int  X0;

    //
    // Chunk by chunk of columns: the chunk's output row is gathered once
    // and written again as long as the following destination rows map to
    // the same source row (an enlargement repeats every source row).
    //
    for (X0 = R.Left; X0 < R.Right; X0 += GDI_CHUNK) {
      int  N = (R.Right - X0 < GDI_CHUNK) ? (R.Right - X0) : GDI_CHUNK;
      int  Last = -1;

      for (Y = R.Top; Y < R.Bottom; Y++) {
        int  Sy = AxisMap (&Ay, Y);

        if (Sy != Last) {
          SourceSpan (&Ax, Row32 (Src, Sy), X0, N, Ob);
          Last = Sy;
        }
        memcpy (Row32 (Dst, Y) + X0, Ob, (size_t)N * 4);
      }
    }
    return;
  }
  for (Y = R.Top; Y < R.Bottom; Y++) {
    unsigned int  *D = Row32 (Dst, Y);

    if (!Combine) {
      const unsigned int  *S = Row32 (Src, AxisMap (&Ay, Y));

      for (X = R.Left; X < R.Right; X++) {
        D[X] = S[AxisMap (&Ax, X)];
      }
    } else {
      int  Y0;
      int  Y1;

      AxisSpan (&Ay, Y, &Y0, &Y1);
      for (X = R.Left; X < R.Right; X++) {
        int           X0;
        int           X1;
        int           Sx;
        int           Sy;
        unsigned int  V = (Mode == GDI_STRETCH_BLACKONWHITE) ? 0xFFFFFFFFu : 0;

        AxisSpan (&Ax, X, &X0, &X1);
        for (Sy = Y0; Sy < Y1; Sy++) {
          const unsigned int  *S = Row32 (Src, Sy);

          for (Sx = X0; Sx < X1; Sx++) {
            if (Mode == GDI_STRETCH_BLACKONWHITE) {
              V &= S[Sx];
            } else {
              V |= S[Sx];
            }
          }
        }
        D[X] = V;
      }
    }
  }
}

/**
  Out[i] = S[AxisMap (Ax, X0 + i)], i < N.  An unstretched, unmirrored axis
  maps to a contiguous source span: one burst copy.
**/
#define GDI_SPAN      512        // source pixels one stretched chunk may span and still be read in one burst

static void
SourceSpan (const GDI_AXIS *Ax, const unsigned int *S, int X0, int N, unsigned int *Out)
{
  int  I;
  int  First = AxisMap (Ax, X0);
  int  Last  = AxisMap (Ax, X0 + N - 1);
  int  Lo    = (First < Last) ? First : Last;  // mirrored: the span runs backwards
  int  Hi    = (First < Last) ? Last : First;

  if ((Ax->SrcSize == Ax->DstSize) && !Ax->Mirror && (Last - First == N - 1)) {
    memcpy (Out, S + First, (size_t)N * 4);    // unstretched, nothing clamped: one burst
    return;
  }
  if (Hi - Lo < GDI_SPAN) {
    unsigned int  Span[GDI_SPAN];

    //
    // Stretched or mirrored: the mapping is monotonic, so every source
    // pixel of the chunk lies in [Lo, Hi] - read that once, gather from it.
    //
    memcpy (Span, S + Lo, (size_t)(Hi - Lo + 1) * 4);
    for (I = 0; I < N; I++) {
      Out[I] = Span[AxisMap (Ax, X0 + I) - Lo];
    }
    return;
  }
  for (I = 0; I < N; I++) {
    Out[I] = S[AxisMap (Ax, X0 + I)];
  }
}

static unsigned int
BlendPixel (unsigned int S, unsigned int D, unsigned int Ca, int HasAlpha)
{
  unsigned int  Out = 0;
  unsigned int  Shift;

  if (HasAlpha) {
    unsigned int  A;

    //
    // Premultiplied source over destination, the source first scaled by
    // the constant alpha (all four channels, as GDI's AlphaBlend does).
    //
    if (Ca != 255) {
      S = (Div255 ((S >> 24) * Ca) << 24) | (Div255 (((S >> 16) & 0xFF) * Ca) << 16) |
          (Div255 (((S >> 8) & 0xFF) * Ca) << 8) | Div255 ((S & 0xFF) * Ca);
    }
    A = S >> 24;
    if (A == 255) {
      return S;
    }
    if (S == 0) {
      return D;
    }
    for (Shift = 0; Shift < 32; Shift += 8) {
      Out |= Sat255 (((S >> Shift) & 0xFF) + Div255 (((D >> Shift) & 0xFF) * (255 - A))) << Shift;
    }
    return Out;
  }
  if (Ca == 255) {
    return S;
  }
  for (Shift = 0; Shift < 32; Shift += 8) {
    Out |= Div255 (((S >> Shift) & 0xFF) * Ca + ((D >> Shift) & 0xFF) * (255 - Ca)) << Shift;
  }
  return Out;
}

void
GdiAlphaBlend (const GDI_SURF *Dst, const GDI_SURF *Src, const GDI_RECT *Rect, const GDI_RECT *SrcRect,
               const GDI_RECT *DstRect, unsigned int ConstantAlpha, int SourceHasAlpha)
{
  GDI_RECT      R;
  GDI_AXIS      Ax;
  GDI_AXIS      Ay;
  int           Y;
  unsigned int  Sb[GDI_CHUNK];
  unsigned int  Db[GDI_CHUNK];

  if ((Dst->Bpp != 4) || (Src->Bpp != 4) || !ClipTo (Dst, Rect, &R)) {
    return;
  }
  if (!AxisInit (&Ax, SrcRect->Left, SrcRect->Right, DstRect->Left, DstRect->Right, 0, Src->Width) ||
      !AxisInit (&Ay, SrcRect->Top, SrcRect->Bottom, DstRect->Top, DstRect->Bottom, 0, Src->Height)) {
    return;
  }
  ConstantAlpha &= 0xFF;
  if (!SourceHasAlpha && (ConstantAlpha == 0)) {
    return;
  }
  for (Y = R.Top; Y < R.Bottom; Y++) {
    unsigned int        *D = Row32 (Dst, Y);
    const unsigned int  *S = Row32 (Src, AxisMap (&Ay, Y));
    int                 X0;

    for (X0 = R.Left; X0 < R.Right; X0 += GDI_CHUNK) {
      int  N = (R.Right - X0 < GDI_CHUNK) ? (R.Right - X0) : GDI_CHUNK;
      int  I;

      SourceSpan (&Ax, S, X0, N, Sb);
      memcpy (Db, D + X0, (size_t)N * 4);
      for (I = 0; I < N; I++) {
        Db[I] = BlendPixel (Sb[I], Db[I], ConstantAlpha, SourceHasAlpha);
      }
      memcpy (D + X0, Db, (size_t)N * 4);
    }
  }
}

void
GdiTransparentBlt (const GDI_SURF *Dst, const GDI_SURF *Src, const GDI_RECT *Rect, const GDI_RECT *SrcRect,
                   const GDI_RECT *DstRect, unsigned int Color, int HonorAlpha)
{
  GDI_RECT      R;
  GDI_AXIS      Ax;
  GDI_AXIS      Ay;
  int           Y;
  unsigned int  Mask = HonorAlpha ? 0xFFFFFFFFu : 0x00FFFFFFu;
  unsigned int  Sb[GDI_CHUNK];
  unsigned int  Db[GDI_CHUNK];

  if ((Dst->Bpp != 4) || (Src->Bpp != 4) || !ClipTo (Dst, Rect, &R)) {
    return;
  }
  if (!AxisInit (&Ax, SrcRect->Left, SrcRect->Right, DstRect->Left, DstRect->Right, 0, Src->Width) ||
      !AxisInit (&Ay, SrcRect->Top, SrcRect->Bottom, DstRect->Top, DstRect->Bottom, 0, Src->Height)) {
    return;
  }
  Color &= Mask;
  for (Y = R.Top; Y < R.Bottom; Y++) {
    unsigned int        *D = Row32 (Dst, Y);
    const unsigned int  *S = Row32 (Src, AxisMap (&Ay, Y));
    int                 X0;

    for (X0 = R.Left; X0 < R.Right; X0 += GDI_CHUNK) {
      int  N = (R.Right - X0 < GDI_CHUNK) ? (R.Right - X0) : GDI_CHUNK;
      int  I;
      int  Keyed = 0;

      SourceSpan (&Ax, S, X0, N, Sb);
      for (I = 0; I < N; I++) {
        Keyed |= ((Sb[I] & Mask) == Color);
      }
      if (!Keyed) {
        memcpy (D + X0, Sb, (size_t)N * 4);      // no key colour here: a plain copy, no destination read
        continue;
      }
      memcpy (Db, D + X0, (size_t)N * 4);
      for (I = 0; I < N; I++) {
        if ((Sb[I] & Mask) != Color) {
          Db[I] = Sb[I];
        }
      }
      memcpy (D + X0, Db, (size_t)N * 4);
    }
  }
}

void
GdiClearTypeBlend (const GDI_SURF *Dst, const GDI_SURF *Alpha, const GDI_SURF *GammaSurf, const GDI_RECT *Rect,
                   int OffX, int OffY, unsigned int Color, unsigned int Color2, unsigned int Gamma)
{
  GDI_RECT             R;
  const unsigned char  *G = NULL;       // gamma
  const unsigned char  *Ig = NULL;      // inverse gamma
  unsigned char        Gt[GDI_GAMMA_WIDTH];
  unsigned int         Ab[GDI_CHUNK];
  unsigned int         Db[GDI_CHUNK];
  int                  Y;
  unsigned int         Shift;

  if ((Dst->Bpp != 4) || (Alpha->Bpp != 4) || !ClipTo (Dst, Rect, &R)) {
    return;
  }
  if ((OffX > GDI_MAX_DIM) || (OffX < -GDI_MAX_DIM) || (OffY > GDI_MAX_DIM) || (OffY < -GDI_MAX_DIM)) {
    return;
  }
  if (R.Left + OffX < 0) { R.Left = -OffX; }
  if (R.Top + OffY < 0) { R.Top = -OffY; }
  if (R.Right + OffX > (int)Alpha->Width) { R.Right = (int)Alpha->Width - OffX; }
  if (R.Bottom + OffY > (int)Alpha->Height) { R.Bottom = (int)Alpha->Height - OffY; }
  if ((R.Right <= R.Left) || (R.Bottom <= R.Top)) {
    return;
  }
  if ((Gamma != GDI_GAMMA_INVALID) && (GammaSurf != NULL) && (GammaSurf->Bpp == 1) &&
      (GammaSurf->Width >= GDI_GAMMA_WIDTH) && (Gamma < GammaSurf->Height)) {
    memcpy (Gt, GammaSurf->Base + (size_t)Gamma * GammaSurf->Pitch, GDI_GAMMA_WIDTH);   // one burst, not 2 reads per channel
    G  = Gt;
    Ig = Gt + 256;
  }
  for (Y = R.Top; Y < R.Bottom; Y++) {
    unsigned int        *D = Row32 (Dst, Y);
    const unsigned int  *A = Row32 (Alpha, Y + OffY) + OffX;
    int                 X0;

    for (X0 = R.Left; X0 < R.Right; X0 += GDI_CHUNK) {
      int  N = (R.Right - X0 < GDI_CHUNK) ? (R.Right - X0) : GDI_CHUNK;
      int  Any = 0;
      int  I;

      memcpy (Ab, A + X0, (size_t)N * 4);
      for (I = 0; I < N; I++) {
        Any |= (int)(Ab[I] & 0x00FFFFFF);
      }
      if (!Any) {
        continue;                               // no glyph in this chunk: the destination is not even read
      }
      memcpy (Db, D + X0, (size_t)N * 4);
      for (I = 0; I < N; I++) {
      unsigned int  Av = Ab[I];
      unsigned int  Dv;
      unsigned int  Out;

      if ((Av & 0x00FFFFFF) == 0) {
        continue;                               // nothing of the glyph here
      }
      Dv  = Db[I];
      Out = Dv & 0xFF000000;                    // destination alpha stays
      for (Shift = 0; Shift < 24; Shift += 8) {
        unsigned int  Dc = (Dv >> Shift) & 0xFF;
        unsigned int  Cc = (Color >> Shift) & 0xFF;
        unsigned int  Ac;
        unsigned int  V;

        if (Gamma == GDI_GAMMA_INVALID) {
          //
          // Antialiased (not ClearType): two coverages, in R for a
          // foreground brighter than the destination and in G for a darker.
          //
          Ac = (Cc >= Dc) ? ((Av >> 16) & 0xFF) : ((Av >> 8) & 0xFF);
          V  = (Ac == 0) ? Dc : (Ac == 255) ? ((Color2 >> Shift) & 0xFF) : Div255 (Dc * (255 - Ac) + Cc * Ac);
        } else {
          Ac = (Av >> Shift) & 0xFF;
          if (Ac == 0) {
            V = Dc;
          } else if (Ac == 255) {
            V = (Color2 >> Shift) & 0xFF;
          } else if (G != NULL) {
            V = Ig[Div255 (G[Dc] * (255 - Ac) + Cc * Ac)];
          } else {
            V = Div255 (Dc * (255 - Ac) + ((Color2 >> Shift) & 0xFF) * Ac);   // no table: blend ungamma'd
          }
        }
        Out |= V << Shift;
      }
      Db[I] = Out;
      }
      memcpy (D + X0, Db, (size_t)N * 4);       // write-combined: one burst
    }
  }
}
