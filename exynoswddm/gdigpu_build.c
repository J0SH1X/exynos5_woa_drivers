/** @file
  GDI on the Mali-T628: per-pass patching of the template (see
  gdigpu_build.h).  The word positions below are the GenXML v4 layouts
  (Mesa 21.3 v4_pack.h); ext/gpugen/gen_gdi.c checks every one of them by
  comparing against passes packed with Mesa's own packers.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <string.h>
#include "gdigpu_build.h"

// --- descriptor field positions (byte offsets inside the descriptor) -----------------------------------------
#define FBD_PARAMS            32      // FRAMEBUFFER section PARAMETERS
#define FBD_P_BOUND_MAX       (FBD_PARAMS + 3 * 4)       // bound_max_x 0..15, bound_max_y 16..31
#define FBD_P_WB_BASE         (FBD_PARAMS + 12 * 4)      // color_writeback.base (64)
#define FBD_P_WB_STRIDE       (FBD_PARAMS + 14 * 4)      // color_writeback.row_stride
#define FBD_TILER             192     // FRAMEBUFFER section TILER (TILER_CONTEXT)
#define FBD_T_SIZE            (FBD_TILER + 0 * 4)        // polygon_list_size
#define FBD_T_BODY            (FBD_TILER + 4 * 4)        // polygon_list_body (64)
#define JOB_NEXT              (6 * 4)                    // JOB_HEADER next (64)
#define TJ_INVOCATION         32      // TILER_JOB section INVOCATION: cl[0] invocations
#define TJ_PRIM_COUNT         (40 + 3 * 4)               // PRIMITIVE index_count - 1
#define TJ_POSITION           (64 + 4 * 4)               // DRAW position (64)
#define FJ_BOUND_MAX          (32 + 1 * 4)               // FRAGMENT_JOB payload bound_max_x 0..11, _y 16..27
#define VB_SIZE               (3 * 4)                    // ATTRIBUTE_BUFFER size
#define TEX_SIZE              0                          // TEXTURE width - 1 0..15, height - 1 16..31
#define TEX_SURF              32                         // SURFACE_WITH_STRIDE after the TEXTURE
#define SURF_POINTER          (TEX_SURF + 0)             // (64)
#define SURF_ROW_STRIDE       (TEX_SURF + 2 * 4)
#define SURF_SURFACE_STRIDE   (TEX_SURF + 3 * 4)

#define RSD_BLEND_CONSTANT    (15 * 4)                   // RENDERER_STATE blend_constant (float, SFBD)
#define VP_MAX_X              (2 * 4)                    // VIEWPORT maximum_x / _y (float)
#define VP_SCISSOR_MAX        (7 * 4)                    // VIEWPORT scissor_maximum_x 0..15, _y 16..31
#define JOB_STATUS_BYTES      16                         // JOB_HEADER exception_status, first_incomplete_task, fault_pointer

#define VERTEX_BYTES          16                         // vec4 per vertex in every array

static void
Put32 (uint8_t *Base, unsigned Offset, uint32_t V)
{
  Base[Offset]     = (uint8_t)V;
  Base[Offset + 1] = (uint8_t)(V >> 8);
  Base[Offset + 2] = (uint8_t)(V >> 16);
  Base[Offset + 3] = (uint8_t)(V >> 24);
}

static void
Put64 (uint8_t *Base, unsigned Offset, uint64_t V)
{
  Put32 (Base, Offset, (uint32_t)V);
  Put32 (Base, Offset + 4, (uint32_t)(V >> 32));
}

uint32_t
GdiGpuFloatBits (int V)
{
  uint32_t  Sign = 0;
  uint32_t  M;
  int       E = 0;

  if (V == 0) {
    return 0;
  }
  if (V < 0) {
    Sign = 0x80000000u;
    M    = (uint32_t)(-(int64_t)V);
  } else {
    M = (uint32_t)V;
  }
  while ((M >> E) > 1) {
    E++;                                // E = floor (log2 (M))
  }
  // mantissa: the bits below the leading one, left aligned into 23 bits (exact for M < 2^24)
  M = (E <= 23) ? (M << (23 - E)) : (M >> (E - 23));
  return Sign | ((uint32_t)(E + 127) << 23) | (M & 0x7FFFFF);
}

//
// pan_tiler.c (Mesa 21.3), hierarchical sizes
//
static unsigned
TileCount (unsigned W, unsigned H, unsigned Tw, unsigned Th)
{
  return ((W + Tw - 1) / Tw) * ((H + Th - 1) / Th);
}

static unsigned
HierarchySize (unsigned W, unsigned H, unsigned Mask, unsigned BytesPerTile)
{
  unsigned  Size = 0x40;                // PROLOGUE_SIZE
  unsigned  B;

  for (B = 0; B < (13 - 4); ++B) {      // MAX_TILE_SHIFT - MIN_TILE_SHIFT
    unsigned  Ts;

    if (!(Mask & (1u << B))) {
      continue;
    }
    Ts = (1u << B) * 16;
    Size += BytesPerTile * TileCount (W, H, Ts, Ts);
  }
  return (Size + 0x1FF) & ~0x1FFu;
}

unsigned
GdiGpuHeaderSize (unsigned W, unsigned H)
{
  return HierarchySize (W, H, GDIGPU_MASK, 0x8);
}

unsigned
GdiGpuFullSize (unsigned W, unsigned H)
{
  return HierarchySize (W, H, GDIGPU_MASK, 0x200);
}

static void
PutVec4 (uint8_t *Base, unsigned Offset, uint32_t X, uint32_t Y, uint32_t Z, uint32_t W)
{
  Put32 (Base, Offset, X);
  Put32 (Base, Offset + 4, Y);
  Put32 (Base, Offset + 8, Z);
  Put32 (Base, Offset + 12, W);
}

#define F_HALF   0x3F000000u            // 0.5f
#define F_ONE    0x3F800000u            // 1.0f

/**
  Six vertices (two triangles) of rectangle R at Offset: positions, or
  texture coordinates shifted by (Dx, Dy).
**/
static void
PutQuad (uint8_t *Base, unsigned Offset, const GDIGPU_RECT *R, int Dx, int Dy, int Positions)
{
  static const unsigned char  Corner[6][2] = { { 0, 0 }, { 1, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 }, { 0, 1 } };
  unsigned                    V;

  for (V = 0; V < 6; V++) {
    int  X = (Corner[V][0] ? R->Right : R->Left) + Dx;
    int  Y = (Corner[V][1] ? R->Bottom : R->Top) + Dy;

    PutVec4 (Base, Offset + V * VERTEX_BYTES, GdiGpuFloatBits (X), GdiGpuFloatBits (Y), Positions ? F_HALF : 0,
             Positions ? F_ONE : 0);
  }
}

/**
  Cleartype texture coordinates: (coverage x, y, destination x, y).
**/
static void
PutQuadCt (uint8_t *Base, unsigned Offset, const GDIGPU_RECT *R, int Dx, int Dy)
{
  static const unsigned char  Corner[6][2] = { { 0, 0 }, { 1, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 }, { 0, 1 } };
  unsigned                    V;

  for (V = 0; V < 6; V++) {
    int  X = Corner[V][0] ? R->Right : R->Left;
    int  Y = Corner[V][1] ? R->Bottom : R->Top;

    PutVec4 (Base, Offset + V * VERTEX_BYTES, GdiGpuFloatBits (X + Dx), GdiGpuFloatBits (Y + Dy), GdiGpuFloatBits (X),
             GdiGpuFloatBits (Y));
  }
}

//
// k / 255 as float bits, k = 0..255 (computed without floating point:
// the correctly rounded quotient, 24 significant bits).
//
static uint32_t
Unorm8Bits (unsigned K)
{
  uint64_t  Num;
  int       E = -1;
  uint32_t  M;

  if (K == 0) {
    return 0;
  }
  if (K == 255) {
    return F_ONE;
  }
  // K / 255 lies in [2^E, 2^(E+1)) with E < 0: find E, then the 24-bit significand rounded to nearest even
  while (((uint64_t)K << (-E)) < 255) {
    E--;
  }
  Num = ((uint64_t)K << (23 - E));      // K * 2^(23 - E) / 255 = significand with 23 fraction bits
  M = (uint32_t)(Num / 255);
  {
    uint64_t  Rem = Num - (uint64_t)M * 255;

    if ((Rem * 2 > 255) || ((Rem * 2 == 255) && (M & 1))) {
      M++;
    }
  }
  if (M >> 24) {                        // rounded up to the next power of two
    M >>= 1;
    E++;
  }
  return ((uint32_t)(E + 127) << 23) | (M & 0x7FFFFF);
}

static int
RectOk (const GDIGPU_RECT *R, unsigned W, unsigned H)
{
  return (R->Left >= 0) && (R->Top >= 0) && (R->Right <= (int)W) && (R->Bottom <= (int)H) && (R->Left < R->Right) &&
         (R->Top < R->Bottom);
}

static int
Textured (unsigned Op)
{
  return (Op == GDIGPU_OP_COPY) || (Op == GDIGPU_OP_ALPHAP) || (Op == GDIGPU_OP_ALPHAC) || (Op == GDIGPU_OP_CT);
}

static int
DrawOk (const GDIGPU_PASS *P, const GDIGPU_DRAW *D, unsigned MinFirst)
{
  unsigned  I;

  if ((D->Op < 1) || (D->Op > GDIGPU_OPS) || (D->ConstantAlpha > 255) || (D->RectCount == 0) || (D->FirstRect & 1) ||
      (D->FirstRect < MinFirst) || (D->FirstRect > GDIGPU_MAX_RECTS) || (D->RectCount > GDIGPU_MAX_RECTS - D->FirstRect)) {
    return 0;
  }
  if ((D->Op == GDIGPU_OP_CT) && ((MinFirst != 0) || (D->GammaVa == 0) || ((D->GammaVa & 63) != 0))) {
    return 0;                           // cleartype reads the destination from memory: first draw only
  }
  if (Textured (D->Op) &&
      ((D->SrcW == 0) || (D->SrcH == 0) || (D->SrcW > GDIGPU_MAX_DIM) || (D->SrcH > GDIGPU_MAX_DIM) || ((D->SrcVa & 63) != 0) ||
       ((D->SrcPitch & 63) != 0) || (D->SrcPitch < D->SrcW * 4) || (D->SrcDx > GDIGPU_MAX_DIM) || (D->SrcDx < -GDIGPU_MAX_DIM) ||
       (D->SrcDy > GDIGPU_MAX_DIM) || (D->SrcDy < -GDIGPU_MAX_DIM))) {
    return 0;
  }
  for (I = D->FirstRect; I < D->FirstRect + D->RectCount; I++) {
    const GDIGPU_RECT  *R = &P->Rects[I];

    if (!RectOk (R, P->FbW, P->FbH)) {
      return 0;
    }
    if (Textured (D->Op) &&
        ((R->Left + D->SrcDx < 0) || (R->Top + D->SrcDy < 0) || (R->Right + D->SrcDx > (int)D->SrcW) ||
         (R->Bottom + D->SrcDy > (int)D->SrcH))) {
      return 0;
    }
  }
  return 1;
}

static uint64_t
Get64 (const uint8_t *Base, unsigned Offset)
{
  uint64_t  V = 0;
  int       I;

  for (I = 7; I >= 0; I--) {
    V = (V << 8) | Base[Offset + I];
  }
  return V;
}

static void
PutTexture (uint8_t *S, unsigned Off, uint64_t Va, unsigned W, unsigned H, uint32_t Pitch)
{
  Put32 (S, Off + TEX_SIZE, (W - 1) | ((uint32_t)(H - 1) << 16));
  Put64 (S, Off + SURF_POINTER, Va);
  Put32 (S, Off + SURF_ROW_STRIDE, Pitch);
  Put32 (S, Off + SURF_SURFACE_STRIDE, Pitch * H);
}

int
GdiGpuBuild (uint8_t *S, const uint8_t *Slots, const GDIGPU_PASS *P)
{
  unsigned     Hdr;
  unsigned     Full;
  unsigned     I;
  unsigned     D;
  unsigned     Next;
  GDIGPU_RECT  All;

  //
  // Everything the hardware is told must be representable and in range.
  //
  if ((P->FbW == 0) || (P->FbH == 0) || (P->FbW > GDIGPU_MAX_DIM) || (P->FbH > GDIGPU_MAX_DIM) || ((P->FbVa & 63) != 0) ||
      ((P->FbPitch & 63) != 0) || (P->FbPitch < P->FbW * 4) || (P->DrawCount == 0) || (P->DrawCount > GDIGPU_MAX_DRAWS)) {
    return -1;
  }
  for (D = 0, Next = 0; D < P->DrawCount; D++) {
    if (!DrawOk (P, &P->Draws[D], Next)) {
      return -1;
    }
    Next = P->Draws[D].FirstRect + P->Draws[D].RectCount;
  }
  Hdr  = GdiGpuHeaderSize (P->FbW, P->FbH);
  Full = GdiGpuFullSize (P->FbW, P->FbH);
  if (Full > GDIGPU_POLY_SIZE) {
    return -1;
  }

  //
  // Job headers: the GPU writes its exception status and fault pointer
  // into them - start every pass from clean ones (a fault left in a header
  // from an earlier pass makes the next one fail at once).  The draws'
  // jobs come clean from their slot templates.
  //
  {
    static const unsigned  Jobs[] = { GO_WV, GO_TILER_PRE, GO_FRAG };

    for (I = 0; I < sizeof (Jobs) / sizeof (Jobs[0]); I++) {
      unsigned  K;

      for (K = 0; K < JOB_STATUS_BYTES; K += 4) {
        Put32 (S, Jobs[I] + K, 0);
      }
    }
  }

  //
  // Viewport and scissor = the framebuffer (as the proven UEFI stream and
  // Mesa do; the tiler bins primitives against them).
  //
  Put32 (S, GO_VIEWPORT + VP_MAX_X, GdiGpuFloatBits ((int)P->FbW));
  Put32 (S, GO_VIEWPORT + VP_MAX_X + 4, GdiGpuFloatBits ((int)P->FbH));
  Put32 (S, GO_VIEWPORT + VP_SCISSOR_MAX, (P->FbW - 1) | ((uint32_t)(P->FbH - 1) << 16));

  //
  // Framebuffer: bounds, colour writeback, polygon list.
  //
  Put32 (S, GO_FBD + FBD_P_BOUND_MAX, (P->FbW - 1) | ((uint32_t)(P->FbH - 1) << 16));
  Put64 (S, GO_FBD + FBD_P_WB_BASE, P->FbVa);
  Put32 (S, GO_FBD + FBD_P_WB_STRIDE, P->FbPitch);
  Put32 (S, GO_FBD + FBD_T_SIZE, Full);
  Put64 (S, GO_FBD + FBD_T_BODY, GDIGPU_POLY_BASE + Hdr);
  Put32 (S, GO_FRAG + FJ_BOUND_MAX, ((P->FbW - 1) >> 4) | ((uint32_t)((P->FbH - 1) >> 4) << 16));

  //
  // Preload: the destination as a texture over the whole framebuffer.
  //
  PutTexture (S, GO_TEX_DST, P->FbVa, P->FbW, P->FbH, P->FbPitch);
  All.Left = 0;
  All.Top = 0;
  All.Right = (int)P->FbW;
  All.Bottom = (int)P->FbH;
  PutQuad (S, GO_POS_PRE, &All, 0, 0, 1);
  PutQuad (S, GO_UV_PRE, &All, 0, 0, 0);

  //
  // The draws, chained after the preload in GDI's order.
  //
  for (D = 0; D < P->DrawCount; D++) {
    const GDIGPU_DRAW  *Dr = &P->Draws[D];
    unsigned           Slot = GO_SLOT (D);
    unsigned           V = Dr->RectCount * 6;
    unsigned           First = Dr->FirstRect * 6 * VERTEX_BYTES;

    memcpy (S + Slot, Slots + ((size_t)D * GDIGPU_OPS + (Dr->Op - 1)) * SL_SIZE, SL_SIZE);
    Put64 (S, Slot + SL_TILER + JOB_NEXT, (D + 1 < P->DrawCount) ? (GDIGPU_BASE + GO_SLOT (D + 1)) : 0);
    Put32 (S, Slot + SL_TILER + TJ_INVOCATION, V - 1);
    Put32 (S, Slot + SL_TILER + TJ_PRIM_COUNT, V - 1);
    Put64 (S, Slot + SL_TILER + TJ_POSITION, GDIGPU_BASE + GO_POS_OP + First);
    for (I = 0; I < Dr->RectCount; I++) {
      PutQuad (S, GO_POS_OP + First + I * 6 * VERTEX_BYTES, &P->Rects[Dr->FirstRect + I], 0, 0, 1);
    }
    //
    // Varying buffer 0: uv (textured) or colour (fill); 1: constant alpha.
    // The template points them at the arrays' start (type in the low bits).
    //
    Put64 (S, Slot + SL_VBUF, Get64 (S, Slot + SL_VBUF) + First);
    Put32 (S, Slot + SL_VBUF + VB_SIZE, V * VERTEX_BYTES);
    if (Textured (Dr->Op)) {
      PutTexture (S, Slot + SL_TEX, Dr->SrcVa, Dr->SrcW, Dr->SrcH, Dr->SrcPitch);
      for (I = 0; I < Dr->RectCount; I++) {
        if (Dr->Op == GDIGPU_OP_CT) {
          PutQuadCt (S, GO_UV_OP + First + I * 6 * VERTEX_BYTES, &P->Rects[Dr->FirstRect + I], Dr->SrcDx, Dr->SrcDy);
        } else {
          PutQuad (S, GO_UV_OP + First + I * 6 * VERTEX_BYTES, &P->Rects[Dr->FirstRect + I], Dr->SrcDx, Dr->SrcDy, 0);
        }
      }
    }
    if (Dr->Op == GDIGPU_OP_CT) {
      //
      // Varying buffers: 0 coordinates (above), 1 foreground, 2 full colour.
      //
      uint32_t  Fg[4];
      uint32_t  Fu[4];

      Fg[0] = Unorm8Bits ((Dr->Color >> 16) & 0xFF);
      Fg[1] = Unorm8Bits ((Dr->Color >> 8) & 0xFF);
      Fg[2] = Unorm8Bits (Dr->Color & 0xFF);
      Fg[3] = Unorm8Bits ((Dr->Color >> 24) & 0xFF);
      Fu[0] = Unorm8Bits ((Dr->Color2 >> 16) & 0xFF);
      Fu[1] = Unorm8Bits ((Dr->Color2 >> 8) & 0xFF);
      Fu[2] = Unorm8Bits (Dr->Color2 & 0xFF);
      Fu[3] = Unorm8Bits ((Dr->Color2 >> 24) & 0xFF);
      Put64 (S, Slot + SL_TEX2 + SURF_POINTER, Dr->GammaVa);
      Put64 (S, Slot + SL_VBUF + 16, Get64 (S, Slot + SL_VBUF + 16) + First);
      Put32 (S, Slot + SL_VBUF + 16 + VB_SIZE, V * VERTEX_BYTES);
      Put64 (S, Slot + SL_VBUF + 32, Get64 (S, Slot + SL_VBUF + 32) + First);
      Put32 (S, Slot + SL_VBUF + 32 + VB_SIZE, V * VERTEX_BYTES);
      for (I = 0; I < V; I++) {
        PutVec4 (S, GO_COL_OP + First + I * VERTEX_BYTES, Fg[0], Fg[1], Fg[2], Fg[3]);
        PutVec4 (S, GO_CA_OP + First + I * VERTEX_BYTES, Fu[0], Fu[1], Fu[2], Fu[3]);
      }
    }
    if ((Dr->Op == GDIGPU_OP_ALPHAP) || (Dr->Op == GDIGPU_OP_ALPHAC)) {
      uint32_t  Ca = Unorm8Bits (Dr->ConstantAlpha);

      Put64 (S, Slot + SL_VBUF + 16, Get64 (S, Slot + SL_VBUF + 16) + First);
      Put32 (S, Slot + SL_VBUF + 16 + VB_SIZE, V * VERTEX_BYTES);
      for (I = 0; I < V; I++) {
        PutVec4 (S, GO_CA_OP + First + I * VERTEX_BYTES, Ca, Ca, Ca, Ca);
      }
      if (Dr->Op == GDIGPU_OP_ALPHAC) {
        Put32 (S, Slot + SL_RSD + RSD_BLEND_CONSTANT, Ca);
      }
    } else if (Dr->Op == GDIGPU_OP_FILL) {
      uint32_t  R = Unorm8Bits ((Dr->Color >> 16) & 0xFF);
      uint32_t  G = Unorm8Bits ((Dr->Color >> 8) & 0xFF);
      uint32_t  B = Unorm8Bits (Dr->Color & 0xFF);
      uint32_t  A = Unorm8Bits ((Dr->Color >> 24) & 0xFF);

      for (I = 0; I < V; I++) {
        PutVec4 (S, GO_COL_OP + First + I * VERTEX_BYTES, R, G, B, A);
      }
    }
  }
  return 0;
}
