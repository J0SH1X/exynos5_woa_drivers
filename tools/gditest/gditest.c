/** @file
  Host test of exynoswddm/gdiops.c: every GDI operation against a naive
  reference, with random surfaces, rectangles (also wild ones), pitches and
  too-small buffers; canaries around every buffer catch writes outside the
  bytes a surface owns.

  Build + run: ./run.sh   (clang, AddressSanitizer + UBSan)

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "../../exynoswddm/gdiops.h"

#define PAD     4096
#define CANARY  0xA5

static uint64_t  Seed = 0x1234567887654321ull;

static uint32_t
Rnd (void)
{
  Seed ^= Seed << 13; Seed ^= Seed >> 7; Seed ^= Seed << 17;
  return (uint32_t)(Seed >> 16);
}

static int
RndIn (int Lo, int Hi)           // inclusive
{
  return Lo + (int)(Rnd () % (uint32_t)(Hi - Lo + 1));
}

typedef struct {
  uint8_t   *Raw;                // PAD + Total + PAD
  size_t    Total;               // bytes the test allocated between the canaries
  GDI_SURF  S;                   // as handed to the operations (validated)
  int       Valid;
} TSURF;

static void
MakeSurf (TSURF *T, int Bpp, int MaxDim)
{
  unsigned  W = (unsigned)RndIn (1, (Rnd () % 4 == 0) ? 900 : MaxDim);   // wide ones cross the 256-pixel chunks of gdiops.c
  unsigned  H = (unsigned)RndIn (1, MaxDim);
  unsigned  Pitch = (W * (unsigned)Bpp + 3) & ~3u;
  size_t    I;

  if (Rnd () % 3 == 0) {
    Pitch = (Pitch + 63) & ~63u;                       // padded rows
  }
  T->Total = (size_t)Pitch * H;
  if (Rnd () % 8 == 0) {
    T->Total -= (size_t)RndIn (1, (int)Pitch);         // last row short: the surface claims more than it has
  }
  T->Raw = malloc (T->Total + 2 * PAD);
  memset (T->Raw, CANARY, T->Total + 2 * PAD);
  for (I = 0; I < T->Total; I++) {
    T->Raw[PAD + I] = (uint8_t)Rnd ();
  }
  T->S.Base   = T->Raw + PAD;
  T->S.Pitch  = Pitch;
  T->S.Width  = W;
  T->S.Height = H;
  T->S.Bpp    = (unsigned)Bpp;
  T->S.Bytes  = (unsigned)T->Total;
  if (Rnd () % 16 == 0) {
    T->S.Width += (unsigned)RndIn (1, 50);             // lies about its size
  }
  if (Rnd () % 16 == 0) {
    T->S.Height += (unsigned)RndIn (1, 50);
  }
  T->Valid = GdiSurfValidate (&T->S);
}

static int
CanaryOk (const TSURF *T)
{
  size_t  I;

  for (I = 0; I < PAD; I++) {
    if ((T->Raw[I] != CANARY) || (T->Raw[PAD + T->Total + I] != CANARY)) {
      return 0;
    }
  }
  return 1;
}

static uint8_t *
Snapshot (const TSURF *T)
{
  uint8_t  *C = malloc (T->Total + 1);

  memcpy (C, T->Raw + PAD, T->Total);
  return C;
}

static uint32_t
Px (const uint8_t *Base, const GDI_SURF *S, int X, int Y)
{
  uint32_t  V;

  if (S->Bpp == 1) {
    return Base[(size_t)Y * S->Pitch + (size_t)X];
  }
  memcpy (&V, Base + (size_t)Y * S->Pitch + (size_t)X * 4, 4);
  return V;
}

static void
SetPx (uint8_t *Base, const GDI_SURF *S, int X, int Y, uint32_t V)
{
  if (S->Bpp == 1) {
    Base[(size_t)Y * S->Pitch + (size_t)X] = (uint8_t)V;
  } else {
    memcpy (Base + (size_t)Y * S->Pitch + (size_t)X * 4, &V, 4);
  }
}

static GDI_RECT
RndRect (const GDI_SURF *S)
{
  GDI_RECT  R;

  if (Rnd () % 10 == 0) {                              // wild
    R.Left = RndIn (-100000, 100000); R.Top = RndIn (-100000, 100000);
    R.Right = RndIn (-100000, 100000); R.Bottom = RndIn (-100000, 100000);
    if (Rnd () % 4 == 0) { R.Left = INT32_MIN; }
    if (Rnd () % 4 == 0) { R.Right = INT32_MAX; }
    if (Rnd () % 4 == 0) { R.Top = INT32_MIN; }
    if (Rnd () % 4 == 0) { R.Bottom = INT32_MAX; }
    return R;
  }
  R.Left   = RndIn (-8, (int)S->Width + 4);
  R.Top    = RndIn (-8, (int)S->Height + 4);
  R.Right  = R.Left + RndIn (0, (int)S->Width + 8);
  R.Bottom = R.Top + RndIn (0, (int)S->Height + 8);
  return R;
}

static int
InSurf (const GDI_SURF *S, int X, int Y)
{
  return (X >= 0) && (Y >= 0) && (X < (int)S->Width) && (Y < (int)S->Height);
}

static uint32_t
RefRop3 (uint32_t Rop, uint32_t P, uint32_t S, uint32_t D)
{
  uint32_t  R = 0;
  int       B;

  for (B = 0; B < 32; B++) {
    unsigned  Idx = (((P >> B) & 1) << 2) | (((S >> B) & 1) << 1) | ((D >> B) & 1);

    R |= ((Rop >> Idx) & 1u) << B;
  }
  return R;
}

static unsigned
RefDiv255 (unsigned V)
{
  return (unsigned)((double)V / 255.0 + 0.5);
}

static long  Failures;
static long  Cases;

static void
Compare (const char *What, const TSURF *T, const uint8_t *Expect, long Case)
{
  Cases++;
  if (memcmp (T->Raw + PAD, Expect, T->Total) != 0) {
    size_t  I;

    for (I = 0; I < T->Total; I++) {
      if (T->Raw[PAD + I] != Expect[I]) {
        break;
      }
    }
    if (Failures++ < 20) {
      printf ("FAIL %s case %ld: byte %zu (x %zu y %zu) got %02x want %02x\n", What, Case, I,
              (I % T->S.Pitch) / T->S.Bpp, I / T->S.Pitch, T->Raw[PAD + I], Expect[I]);
    }
  }
  if (!CanaryOk (T)) {
    if (Failures++ < 20) {
      printf ("FAIL %s case %ld: wrote outside the surface\n", What, Case);
    }
  }
}

// --- the tests ---------------------------------------------------------------------------------------------------

static void
TestFill (long N)
{
  long  C;

  for (C = 0; C < N; C++) {
    TSURF     T;
    GDI_RECT  R;
    uint32_t  Color = Rnd () ^ (Rnd () << 16);
    unsigned  Rop = (unsigned)RndIn (0, 8);
    unsigned  Rop3 = Rnd () & 0xFF;
    uint8_t   *E;
    int       X, Y;

    MakeSurf (&T, (Rnd () % 4 == 0) ? 1 : 4, 70);
    if (T.Valid) {
      R = RndRect (&T.S);
      E = Snapshot (&T);
      for (Y = 0; Y < (int)T.S.Height; Y++) {
        for (X = 0; X < (int)T.S.Width; X++) {
          uint32_t  D = Px (E, &T.S, X, Y), V = D, Cc = (T.S.Bpp == 1) ? (Color & 0xFF) : Color;

          if ((X < R.Left) || (X >= R.Right) || (Y < R.Top) || (Y >= R.Bottom)) {
            continue;
          }
          switch (Rop) {
            case 1: V = Cc; break;
            case 2: V = D ^ Cc; break;
            case 3: V = ~(Cc ^ D); break;
            case 4: V = ~D; break;
            case 5: V = D & Cc; break;
            case 6: V = D | Cc; break;
            case 7: V = RefRop3 (Rop3, Cc, 0, D); break;
            default: break;
          }
          SetPx (E, &T.S, X, Y, V);
        }
      }
      GdiColorFill (&T.S, &R, Color, Rop, Rop3);
      Compare ("ColorFill", &T, E, C);
      free (E);
    }
    free (T.Raw);
  }
}

static void
TestBlt (long N)
{
  long  C;

  for (C = 0; C < N; C++) {
    TSURF     D, S;
    int       Same = (Rnd () % 3 == 0);
    int       Bpp = (Rnd () % 5 == 0) ? 1 : 4;
    GDI_RECT  R;
    int       Dx, Dy, X, Y;
    unsigned  Rop = (unsigned)RndIn (0, 6);
    unsigned  Rop3 = Rnd () & 0xFF;
    uint8_t   *E, *Ss;

    MakeSurf (&D, Bpp, 60);
    if (Same) {
      S = D;
    } else {
      MakeSurf (&S, (Rnd () % 20 == 0) ? (5 - Bpp) : Bpp, 60);
    }
    if (D.Valid && S.Valid) {
      R  = RndRect (&D.S);
      Dx = (Rnd () % 12 == 0) ? RndIn (-100000, 100000) : RndIn (-12, 12);
      Dy = (Rnd () % 12 == 0) ? RndIn (-100000, 100000) : RndIn (-12, 12);
      if (Rnd () % 6 == 0) { Dy = 0; }
      if (Rnd () % 6 == 0) { Dx = 0; }
      E  = Snapshot (&D);
      Ss = Snapshot (&S);                       // the source as it was: the reference of an overlapping copy
      if ((D.S.Bpp == S.S.Bpp) && (Dx <= 16384) && (Dx >= -16384) && (Dy <= 16384) && (Dy >= -16384)) {
        for (Y = 0; Y < (int)D.S.Height; Y++) {
          for (X = 0; X < (int)D.S.Width; X++) {
            uint32_t  Dv, Sv, V;

            if ((X < R.Left) || (X >= R.Right) || (Y < R.Top) || (Y >= R.Bottom) || !InSurf (&S.S, X + Dx, Y + Dy)) {
              continue;
            }
            Dv = Px (E, &D.S, X, Y);
            Sv = Px (Ss, &S.S, X + Dx, Y + Dy);
            V  = Dv;
            switch (Rop) {
              case 1: V = Sv; break;
              case 2: V = Dv ^ Sv; break;
              case 3: V = Dv & Sv; break;
              case 4: V = Dv | Sv; break;
              case 5: V = RefRop3 (Rop3, 0, Sv, Dv); break;
              default: break;
            }
            SetPx (E, &D.S, X, Y, V);
          }
        }
      }
      GdiBitBlt (&D.S, &S.S, &R, Dx, Dy, Rop, Rop3);
      Compare (Same ? "BitBlt(same surface)" : "BitBlt", &D, E, C);
      if (!Same && (memcmp (S.Raw + PAD, Ss, S.Total) != 0)) {
        Failures++;
        printf ("FAIL BitBlt case %ld: source modified\n", C);
      }
      free (E);
      free (Ss);
    }
    if (!Same) {
      free (S.Raw);
    }
    free (D.Raw);
  }
}

static int
RefMap (int D, int DstLo, int DstSize, int SrcLo, int SrcSize, int Mirror, int Limit)
{
  long long  I = D - DstLo;
  long long  V;

  if (I < 0) { I = 0; }
  if (I >= DstSize) { I = DstSize - 1; }
  V = ((2 * I + 1) * (long long)SrcSize) / (2 * (long long)DstSize);              // truncate ((i + 0.5) * Ws / Wd)
  V = Mirror ? (SrcLo + SrcSize - 1 - V) : (SrcLo + V);                         // GDI mirrors in source space
  if (V < 0) { V = 0; }
  if (V >= Limit) { V = Limit - 1; }
  return (int)V;
}

static void
RndStretchRects (const GDI_SURF *D, const GDI_SURF *S, GDI_RECT *Sr, GDI_RECT *Dr, int Unit)
{
  Sr->Left   = RndIn (0, (int)S->Width - 1);
  Sr->Top    = RndIn (0, (int)S->Height - 1);
  Sr->Right  = RndIn (Sr->Left + 1, (int)S->Width);
  Sr->Bottom = RndIn (Sr->Top + 1, (int)S->Height);
  Dr->Left   = RndIn (-6, (int)D->Width - 1);
  Dr->Top    = RndIn (-6, (int)D->Height - 1);
  if (Unit) {
    Dr->Right  = Dr->Left + (Sr->Right - Sr->Left);
    Dr->Bottom = Dr->Top + (Sr->Bottom - Sr->Top);
  } else {
    Dr->Right  = Dr->Left + RndIn (1, 2 * (int)D->Width);
    Dr->Bottom = Dr->Top + RndIn (1, 2 * (int)D->Height);
  }
  if (Rnd () % 25 == 0) { Sr->Right = Sr->Left; }                   // degenerate
  if (Rnd () % 25 == 0) { Dr->Bottom = Dr->Top - 3; }
  if (Rnd () % 25 == 0) { Sr->Left -= 40; }                         // outside the source: clamped, not read
  if (Rnd () % 25 == 0) { Sr->Bottom += 40; }
}

static int
RectsOk (const GDI_RECT *Sr, const GDI_RECT *Dr)
{
  return (Sr->Right > Sr->Left) && (Sr->Bottom > Sr->Top) && (Dr->Right > Dr->Left) && (Dr->Bottom > Dr->Top);
}

static void
TestStretch (long N)
{
  long  C;

  for (C = 0; C < N; C++) {
    TSURF     D, S;
    GDI_RECT  R, Sr, Dr;
    unsigned  Mode = (unsigned)RndIn (1, 4);
    int       Mx = Rnd () % 3 == 0, My = Rnd () % 3 == 0, X, Y;
    uint8_t   *E;

    MakeSurf (&D, 4, 40);
    MakeSurf (&S, 4, 40);
    if (D.Valid && S.Valid) {
      RndStretchRects (&D.S, &S.S, &Sr, &Dr, Rnd () % 4 == 0);
      R = (Rnd () % 3 == 0) ? Dr : RndRect (&D.S);
      E = Snapshot (&D);
      if (RectsOk (&Sr, &Dr)) {
        int  Ws = Sr.Right - Sr.Left, Wd = Dr.Right - Dr.Left, Hs = Sr.Bottom - Sr.Top, Hd = Dr.Bottom - Dr.Top;
        int  Combine = ((Mode == 1) || (Mode == 2)) && ((Ws > Wd) || (Hs > Hd));

        for (Y = 0; Y < (int)D.S.Height; Y++) {
          for (X = 0; X < (int)D.S.Width; X++) {
            uint32_t  V;

            if ((X < R.Left) || (X >= R.Right) || (Y < R.Top) || (Y >= R.Bottom)) {
              continue;
            }
            if (!Combine) {
              V = Px (S.Raw + PAD, &S.S, RefMap (X, Dr.Left, Wd, Sr.Left, Ws, Mx, (int)S.S.Width),
                      RefMap (Y, Dr.Top, Hd, Sr.Top, Hs, My, (int)S.S.Height));
            } else {
              //
              // Brute force: every source pixel of the rectangle that falls into this destination pixel.
              //
              long long  Ix = X - Dr.Left, Iy = Y - Dr.Top;
              int        Sx, Sy, Any = 0;

              if (Ix < 0) { Ix = 0; }
              if (Ix >= Wd) { Ix = Wd - 1; }
              if (Iy < 0) { Iy = 0; }
              if (Iy >= Hd) { Iy = Hd - 1; }
              V = (Mode == 1) ? 0xFFFFFFFFu : 0;
              for (Sy = 0; Sy < Hs; Sy++) {
                long long  Ty = (Hs > Hd) ? ((long long)Sy * Hd / Hs) : -1;
                int        Yy;

                if (Hs > Hd) {
                  // source row Sy belongs to destination Iy iff floor (Iy * Hs / Hd) <= Sy < floor ((Iy + 1) * Hs / Hd)
                  if (!((Iy * Hs / Hd <= Sy) && (Sy < (Iy + 1) * Hs / Hd))) { continue; }
                } else if (Sy != (int)(((2 * Iy + 1) * Hs) / (2 * (long long)Hd))) {
                  continue;
                }
                (void)Ty;
                Yy = My ? (Sr.Top + Hs - 1 - Sy) : (Sy + Sr.Top);
                if (Yy < 0) { Yy = 0; }
                if (Yy >= (int)S.S.Height) { Yy = (int)S.S.Height - 1; }
                for (Sx = 0; Sx < Ws; Sx++) {
                  int  Xx;

                  if (Ws > Wd) {
                    if (!((Ix * Ws / Wd <= Sx) && (Sx < (Ix + 1) * Ws / Wd))) { continue; }
                  } else if (Sx != (int)(((2 * Ix + 1) * Ws) / (2 * (long long)Wd))) {
                    continue;
                  }
                  Xx = Mx ? (Sr.Left + Ws - 1 - Sx) : (Sx + Sr.Left);
                  if (Xx < 0) { Xx = 0; }
                  if (Xx >= (int)S.S.Width) { Xx = (int)S.S.Width - 1; }
                  if (Mode == 1) { V &= Px (S.Raw + PAD, &S.S, Xx, Yy); } else { V |= Px (S.Raw + PAD, &S.S, Xx, Yy); }
                  Any = 1;
                }
              }
              if (!Any) {
                continue;        // cannot happen for in-range rectangles; keep the destination
              }
            }
            SetPx (E, &D.S, X, Y, V);
          }
        }
      }
      GdiStretchBlt (&D.S, &S.S, &R, &Sr, &Dr, Mode, Mx, My);
      Compare ("StretchBlt", &D, E, C);
      free (E);
    }
    free (S.Raw);
    free (D.Raw);
  }
}

static uint32_t
RefBlend (uint32_t S, uint32_t D, unsigned Ca, int HasAlpha)
{
  uint32_t  Out = 0;
  int       Sh;

  for (Sh = 0; Sh < 32; Sh += 8) {
    unsigned  Sc = (S >> Sh) & 0xFF, Dc = (D >> Sh) & 0xFF, V;

    if (HasAlpha) {
      unsigned  Sa = S >> 24;

      if (Ca != 255) {
        Sc = RefDiv255 (Sc * Ca);
        Sa = RefDiv255 (Sa * Ca);
      }
      V = Sc + RefDiv255 (Dc * (255 - Sa));
      if (V > 255) { V = 255; }
    } else {
      V = RefDiv255 (Sc * Ca + Dc * (255 - Ca));
    }
    Out |= V << Sh;
  }
  return Out;
}

static void
TestAlphaAndTransparent (long N)
{
  long  C;

  for (C = 0; C < N; C++) {
    TSURF     D, S;
    GDI_RECT  R, Sr, Dr;
    int       Transparent = Rnd () % 2, HasAlpha = Rnd () % 2, Honor = Rnd () % 2, X, Y;
    unsigned  Ca = (Rnd () % 4 == 0) ? 255 : (Rnd () % 8 == 0) ? 0 : (Rnd () & 0xFF);
    uint32_t  Key;
    uint8_t   *E;
    size_t    I;

    MakeSurf (&D, 4, 40);
    MakeSurf (&S, 4, 40);
    if (D.Valid && S.Valid) {
      // few distinct colours so the colour key hits; premultiplied-looking pixels now and then
      for (I = 0; I + 4 <= S.Total; I += 4) {
        uint32_t  V = (Rnd () % 3 == 0) ? 0x00112233u : (Rnd () % 3 == 0) ? 0xFF112233u : Rnd () ^ (Rnd () << 16);

        if (Rnd () % 5 == 0) { V = 0; }
        if (Rnd () % 5 == 0) { V |= 0xFF000000u; }
        memcpy (S.Raw + PAD + I, &V, 4);
      }
      Key = (Rnd () % 2) ? 0x00112233u : 0xFF112233u;
      RndStretchRects (&D.S, &S.S, &Sr, &Dr, Rnd () % 2);
      R = (Rnd () % 3 == 0) ? Dr : RndRect (&D.S);
      E = Snapshot (&D);
      if (RectsOk (&Sr, &Dr)) {
        int  Ws = Sr.Right - Sr.Left, Wd = Dr.Right - Dr.Left, Hs = Sr.Bottom - Sr.Top, Hd = Dr.Bottom - Dr.Top;

        for (Y = 0; Y < (int)D.S.Height; Y++) {
          for (X = 0; X < (int)D.S.Width; X++) {
            uint32_t  Sv, Dv;

            if ((X < R.Left) || (X >= R.Right) || (Y < R.Top) || (Y >= R.Bottom)) {
              continue;
            }
            Sv = Px (S.Raw + PAD, &S.S, RefMap (X, Dr.Left, Wd, Sr.Left, Ws, 0, (int)S.S.Width),
                     RefMap (Y, Dr.Top, Hd, Sr.Top, Hs, 0, (int)S.S.Height));
            Dv = Px (E, &D.S, X, Y);
            if (Transparent) {
              uint32_t  Mask = Honor ? 0xFFFFFFFFu : 0x00FFFFFFu;

              if ((Sv & Mask) != (Key & Mask)) {
                SetPx (E, &D.S, X, Y, Sv);
              }
            } else {
              SetPx (E, &D.S, X, Y, RefBlend (Sv, Dv, Ca, HasAlpha));
            }
          }
        }
      }
      if (Transparent) {
        GdiTransparentBlt (&D.S, &S.S, &R, &Sr, &Dr, Key, Honor);
        Compare ("TransparentBlt", &D, E, C);
      } else {
        GdiAlphaBlend (&D.S, &S.S, &R, &Sr, &Dr, Ca, HasAlpha);
        Compare ("AlphaBlend", &D, E, C);
      }
      free (E);
    }
    free (S.Raw);
    free (D.Raw);
  }
}

static void
TestClearType (long N)
{
  long  C;

  for (C = 0; C < N; C++) {
    TSURF     D, A, G;
    GDI_RECT  R;
    int       OffX = RndIn (-10, 10), OffY = RndIn (-10, 10), X, Y, Sh;
    uint32_t  Color = Rnd () ^ (Rnd () << 16), Color2 = Rnd () ^ (Rnd () << 16);
    unsigned  Gamma = (Rnd () % 3 == 0) ? GDI_GAMMA_INVALID : (unsigned)RndIn (0, 17);
    int       NoTable = Rnd () % 10 == 0;
    uint8_t   *E;
    size_t    I;

    MakeSurf (&D, 4, 40);
    MakeSurf (&A, 4, 50);
    // gamma surface: 512 x 16 A8, random tables
    G.Total = 512 * 16;
    G.Raw   = malloc (G.Total + 2 * PAD);
    memset (G.Raw, CANARY, G.Total + 2 * PAD);
    for (I = 0; I < G.Total; I++) { G.Raw[PAD + I] = (uint8_t)Rnd (); }
    G.S.Base = G.Raw + PAD; G.S.Pitch = 512; G.S.Width = 512; G.S.Height = 16; G.S.Bpp = 1; G.S.Bytes = 512 * 16;
    G.Valid = GdiSurfValidate (&G.S);
    if (Rnd () % 12 == 0) { OffX = RndIn (-100000, 100000); }
    if (D.Valid && A.Valid && G.Valid) {
      // coverage: mostly 0 and 255, some in between
      for (I = 0; I + 4 <= A.Total; I += 4) {
        uint32_t  V = (Rnd () % 3 == 0) ? 0 : (Rnd () % 3 == 0) ? 0x00FFFFFFu : Rnd () ^ (Rnd () << 16);

        if (Rnd () % 6 == 0) { V = (V & 0xFF00FFFFu) | 0x00FF0000u; }
        if (Rnd () % 6 == 0) { V &= 0xFFFF00FFu; }
        memcpy (A.Raw + PAD + I, &V, 4);
      }
      R = RndRect (&D.S);
      E = Snapshot (&D);
      if ((OffX <= 16384) && (OffX >= -16384)) {
        for (Y = 0; Y < (int)D.S.Height; Y++) {
          for (X = 0; X < (int)D.S.Width; X++) {
            uint32_t  Av, Dv, Out;

            if ((X < R.Left) || (X >= R.Right) || (Y < R.Top) || (Y >= R.Bottom) || !InSurf (&A.S, X + OffX, Y + OffY)) {
              continue;
            }
            Av  = Px (A.Raw + PAD, &A.S, X + OffX, Y + OffY);
            Dv  = Px (E, &D.S, X, Y);
            Out = Dv & 0xFF000000u;
            for (Sh = 0; Sh < 24; Sh += 8) {
              unsigned  Dc = (Dv >> Sh) & 0xFF, Cc = (Color >> Sh) & 0xFF, C2 = (Color2 >> Sh) & 0xFF, Ac, V;

              if (Gamma == GDI_GAMMA_INVALID) {
                Ac = (Cc >= Dc) ? ((Av >> 16) & 0xFF) : ((Av >> 8) & 0xFF);
                V  = (Ac == 0) ? Dc : (Ac == 255) ? C2 : (unsigned)((double)Dc + ((double)Cc - (double)Dc) * Ac / 255.0 + 0.5);
              } else {
                Ac = (Av >> Sh) & 0xFF;
                if (Ac == 0) {
                  V = Dc;
                } else if (Ac == 255) {
                  V = C2;
                } else if (!NoTable && (Gamma < 16)) {
                  const uint8_t  *Gt = G.Raw + PAD + Gamma * 512, *Ig = Gt + 256;
                  double         T = Gt[Dc];

                  V = Ig[(unsigned)(T + ((double)Cc - T) * Ac / 255.0 + 0.5)];
                } else {
                  V = (unsigned)((double)Dc + ((double)C2 - (double)Dc) * Ac / 255.0 + 0.5);
                }
              }
              Out |= V << Sh;
            }
            SetPx (E, &D.S, X, Y, Out);
          }
        }
      }
      GdiClearTypeBlend (&D.S, &A.S, NoTable ? NULL : &G.S, &R, OffX, OffY, Color, Color2, Gamma);
      Compare ("ClearTypeBlend", &D, E, C);
      if (!CanaryOk (&G) || !CanaryOk (&A)) {
        Failures++;
        printf ("FAIL ClearTypeBlend case %ld: alpha / gamma surface written\n", C);
      }
      free (E);
    }
    free (G.Raw);
    free (A.Raw);
    free (D.Raw);
  }
}

static void
TestRop3Names (void)
{
  uint32_t  P = 0x0F0F3355u, S = 0x00FF5A5Au, D = 0x33CC0FF0u;
  unsigned  Rop;

  for (Rop = 0; Rop < 256; Rop++) {
    Cases++;
    if (GdiRop3 (Rop, P, S, D) != RefRop3 (Rop, P, S, D)) {
      Failures++;
      printf ("FAIL Rop3 0x%02x\n", Rop);
    }
  }
  // the named ones
  if ((GdiRop3 (0xCC, P, S, D) != S) || (GdiRop3 (0xF0, P, S, D) != P) || (GdiRop3 (0x66, P, S, D) != (S ^ D)) ||
      (GdiRop3 (0x88, P, S, D) != (S & D)) || (GdiRop3 (0xEE, P, S, D) != (S | D)) || (GdiRop3 (0x55, P, S, D) != ~D) ||
      (GdiRop3 (0x5A, P, S, D) != (P ^ D)) || (GdiRop3 (0x00, P, S, D) != 0) || (GdiRop3 (0xFF, P, S, D) != 0xFFFFFFFFu)) {
    Failures++;
    printf ("FAIL Rop3 named codes\n");
  }
}

int
main (int argc, char **argv)
{
  long  N = (argc > 1) ? atol (argv[1]) : 20000;

  TestRop3Names ();
  TestFill (N);
  TestBlt (N * 2);
  TestStretch (N);
  TestAlphaAndTransparent (N * 2);
  TestClearType (N);
  printf ("%ld cases, %ld failures\n", Cases, Failures);
  return Failures ? 1 : 0;
}
