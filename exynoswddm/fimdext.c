/** @file
  FIMD1 extras for exynosdisp2 (not in the shared UEFI cores yet): the
  frame (vsync) interrupt through combiner group 18 input 5 (GSI 50), and
  window 1 as a 64x64 ARGB hardware cursor with per-pixel alpha.

  Registers from mainline (include/video/samsung_fimd.h,
  drivers/gpu/drm/exynos/exynos_drm_fimd.c: fimd_enable_vblank,
  fimd_win_set_pixfmt / bldmod / bldeq, the 4-word burst for narrow
  windows); combiner layout as in exynosgpio (group g: set (g / 4) * 0x10,
  bit (g % 4) * 8 + input).

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include "ExynosEnv.h"
#include "../../uefi/Exynos5420Pkg/Drivers/Display/ExynosDisplay.h"
#include "fimdext.h"

#define VIDINTCON0              0x130
#define VIDINTCON0_INT_ENABLE   (1u << 0)
#define VIDINTCON0_INT_FRAME    (1u << 12)
#define VIDINTCON0_FRAMESEL0_MASK  (3u << 15)
#define VIDINTCON0_FRAMESEL0_VSYNC (1u << 15)
#define VIDINTCON1              0x134
#define VIDINTCON1_INT_FRAME    (1u << 1)

#define COMBINER_SET_18         0x40               // groups 16..19
#define COMBINER_ENABLE_SET     0x0
#define COMBINER_ENABLE_CLEAR   0x4
#define COMBINER_BIT_FIMD1_VSYNC (1u << (2 * 8 + 5))

#define WINCON1                 0x024
#define WINCON1_ENWIN           (1u << 0)
#define WINCON1_ALPHA_SEL       (1u << 1)
#define WINCON1_BPP_A8888       (0xDu << 2)       // "25 bpp A1888" mode, 8-bit alpha with BLENDCON
#define WINCON1_BLD_PIX         (1u << 6)
#define WINCON1_ALPHA_MUL       (1u << 7)
#define WINCON1_BURSTLEN_4WORD  (2u << 9)
#define WINCON1_WSWP            (1u << 15)
#define SHADOWCON               0x034
#define SHADOWCON_CH1           (1u << 1)
#define VIDOSD1A                0x050
#define VIDOSD1B                0x054
#define VIDOSD1C                0x058
#define VIDOSD1D                0x05C
#define VIDW01ADD0B0            0x0A8
#define VIDW01ADD1B0            0x0D8
#define VIDW01ADD2              0x104
#define VIDW1ALPHA0             0x224
#define VIDW1ALPHA1             0x228
#define WIN1MAP                 0x184
#define BLENDEQ1                0x244
#define BLENDCON                0x260
#define BLENDEQ_ALPHA_A         0x2
#define BLENDEQ_ONE_MINUS_ALPHA_A 0x3
#define WIN1_RGB_ORDER          0x2024

#define X_POS(v)  ((((v) & 0x7FFu) << 11) | ((((v) & 0x800u) >> 11) << 23))
#define Y_POS(v)  (((v) & 0x7FFu) | ((((v) & 0x800u) >> 11) << 22))

VOID
FimdVsyncIrq (
  IN EXYNOS_DISPLAY  *Disp,
  IN UINTN           Combiner,
  IN BOOLEAN         Enable
  )
{
  UINT32  Con = MmioRead32 (Disp->Fimd + VIDINTCON0);

  if (Enable) {
    Con &= ~VIDINTCON0_FRAMESEL0_MASK;
    Con |= VIDINTCON0_INT_ENABLE | VIDINTCON0_INT_FRAME | VIDINTCON0_FRAMESEL0_VSYNC;
    MmioWrite32 (Disp->Fimd + VIDINTCON1, VIDINTCON1_INT_FRAME);
    MmioWrite32 (Disp->Fimd + VIDINTCON0, Con);
    MmioWrite32 (Combiner + COMBINER_SET_18 + COMBINER_ENABLE_SET, COMBINER_BIT_FIMD1_VSYNC);
  } else {
    MmioWrite32 (Disp->Fimd + VIDINTCON0, Con & ~(VIDINTCON0_INT_ENABLE | VIDINTCON0_INT_FRAME));
    MmioWrite32 (Disp->Fimd + VIDINTCON1, VIDINTCON1_INT_FRAME);
  }
}

BOOLEAN
FimdVsyncAck (
  IN EXYNOS_DISPLAY  *Disp
  )
{
  if ((MmioRead32 (Disp->Fimd + VIDINTCON1) & VIDINTCON1_INT_FRAME) == 0) {
    return FALSE;
  }
  MmioWrite32 (Disp->Fimd + VIDINTCON1, VIDINTCON1_INT_FRAME);
  return TRUE;
}

VOID
FimdCursorSetup (
  IN EXYNOS_DISPLAY  *Disp,
  IN UINT32          Phys          // 64 x 64 ARGB8888, stride 256
  )
{
  MmioWrite32 (Disp->Fimd + WINCON1, 0);
  MmioWrite32 (Disp->Fimd + VIDW01ADD0B0, Phys);
  MmioWrite32 (Disp->Fimd + VIDW01ADD1B0, Phys + FIMD_CURSOR_SIZE * FIMD_CURSOR_SIZE * 4);
  MmioWrite32 (Disp->Fimd + VIDW01ADD2, FIMD_CURSOR_SIZE * 4);      // no offset, page width 256
  MmioWrite32 (Disp->Fimd + VIDOSD1D, FIMD_CURSOR_SIZE * FIMD_CURSOR_SIZE);
  MmioWrite32 (Disp->Fimd + WIN1MAP, 0);
  MmioWrite32 (Disp->Fimd + WIN1_RGB_ORDER, 0);
  // opaque plane alpha, the pixels' own alpha blends (coverage)
  MmioWrite32 (Disp->Fimd + VIDOSD1C, (0xFu << 20) | (0xFu << 16) | (0xFu << 12));
  MmioWrite32 (Disp->Fimd + VIDW1ALPHA0, (0xFu << 16) | (0xFu << 8) | 0xFu);
  MmioWrite32 (Disp->Fimd + VIDW1ALPHA1, 0);
  MmioAndThenOr32 (Disp->Fimd + BLENDEQ1, ~0x3CFu, BLENDEQ_ALPHA_A | (BLENDEQ_ONE_MINUS_ALPHA_A << 6));
  MmioOr32 (Disp->Fimd + BLENDCON, 1u);       // 8-bit alpha values
  MmioOr32 (Disp->Fimd + SHADOWCON, SHADOWCON_CH1);
}

VOID
FimdCursorMove (
  IN EXYNOS_DISPLAY  *Disp,
  IN INT32           X,             // panel coordinates of the top left corner
  IN INT32           Y,
  IN BOOLEAN         Visible
  )
{
  UINT32  Left;
  UINT32  Top;

  if (!Visible) {
    MmioWrite32 (Disp->Fimd + WINCON1, 0);
    return;
  }
  //
  // FIMD windows cannot start off screen: clamp (the cursor sticks at the
  // right / bottom edge for its last 64 pixels).
  //
  Left = (UINT32)MAX (0, MIN (X, EXYNOS_DISP_WIDTH - FIMD_CURSOR_SIZE));
  Top  = (UINT32)MAX (0, MIN (Y, EXYNOS_DISP_HEIGHT - FIMD_CURSOR_SIZE));
  MmioWrite32 (Disp->Fimd + VIDOSD1A, X_POS (Left) | Y_POS (Top));
  MmioWrite32 (Disp->Fimd + VIDOSD1B, X_POS (Left + FIMD_CURSOR_SIZE - 1) | Y_POS (Top + FIMD_CURSOR_SIZE - 1));
  MmioWrite32 (Disp->Fimd + WINCON1, WINCON1_ENWIN | WINCON1_BPP_A8888 | WINCON1_WSWP | WINCON1_BURSTLEN_4WORD |
                                     WINCON1_ALPHA_SEL | WINCON1_BLD_PIX | WINCON1_ALPHA_MUL);
}

/**
  Flip window 0 to another buffer: both address registers written under
  SHADOWCON's window 0 protect bit, so the next vsync latches them together.
**/
VOID
FimdFlip (
  IN EXYNOS_DISPLAY  *Disp,
  IN UINT32          Phys
  )
{
  MmioOr32 (Disp->Fimd + SHADOWCON, 1u << 10);
  FimdSetScanout (Disp, Phys);
  MmioAnd32 (Disp->Fimd + SHADOWCON, ~(1u << 10));
}
