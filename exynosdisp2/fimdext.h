/** @file
  FIMD1 extras for exynosdisp2 (see fimdext.c).

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef __FIMDEXT_H__
#define __FIMDEXT_H__

#define FIMD_CURSOR_SIZE          64
#define EXYNOS_COMBINER_PHYS      0x10440000   // 0x1000, shared with exynosgpio (write-1 set / clear registers)

VOID    FimdVsyncIrq (IN EXYNOS_DISPLAY *Disp, IN UINTN Combiner, IN BOOLEAN Enable);
BOOLEAN FimdVsyncAck (IN EXYNOS_DISPLAY *Disp);
VOID    FimdCursorSetup (IN EXYNOS_DISPLAY *Disp, IN UINT32 Phys);
VOID    FimdCursorMove (IN EXYNOS_DISPLAY *Disp, IN INT32 X, IN INT32 Y, IN BOOLEAN Visible);
VOID    FimdFlip (IN EXYNOS_DISPLAY *Disp, IN UINT32 Phys);

#endif
