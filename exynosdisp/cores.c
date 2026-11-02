//
// The display cores (DP TX, FIMD1, panel / backlight sequencing) and the
// bit-banged I2C buses, built unchanged from the UEFI tree; this C file
// also owns the OS log ring for the driver.
//
#define OSLOG_GLOBAL
#include "../../uefi/Exynos5420Pkg/Drivers/Display/ExynosDpCore.c"
#include "../../uefi/Exynos5420Pkg/Drivers/Display/FimdCore.c"
#include "../../uefi/Exynos5420Pkg/Drivers/Display/PanelCore.c"
#include "../../uefi/Exynos5420Pkg/Drivers/I2CDxe/GpioI2c.c"

UINTN  gExynosGpioBBase;

//
// LP8556 writes (PanelCore.c) on bus 19.
//
BOOLEAN
EnvI2cWrite (
  IN UINT32  Bus,
  IN UINT8   Addr,
  IN UINT8   *Buf,
  IN UINTN   Count
  )
{
  EXYNOS_I2C_MSG  Msg;

  Msg.Addr  = Addr;
  Msg.Flags = 0;
  Msg.Len   = (UINT16)Count;
  Msg.Buf   = Buf;
  return GpioI2cXfer (Bus, &Msg, 1) == 1;
}
