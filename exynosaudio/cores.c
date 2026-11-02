//
// The codec and I2S cores, built unchanged from the UEFI tree; this C file
// also owns the OS log ring for the driver.
//
#define OSLOG_GLOBAL
#include "../../uefi/Exynos5420Pkg/Drivers/Audio/Wm5102Core.c"
#include "../../uefi/Exynos5420Pkg/Drivers/Audio/ExynosI2sCore.c"
