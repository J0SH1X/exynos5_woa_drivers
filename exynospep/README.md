# exynospep — power engine plugin (PoFx) for Windows RT 8.1

Status (2026-11-03): **v0 prepared, not run.** Separate from `../build.sh`
and Artifacts5; `./exynospep/build.sh` puts inf/sys/cat into
`exynospep/out`. Firmware with the `PEP0` device (and `GPU0`):
`uefi/Exynos5420Pkg/Artifacts/boot-v1awifi-dma-gpu-pep.img`.

## What a PEP is for on RT 8.1

The kernel offers every power-managed device to the plugin (DPM
notifications), and the ARM processor driver `fxppm.sys` asks it for the
processors' idle and performance states (PPM notifications). With no PEP
registered, the processors get no power management from Windows here.

Taken from RT 8.1's own binaries (boot.wim ntoskrnl.exe + fxppm.sys with
their public PDBs):

- `PoFxRegisterPlugin` → `PopFxRegisterPluginEx`: `PEP_INFORMATION` must be
  **version 2, 12 bytes** (device + processor callbacks, device one
  non-null); kernel information version 1 (≥ 28 bytes) or 2 (≥ 44).
- PPM notification numbers `fxppm.sys` sends: 0x01 capabilities, 0x02 /
  0x12 idle states v1 / v2, 0x03 select, 0x04 cancel, 0x05 execute, 0x06
  complete, 0x07 halted, 0x08 wake, 0x09 / 0x0A feedback counters, 0x0B
  perf capabilities, 0x0C perf constraints, 0x0D perf set, 0x0E parking,
  0x10 / 0x13 / 0x17 platform states, 0x11 LP settings, 0x14 test idle.
- WDK10's `pepfx.h` does not build for NTDDI 8.1, so the layouts are
  declared in `exynospep.c` (from `pep_x.h`).

## v0

- registers in `DriverEntry` (boot start, group Boot Bus Extender), logs
  every device ID the kernel offers;
- accepts the processors (`ACPI0007`): one idle state, **WFI (C1)** — the
  only runtime state the 5420 supports in practice (`~/exynos5/cpuidle.md`:
  C2 disabled for the 5420 downstream, AFTR / LPA only for the last A7);
- no performance states, no platform states, no parking;
- declines every other device (their power handling is unchanged).

## Test

1. Flash `boot-v1awifi-dma-gpu-pep.img` (adds `PEP0`, `GPU0`).
2. `pnputil -a exynospep.inf` on `ACPI\EXYN5014`, reboot.
3. Log: `PoFxRegisterPlugin ... 0x00000000`, the device list, `processor N
   registered` for 0..3, PPM notifications 0x01 / 0x12 (or 0x02).
4. If the boot hangs: TWRP mass storage, rename
   `Windows\System32\drivers\exynospep.sys`, then read the log ring from
   UEFI (`usbdbg.py oslog`).

## Next

- **CPU DVFS** (`PPM_QUERY_PERF_CAPABILITIES` / `PERF_SET`, 0x0B / 0x0D):
  one domain for the four A15s, 200..1800 MHz from the kernel's table
  (APLL M/P/S, dividers, VDD_ARM per ASV group 0 + margin), applied from a
  work item (BUCK2 over the PMIC's I2C needs PASSIVE level; raise voltage
  before frequency, lower after) — CpuFreqDxe's sequence, moved into a
  shared core. `PEP0` then gets the PMIC as an `I2cSerialBus` resource.
- Devices: accept the ones whose power we can actually cut (display
  already does it itself, GPU, Wi-Fi, BT) once their drivers use PoFx.
- Platform idle states / connected standby: needs the PMU sleep sequence
  (cpuidle.md §suspend) and wake sources — later.
