# exynosgpu — Mali-T628 MP6 on Windows RT 8.1

Status (2026-11-03): **phase 1 shader pass works in UEFI** (GpuTest step 6: tiler + fragment, gradient shader and texture-sampled blit, pixels exact; stream from ext/gpugen/gen_blit.c). **Phase 0 passes in UEFI and in Windows** (1.0.0.71: NULL, WRITE_VALUE, clear-only fragment job, all completed by the job interrupt; MP6 r0p1, shaders 0x3f). Timestamp write gives 0 (counter not started). The PMIC open is busy (exclusive connection held by mxt1664s), so vdd_g3d stays at the bootloader’s 1025 mV until a shared PMIC driver exists.

## What exists

| Piece | Where | State |
|---|---|---|
| Shared core: G3D clock (VPLL → aclk_g3d), G3D power domain, soft reset, T62x quirks, L2 / shader / tiler power-up, Mali LPAE page tables, address space setup, job slot submission, NULL / WRITE_VALUE job builders | `uefi/Exynos5420Pkg/Drivers/Gpu/MaliCore.{c,h}` | builds (UEFI + Windows), untested |
| UEFI test: vdd_g3d (BUCK4), power-up, IDs, page tables (GPU VA = PA), NULL job, WRITE_VALUE job, NULL → WRITE_VALUE chain, power-down | `uefi/Exynos5420Pkg/Application/GpuTestApp`, staged in `~/exynos5/gpu-test` | builds, untested |
| Windows KMDF driver: resources (registers, job / MMU / GPU interrupts, PMIC), vdd_g3d over SPB, power-up in D0, interrupt-driven self-test (NULL + WRITE_VALUE) | `exynosgpu/` (EXYN5013) | builds, untested |
| DSDT `GPU0` (EXYN5013) | `Exynos5420Pkg/AcpiTables/Dsdt/Dsdt.asl` | builds |

Sources: mainline Panfrost (register map, reset / quirks / power-on, MMU,
job submission; `panfrost_issues.h` T620), `io-pgtable-arm.c` (Mali LPAE
format), Samsung's r15p0 `mali_kbase` (`platform/exynos/gpu_exynos5420.c`,
`mach-exynos/clock-exynos5420.c`: clock path, power domain, OPPs), Mesa's
Panfrost genxml (job header, WRITE_VALUE payload — from memory, to be
confirmed by the WRITE_VALUE test). Mainline marks T628 support as partial;
where the two trees differ the downstream SoC code wins for the Exynos side
and Panfrost for the GPU side.

OPPs (gpu_exynos5420.c): 533 / 480 / 420 / 350 / 266 / 177 MHz at
1037.5 / 1000 / 962.5 / 912.5 / 862.5 / 812.5 mV (BUCK4). The tests use
266 MHz at 887.5 mV.

## UEFI test (next on the device)

    cd ~/exynos5/gpu-test && sudo /bin/sh -c "$PWD/push.sh GpuTest.efi startup.nsh bootshell.flag"
    sudo /bin/sh -c "python3 ~/exynos5/uefi/Exynos5420Pkg/Tools/usbdbg.py reboot normal"

Expected in the log: `GpuTest: before:` (bootloader's clock state),
`vdd_g3d now 887 mV`, `GPU_ID 0x0620xxxx`, shaders 0x3f (MP6), then
`NULL job: done`, `WRITE_VALUE job: done`, `result word 0xc0ffee42`.
If the WRITE_VALUE job faults or the word stays 0x11111111, the job
layout (type 2, value type 6 = immediate 32) is the first suspect;
MMU faults point at the page tables / MEMATTR.

## Road to acceleration

1. **Phase 0 — the GPU runs jobs** (this). NULL and WRITE_VALUE jobs
   through our page tables, in UEFI and in Windows with the job interrupt.
2. **Phase 1 — first pixels from the GPU.** A FRAGMENT job with a
   framebuffer descriptor that only clears (no tiler work) into a buffer,
   then a tiler + fragment pass with a fixed blit shader (precompiled
   Midgard binary). Gives a GPU fill / copy path that exynosdisp2 could use
   for presents before anything WDDM-render exists.
3. **Phase 2 — WDDM render.** RT 8.1's DWM composites with D3D11 at
   feature level 9_x or higher. A full WDDM 1.2 driver (render + display
   in one adapter: exynosdisp2's display side + this GPU side): allocations
   (GPU page tables per process context), SubmitCommand / job chains,
   fences from the job interrupt, paging via CPU copies (unified memory),
   preemption off. Kernel side is ours to write.
4. **Phase 3 — user-mode driver.** A D3D UMD DLL: for feature level 9_x
   the D3D9 UMD DDI (the runtime maps D3D11 FL 9_x onto it). Shader
   compilation is the heavy part: Mesa already has a Midgard compiler
   (NIR → Midgard) and Gallium Nine (D3D9 shader bytecode → NIR); the
   plan is to build the Panfrost Gallium driver + Nine's translation for
   ARM32 Windows and put a D3D9-DDI front end on them.

## WDDM registration on RT 8.1 (from dxgkrnl.sys + its PDB)

`DxgkInternalDeviceIoctl` (sent to `\Device\DxgKrnl`, as exynosdisp2's
`DodRegister` does) returns the registration entry for each driver model:

| IOCTL | Entry | Driver model |
|---|---|---|
| `0x230043` | `DpiKmdDodInitialize` | display-only (exynosdisp2 today) |
| `0x230047` | `DpiInitializeWin8` | full WDDM, Win8 DDI (`DRIVER_INITIALIZATION_DATA`) |
| `0x23003F` | (third code in the same pool, presumably `DpiInitialize`) | older full-WDDM entry |

So the render driver registers without displib exactly like exynosdisp2,
with `0x230047` and a full `DRIVER_INITIALIZATION_DATA`.

## Risks / open questions

- **User-mode code integrity on RT: checked 2026-11-03** (`tools/umcitest`).
  With test signing on, a test-signed desktop EXE runs and loads a
  test-signed DLL; an unsigned EXE is blocked. A test-signed UMD can
  therefore load into DWM. Still unchecked: Store-app (AppContainer)
  processes, which may require a higher signing level for loaded DLLs.
- T628 in mainline is "partial": Panfrost's Midgard support targets
  T720+/T760 best; T62x quirks (HW_ISSUE_10649 START_MMU, 11035, 10327)
  are applied, others may surface with real 3D work.
- No GPU DVFS / thermal yet: the driver runs one OPP.
- vdd_g3d is written by two places in the long run (UEFI test, exynosgpu);
  nothing else touches BUCK4.
