# exynoswddm — display + Mali-T628 as one full WDDM 1.2 adapter

Status (2026-11-03): **DWM desktop works (1.0.0.95, present-only mode).**
Login, desktop, Start screen and charms render through this driver with
Mesa d3d10umd + softpipe (exynosumd.dll) as the UMD; slow (CPU rendering,
seconds per frame). No device error.
- Model: one **aperture** segment; allocations are system pages mapped into
  the Mali MMU at a fixed GPU address each (exynosgpu, common/ExGpuIf.h);
  DMA buffers in system memory; flips copy the primary into two contiguous
  scan-out buffers in the firmware carve-out (needs
  boot-v1awifi-dma-gpu-carveout.img, exynosgpu >= 1.0.0.79).
- Present-only mode (EXW_GPU_MODE 0): GPU_INFO says "no GPU", the UMD
  renders on the CPU. Shared surfaces: an opener (DWM) may not lock them,
  so the creator publishes frames (escape 0x101, at Present and at Flush of
  dirty surfaces) into a per-allocation shadow and DWM fetches them
  (escape 0x102) before draws.
- Fixes that got DWM up (see git history / memory notes):
  MultiEngineAware = 1; DXGKARG_CREATEDEVICE Flags/pInfo union (never write
  pInfo); DescribeAllocation refresh = the VidPN VSyncFreq exactly (else
  SetDisplayMode fails and DXGI SetFullscreenState returns 0x887A0025);
  flips without a DMA buffer succeed (else every present 0xC01E0001).
- Diagnostics: ddtrace.c (first calls + failures of every DDI),
  oslogfile.c (OS log ring copied to C:\malikm\oslog-kmd.txt), bugcheck
  callback naming module+offset.
- Next: GPU mode (EXW_GPU_MODE 1, panfrost on the T628).
- Device: same as exynosdisp2 (`ACPI\EXYN5010`); exynosdisp2 stays the
  fallback.

## Milestone 1: what it does

| Part | File | What |
|---|---|---|
| Display | `exynoswddm.c` | exynosdisp2's display side: cold panel bring-up, VidPN, brightness, FIMD1 window-1 cursor, vsync on GSI 50. Registration through dxgkrnl's full-WDDM entry (internal IOCTL `0x230047`, `DpiInitializeWin8`). Flips go through `DxgkDdiSetVidPnSourceAddress`, which programs FIMD1's shadowed window-0 address; the vsync interrupt reports the latched address (`DXGK_INTERRUPT_CRTC_VSYNC`). A boot buffer with the firmware picture is the scanout until the first flip. Driver rotation is off in milestone 1. |
| Render | `render.c` | **Segment:** one memory segment, 128 MiB (64 MiB fallback) of contiguous write-combined RAM allocated at start. It is CPU-visible and populated from system memory; allocations, DMA buffers and paging buffers all live in it. **Allocations:** `EX_ALLOC_DESC` private data, plus the kernel's standard allocations: shared primary, shadow, staging. **Paging:** transfer (MDL ↔ segment, segment ↔ segment) and fill. **Present:** Blt / ColorFill become `EX_CMD_BLT`; Flip is a NOP. **Render:** validated UMD command buffers. **Patch:** resolves allocation-list references. **Submit:** a worker thread executes each submission on the CPU in order, then reports `DMA_COMPLETED` for its fence. |
| Formats | `ExCmd.h` | DMA command format, allocation descriptor, UMD query / log, shared with the UMD. |
| UMD | `../exynosumd/` | `exynosumd.dll` stub. `OpenAdapter` (D3D9), `OpenAdapter10` and `OpenAdapter10_2` log the caller and the interface version into the kernel log (`exynosumd: ...`), then fail with `E_NOTIMPL`. |

The INF installs both files. The UMD goes to `System32`, and
`UserModeDriverName` points at it.

## What the first boot should tell us

1. dxgkrnl accepts the full registration: the log shows `full WDDM
   registration IOCTL - 0x00000000` and then `StartDevice - 0`.
2. The segment gets allocated, and the standard allocations and the
   shared primary are created.
3. Kernel presents run, followed by a flip to the primary: look for
   `present blt` and `scanout -> 0x...`. The desktop or logon screen
   should appear even without a working UMD, if Windows' fallback renders
   it.
4. **Which runtime opens the UMD**, and whether Windows falls back when
   it fails (WARP, or Basic Render).

## Install / recovery

- **Install:** use Device Manager on the display adapter, *Update driver
  → Have disk → exynoswddm.inf*, then reboot.
- **Back to exynosdisp2:** in Device Manager, *Update driver → Have disk
  → exynosdisp2.inf*.
- **If Windows no longer shows a desktop:** in MSC mode, rename
  `Windows\System32\drivers\exynoswddm.sys`. The device then fails to
  start and Windows uses Basic Display on the firmware framebuffer, the
  same mechanism exynosdisp2 relies on. Reinstall exynosdisp2 from
  there.

## Next milestones

2. **A real UMD.** Implement D3D9 DDI resource creation, Lock / Unlock,
   Blt and Present in `EX_CMD` form, so DWM runs, with composition still
   on the CPU path.
3. **GPU-executed DMA commands.** Add `EX_CMD_GPU_JOB` with the Mali core
   (job interrupt for fences) and per-process Mali page tables, then the
   GPU blit proven in UEFI (`GpuTest` step 6).
4. **Shaders.** Compile D3D9 bytecode to Midgard (Mesa: Nine → NIR →
   Midgard).
