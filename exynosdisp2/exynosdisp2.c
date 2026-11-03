/** @file
  v1awifi internal display as a WDDM 1.2 display-only (KMDOD) miniport for
  Windows RT 8.1: Samsung LSL122DL01 2560x1600 eDP panel on FIMD1 + DP1,
  LP8556 backlight.

  The panel is brought up from cold by the shared display cores (UEFI
  Drivers/Display: panel power cycle, DP link training, FIMD1, LP8556 /
  PWM0; cores.c) at start, on monitor D0 and on adapter D0; monitor D3
  turns it off.  Scanout is the firmware's framebuffer (POST display
  ownership).  Presents are CPU copies into it, rotated as the path asks
  (the panel is landscape; Windows' portrait modes are 1600x2560 sources),
  so rotation is supported (DriverRotation, SupportSmoothRotation).
  Brightness goes through DXGK_BRIGHTNESS_INTERFACE (PWM0 duty).

  v2 (exynosdisp2, on top of exynosdisp 1.0.0.51): the driver's own two
  scanout buffers, flipped through FIMD1's shadowed window 0 address
  (latched at vsync), so a present never writes into the buffer on screen
  (each one gets the current and the previous present's rectangles); the
  FIMD1 frame interrupt (combiner group 18 input 5, GSI 50, from the DSDT)
  completes flips and reports display-only vsync to Windows; window 1 is a
  64x64 ARGB hardware cursor.  The firmware framebuffer is only used at
  start (its picture is copied in) and when the display is handed back
  (front buffer copied back, panel left on).  Without the buffers it runs
  as v1.

  The monitor is described by a synthesized EDID (the panel's DP AUX EDID
  is not used, as in the v1awifi kernel).

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ntddk.h>
#include <initguid.h>
#include <dispmprt.h>
#include "ExynosEnv.h"
#include "../../uefi/Exynos5420Pkg/Drivers/Display/ExynosDisplay.h"
#include "fimdext.h"

#define WIDTH          EXYNOS_DISP_WIDTH
#define HEIGHT         EXYNOS_DISP_HEIGHT
#define REFRESH_HZ     60
#define PIXEL_RATE     266000000u
#define HTOTAL         2714
#define VTOTAL         1633
#define FALLBACK_FB    0x67000000u       // UEFI's GOP framebuffer
#define DEFAULT_LEVEL  75                // percent
#define FB_BYTES       ((SIZE_T)WIDTH * HEIGHT * 4)
#define MAX_RECTS      32
#define FLIP_WAIT_MS   50

typedef struct {
  PUCHAR                  Va;
  PHYSICAL_ADDRESS        Pa;
} SCANOUT;

typedef struct {
  DEVICE_OBJECT           *Pdo;
  DXGKRNL_INTERFACE       Dxgk;
  EXYNOS_DISPLAY          Disp;
  PUCHAR                  Fb;            // mapped scanout, Pitch * HEIGHT bytes
  UINT32                  Pitch;
  PHYSICAL_ADDRESS        FbPhys;
  BOOLEAN                 PanelOn;
  BOOLEAN                 SourceVisible;
  BOOLEAN                 NeedFull;      // next present copies the whole source (rotation changed)
  BOOLEAN                 LogNextPresent;
  ULONG                   RotatedPresentLogs;
  BOOLEAN                 LastPanelShaped;
  D3DKMDT_VIDPN_PRESENT_PATH_ROTATION  Rotation;
  UCHAR                   Level;         // brightness, percent
  ULONG                   Presents;
  //
  // v2: own scanout buffers, vsync, hardware cursor.
  //
  UINTN                   Combiner;
  SCANOUT                 Buf[2];
  ULONG                   Front;         // buffer on screen (or being flipped to)
  volatile LONG           FlipPending;
  KEVENT                  FlipDone;
  volatile LONG           VsyncToWindows;
  RECT                    Prev[MAX_RECTS];  // previous present's rectangles (the back buffer lacks them)
  ULONG                   PrevCount;
  BOOLEAN                 PrevFull;
  D3DKMDT_VIDPN_PRESENT_PATH_ROTATION  PrevCopy;
  ULONG                   Vsyncs;
  ULONG                   FlipTimeouts;
  SCANOUT                 Cursor;        // 64 x 64 ARGB
  UINT32                  Shape[FIMD_CURSOR_SIZE * FIMD_CURSOR_SIZE];
  INT32                   CursorX;
  INT32                   CursorY;
  BOOLEAN                 CursorVisible;
} ADAPTER;

DRIVER_INITIALIZE  DriverEntry;

static ULONG  mDdiLogs;

#define DDI_LOG(...)  do { if (mDdiLogs < 200) { mDdiLogs++; OsLog (__VA_ARGS__); } } while (0)

static PVOID
MapPhys (ULONG Address, SIZE_T Size, MEMORY_CACHING_TYPE Type)
{
  PHYSICAL_ADDRESS  Phys;

  Phys.QuadPart = Address;
  return MmMapIoSpace (Phys, Size, Type);
}

static VOID
UnmapRegisters (ADAPTER *A)
{
  if (A->Disp.Fimd   != 0) { MmUnmapIoSpace ((PVOID)A->Disp.Fimd, 0x30000); }
  if (A->Disp.Dp     != 0) { MmUnmapIoSpace ((PVOID)A->Disp.Dp, 0x1000); }
  if (A->Disp.CmuTop != 0) { MmUnmapIoSpace ((PVOID)A->Disp.CmuTop, 0x1000); }
  if (A->Disp.Pmu    != 0) { MmUnmapIoSpace ((PVOID)A->Disp.Pmu, 0x5000); }
  if (A->Disp.Sysreg != 0) { MmUnmapIoSpace ((PVOID)A->Disp.Sysreg, 0x1000); }
  if (A->Disp.GpioLb != 0) { MmUnmapIoSpace ((PVOID)A->Disp.GpioLb, 0x1000); }
  if (A->Disp.GpioRt != 0) { MmUnmapIoSpace ((PVOID)A->Disp.GpioRt, 0x1000); }
  if (A->Disp.GpioB  != 0) { MmUnmapIoSpace ((PVOID)A->Disp.GpioB, 0x1000); }
  if (A->Disp.Pwm    != 0) { MmUnmapIoSpace ((PVOID)A->Disp.Pwm, 0x100); }
  RtlZeroMemory (&A->Disp, FIELD_OFFSET (EXYNOS_DISPLAY, FbPhys));
}

static NTSTATUS
MapRegisters (ADAPTER *A)
{
  A->Disp.Fimd   = (UINTN)MapPhys (EXYNOS_DISP_FIMD1_PHYS, 0x30000, MmNonCached);
  A->Disp.Dp     = (UINTN)MapPhys (EXYNOS_DISP_DP1_PHYS, 0x1000, MmNonCached);
  A->Disp.CmuTop = (UINTN)MapPhys (EXYNOS_DISP_CMU_TOP_PHYS, 0x1000, MmNonCached);
  A->Disp.Pmu    = (UINTN)MapPhys (EXYNOS_DISP_PMU_PHYS, 0x5000, MmNonCached);
  A->Disp.Sysreg = (UINTN)MapPhys (EXYNOS_DISP_SYSREG_PHYS, 0x1000, MmNonCached);
  A->Disp.GpioLb = (UINTN)MapPhys (EXYNOS_DISP_GPIO_LB_PHYS, 0x1000, MmNonCached);
  A->Disp.GpioRt = (UINTN)MapPhys (EXYNOS_DISP_GPIO_RT_PHYS, 0x1000, MmNonCached);
  A->Disp.GpioB  = (UINTN)MapPhys (EXYNOS_DISP_GPIO_B_PHYS, 0x1000, MmNonCached);
  A->Disp.Pwm    = (UINTN)MapPhys (EXYNOS_DISP_PWM_PHYS, 0x100, MmNonCached);
  gExynosGpioBBase = A->Disp.GpioB;     // GpioI2c.c (LP8556 bus)
  if ((A->Disp.Fimd == 0) || (A->Disp.Dp == 0) || (A->Disp.CmuTop == 0) || (A->Disp.Pmu == 0) ||
      (A->Disp.Sysreg == 0) || (A->Disp.GpioLb == 0) || (A->Disp.GpioRt == 0) || (A->Disp.GpioB == 0) ||
      (A->Disp.Pwm == 0)) {
    UnmapRegisters (A);
    return STATUS_INSUFFICIENT_RESOURCES;
  }
  return STATUS_SUCCESS;
}

static BOOLEAN
AllocScanout (SCANOUT *S, SIZE_T Bytes)
{
  PHYSICAL_ADDRESS  Low;
  PHYSICAL_ADDRESS  High;
  PHYSICAL_ADDRESS  Boundary;

  Low.QuadPart      = 0;
  High.QuadPart     = 0xFFFFFFFF;
  Boundary.QuadPart = 0;
  S->Va = MmAllocateContiguousMemorySpecifyCache (Bytes, Low, High, Boundary, MmWriteCombined);
  if (S->Va == NULL) {
    return FALSE;
  }
  S->Pa = MmGetPhysicalAddress (S->Va);
  RtlZeroMemory (S->Va, Bytes);
  return TRUE;
}

static VOID
FreeScanout (SCANOUT *S, SIZE_T Bytes)
{
  if (S->Va != NULL) {
    MmFreeContiguousMemorySpecifyCache (S->Va, Bytes, MmWriteCombined);
    S->Va = NULL;
  }
}

static BOOLEAN
Rotated (D3DKMDT_VIDPN_PRESENT_PATH_ROTATION Rotation);

//
// Source (Windows) to panel coordinates for the path's rotation.
//
static VOID
ToPanel (D3DKMDT_VIDPN_PRESENT_PATH_ROTATION Rotation, LONG X, LONG Y, LONG *Tx, LONG *Ty)
{
  switch (Rotation) {
    case D3DKMDT_VPPR_ROTATE90:
      *Tx = WIDTH - 1 - Y;
      *Ty = X;
      break;
    case D3DKMDT_VPPR_ROTATE180:
      *Tx = WIDTH - 1 - X;
      *Ty = HEIGHT - 1 - Y;
      break;
    case D3DKMDT_VPPR_ROTATE270:
      *Tx = Y;
      *Ty = HEIGHT - 1 - X;
      break;
    default:
      *Tx = X;
      *Ty = Y;
      break;
  }
}

/**
  Cursor image (rotated like the desktop) and position into window 1.
**/
static VOID
CursorUpdate (ADAPTER *A, BOOLEAN NewShape)
{
  LONG    X;
  LONG    Y;
  LONG    Tx;
  LONG    Ty;
  LONG    MinX = WIDTH;
  LONG    MinY = HEIGHT;
  UINT32  *Dst = (UINT32 *)A->Cursor.Va;

  if ((Dst == NULL) || !A->PanelOn) {
    return;
  }
  if (NewShape) {
    for (Y = 0; Y < FIMD_CURSOR_SIZE; Y++) {
      for (X = 0; X < FIMD_CURSOR_SIZE; X++) {
        LONG  Cx;
        LONG  Cy;

        switch (A->Rotation) {
          case D3DKMDT_VPPR_ROTATE90:  Cx = FIMD_CURSOR_SIZE - 1 - Y; Cy = X; break;
          case D3DKMDT_VPPR_ROTATE180: Cx = FIMD_CURSOR_SIZE - 1 - X; Cy = FIMD_CURSOR_SIZE - 1 - Y; break;
          case D3DKMDT_VPPR_ROTATE270: Cx = Y; Cy = FIMD_CURSOR_SIZE - 1 - X; break;
          default:                     Cx = X; Cy = Y; break;
        }
        Dst[Cy * FIMD_CURSOR_SIZE + Cx] = A->Shape[Y * FIMD_CURSOR_SIZE + X];
      }
    }
  }
  //
  // The 64 x 64 box at (CursorX, CursorY) in source coordinates: its
  // top left corner on the panel is the minimum of the mapped corners.
  //
  for (Y = 0; Y <= 1; Y++) {
    for (X = 0; X <= 1; X++) {
      ToPanel (A->Rotation, A->CursorX + X * (FIMD_CURSOR_SIZE - 1), A->CursorY + Y * (FIMD_CURSOR_SIZE - 1), &Tx, &Ty);
      MinX = min (MinX, Tx);
      MinY = min (MinY, Ty);
    }
  }
  FimdCursorMove (&A->Disp, MinX, MinY, A->CursorVisible);
}

static UCHAR
PwmLevel (UCHAR Percent)
{
  return (Percent == 0) ? 0 : (UCHAR)(5 + (Percent * 250) / 100);
}

static VOID
PanelOn (ADAPTER *A)
{
  INTN  Result;

  if (A->PanelOn) {
    return;
  }
  Result = ExynosDisplayEnable (&A->Disp, PwmLevel (A->Level));
  OsLog ("exynosdisp2: panel on - %d (tries %u, step %u, link 0x%02x x %u)", (INT32)Result, A->Disp.Tries,
         A->Disp.Step, A->Disp.LinkRate, A->Disp.LaneCount);
  A->PanelOn = (Result == 0);
  if (A->PanelOn) {
    FimdSetScanout (&A->Disp, A->Buf[A->Front].Pa.LowPart);
    if (A->Cursor.Va != NULL) {
      FimdCursorSetup (&A->Disp, A->Cursor.Pa.LowPart);
      CursorUpdate (A, FALSE);
    }
    if (A->VsyncToWindows || A->FlipPending) {
      FimdVsyncIrq (&A->Disp, A->Combiner, TRUE);
    }
  }
}

static VOID
PanelOff (ADAPTER *A)
{
  if (A->PanelOn) {
    FimdVsyncIrq (&A->Disp, A->Combiner, FALSE);
    if (InterlockedExchange (&A->FlipPending, 0) != 0) {
      KeSetEvent (&A->FlipDone, IO_NO_INCREMENT, FALSE);
    }
    ExynosDisplayDisable (&A->Disp);
    A->PanelOn = FALSE;
    OsLog ("exynosdisp2: panel off");
  }
}

/**
  Hand the display back (stop / release): vsync interrupt and cursor off,
  the picture on screen copied into the firmware framebuffer and scanned
  out from there (the panel stays on: Basic Display cannot bring it up),
  own buffers freed, everything unmapped.
**/
static VOID
GiveBack (ADAPTER *A)
{
  ULONG  Y;

  if (A->Disp.Fimd != 0) {
    if (A->PanelOn) {
      FimdVsyncIrq (&A->Disp, A->Combiner, FALSE);
      FimdCursorMove (&A->Disp, 0, 0, FALSE);
    }
    if ((A->Fb != NULL) && (A->Buf[A->Front].Va != NULL) && (A->Buf[A->Front].Va != A->Fb)) {
      for (Y = 0; Y < HEIGHT; Y++) {
        RtlCopyMemory (A->Fb + (SIZE_T)Y * A->Pitch, A->Buf[A->Front].Va + (SIZE_T)Y * WIDTH * 4, WIDTH * 4);
      }
      A->Disp.FbPhys = A->FbPhys.LowPart;
      A->Disp.Stride = A->Pitch;
      if (A->PanelOn) {
        FimdFlip (&A->Disp, A->FbPhys.LowPart);
        EnvSleepMs (20);                 // let the next vsync latch it before the buffers go
      }
    }
  }
  if (InterlockedExchange (&A->FlipPending, 0) != 0) {
    KeSetEvent (&A->FlipDone, IO_NO_INCREMENT, FALSE);
  }
  UnmapRegisters (A);
  if (A->Buf[0].Va != A->Fb) {
    FreeScanout (&A->Buf[0], FB_BYTES);
    FreeScanout (&A->Buf[1], FB_BYTES);
  }
  RtlZeroMemory (A->Buf, sizeof (A->Buf));
  A->Front = 0;
  FreeScanout (&A->Cursor, FIMD_CURSOR_SIZE * FIMD_CURSOR_SIZE * 4);
  if (A->Combiner != 0) {
    MmUnmapIoSpace ((PVOID)A->Combiner, 0x1000);
    A->Combiner = 0;
  }
  if (A->Fb != NULL) {
    MmUnmapIoSpace (A->Fb, (SIZE_T)A->Pitch * HEIGHT);
    A->Fb = NULL;
  }
  A->PanelOn = FALSE;
}

//
// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------
//

static NTSTATUS APIENTRY
ExAddDevice (IN_CONST_PDEVICE_OBJECT PhysicalDeviceObject, OUT_PPVOID MiniportDeviceContext)
{
  ADAPTER  *A;

  A = ExAllocatePoolWithTag (NonPagedPool, sizeof (ADAPTER), 'psiD');
  if (A == NULL) {
    return STATUS_NO_MEMORY;
  }
  RtlZeroMemory (A, sizeof (*A));
  A->Pdo      = PhysicalDeviceObject;
  A->Level    = DEFAULT_LEVEL;
  A->Rotation = D3DKMDT_VPPR_IDENTITY;
  A->PrevCopy = D3DKMDT_VPPR_IDENTITY;
  KeInitializeEvent (&A->FlipDone, NotificationEvent, TRUE);
  *MiniportDeviceContext = A;
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExStartDeviceImpl (IN_CONST_PVOID MiniportDeviceContext, IN_PDXGK_START_INFO DxgkStartInfo,
               IN_PDXGKRNL_INTERFACE DxgkInterface, OUT_PULONG NumberOfVideoPresentSources,
               OUT_PULONG NumberOfChildren)
{
  ADAPTER                   *A = (ADAPTER *)MiniportDeviceContext;
  DXGK_DISPLAY_INFORMATION  Info;
  NTSTATUS                  Status;

  UNREFERENCED_PARAMETER (DxgkStartInfo);

  RtlCopyMemory (&A->Dxgk, DxgkInterface, sizeof (A->Dxgk));
  RtlZeroMemory (&Info, sizeof (Info));
  Status = A->Dxgk.DxgkCbAcquirePostDisplayOwnership (A->Dxgk.DeviceHandle, &Info);
  if (!NT_SUCCESS (Status) || (Info.Width == 0)) {
    OsLog ("exynosdisp2: no POST display (0x%08lx), using 0x%08lx", Status, FALLBACK_FB);
    Info.PhysicAddress.QuadPart = FALLBACK_FB;
    Info.Pitch                  = WIDTH * 4;
  }
  OsLog ("exynosdisp2: POST display %ux%u pitch %u format %u at 0x%08lx", Info.Width, Info.Height, Info.Pitch,
         Info.ColorFormat, Info.PhysicAddress.LowPart);
  A->FbPhys = Info.PhysicAddress;
  A->Pitch  = (Info.Pitch >= WIDTH * 4) ? Info.Pitch : WIDTH * 4;
  A->Fb     = MmMapIoSpace (A->FbPhys, (SIZE_T)A->Pitch * HEIGHT, MmWriteCombined);
  if (A->Fb == NULL) {
    return STATUS_INSUFFICIENT_RESOURCES;
  }
  Status = MapRegisters (A);
  if (!NT_SUCCESS (Status)) {
    MmUnmapIoSpace (A->Fb, (SIZE_T)A->Pitch * HEIGHT);
    A->Fb = NULL;
    return Status;
  }
  //
  // Own scanout buffers (the firmware's picture copied into both) and the
  // cursor; without them the v1 path (scanout = firmware framebuffer).
  //
  A->Combiner = (UINTN)MapPhys (EXYNOS_COMBINER_PHYS, 0x1000, MmNonCached);
  if ((A->Combiner != 0) && AllocScanout (&A->Buf[0], FB_BYTES) && AllocScanout (&A->Buf[1], FB_BYTES)) {
    ULONG  Y;

    for (Y = 0; Y < HEIGHT; Y++) {
      RtlCopyMemory (A->Buf[0].Va + (SIZE_T)Y * WIDTH * 4, A->Fb + (SIZE_T)Y * A->Pitch, WIDTH * 4);
    }
    RtlCopyMemory (A->Buf[1].Va, A->Buf[0].Va, FB_BYTES);
    AllocScanout (&A->Cursor, FIMD_CURSOR_SIZE * FIMD_CURSOR_SIZE * 4);
    A->Disp.FbPhys = A->Buf[0].Pa.LowPart;
    A->Disp.Stride = WIDTH * 4;
    OsLog ("exynosdisp2: own scanout buffers at 0x%08lx / 0x%08lx, cursor %s", A->Buf[0].Pa.LowPart,
           A->Buf[1].Pa.LowPart, (A->Cursor.Va != NULL) ? "on window 1" : "software");
  } else {
    FreeScanout (&A->Buf[0], FB_BYTES);
    FreeScanout (&A->Buf[1], FB_BYTES);
    A->Buf[0].Va   = A->Fb;
    A->Buf[0].Pa   = A->FbPhys;
    A->Buf[1]      = A->Buf[0];
    A->Disp.FbPhys = A->FbPhys.LowPart;
    A->Disp.Stride = A->Pitch;
    OsLog ("exynosdisp2: no own scanout buffers, drawing into the firmware framebuffer");
  }
  A->Front    = 0;
  A->PrevFull = TRUE;
  A->SourceVisible = TRUE;
  ExynosDisplayLogState (&A->Disp);
  PanelOn (A);

  *NumberOfVideoPresentSources = 1;
  *NumberOfChildren            = 1;
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExStartDevice (IN_CONST_PVOID MiniportDeviceContext, IN_PDXGK_START_INFO DxgkStartInfo,
               IN_PDXGKRNL_INTERFACE DxgkInterface, OUT_PULONG NumberOfVideoPresentSources,
               OUT_PULONG NumberOfChildren)
{
  NTSTATUS  Status;

  Status = ExStartDeviceImpl (MiniportDeviceContext, DxgkStartInfo, DxgkInterface, NumberOfVideoPresentSources, NumberOfChildren);
  DDI_LOG ("exynosdisp2: StartDevice - 0x%08lx", Status);
  return Status;
}

static NTSTATUS APIENTRY
ExStopDevice (IN_CONST_PVOID MiniportDeviceContext)
{
  ADAPTER  *A = (ADAPTER *)MiniportDeviceContext;

  //
  // The panel stays on: whoever takes over next (Basic Display on the
  // firmware framebuffer) cannot bring it up again.
  //
  OsLog ("exynosdisp2: stop device (panel left on, %lu vsyncs, %lu flip timeouts)", A->Vsyncs, A->FlipTimeouts);
  GiveBack (A);
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExRemoveDevice (IN_CONST_PVOID MiniportDeviceContext)
{
  ExFreePoolWithTag ((PVOID)MiniportDeviceContext, 'psiD');
  return STATUS_SUCCESS;
}

/**
  Leave the panel running and hand the framebuffer to the next owner
  (Basic Display Driver).
**/
static NTSTATUS APIENTRY
ExStopDeviceAndReleasePostDisplayOwnership (_In_ PVOID MiniportDeviceContext, _In_ D3DDDI_VIDEO_PRESENT_TARGET_ID TargetId,
                                            _Out_ PDXGK_DISPLAY_INFORMATION DisplayInfo)
{
  ADAPTER  *A = (ADAPTER *)MiniportDeviceContext;

  UNREFERENCED_PARAMETER (TargetId);

  DisplayInfo->Width         = WIDTH;
  DisplayInfo->Height        = HEIGHT;
  DisplayInfo->Pitch         = A->Pitch;
  DisplayInfo->ColorFormat   = D3DDDIFMT_A8R8G8B8;
  DisplayInfo->PhysicAddress = A->FbPhys;
  DisplayInfo->TargetId      = 0;
  DisplayInfo->AcpiId        = 0;
  OsLog ("exynosdisp2: display released (%lu vsyncs, %lu flip timeouts)", A->Vsyncs, A->FlipTimeouts);
  GiveBack (A);
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExDispatchIoRequestImpl (IN_CONST_PVOID MiniportDeviceContext, IN_ULONG VidPnSourceId,
                     IN_PVIDEO_REQUEST_PACKET VideoRequestPacket)
{
  UNREFERENCED_PARAMETER (MiniportDeviceContext);
  UNREFERENCED_PARAMETER (VidPnSourceId);
  UNREFERENCED_PARAMETER (VideoRequestPacket);
  return STATUS_NOT_SUPPORTED;
}

static NTSTATUS APIENTRY
ExDispatchIoRequest (IN_CONST_PVOID MiniportDeviceContext, IN_ULONG VidPnSourceId,
                     IN_PVIDEO_REQUEST_PACKET VideoRequestPacket)
{
  NTSTATUS  Status;

  Status = ExDispatchIoRequestImpl (MiniportDeviceContext, VidPnSourceId, VideoRequestPacket);
  DDI_LOG ("exynosdisp2: DispatchIoRequest - 0x%08lx", Status);
  return Status;
}

/**
  FIMD1 frame interrupt (vsync): the flipped-to buffer is on screen now.
**/
static BOOLEAN APIENTRY
ExInterruptRoutine (IN_CONST_PVOID MiniportDeviceContext, IN_ULONG MessageNumber)
{
  ADAPTER                          *A = (ADAPTER *)MiniportDeviceContext;
  DXGKARGCB_NOTIFY_INTERRUPT_DATA  Notify;

  UNREFERENCED_PARAMETER (MessageNumber);

  if ((A->Disp.Fimd == 0) || !FimdVsyncAck (&A->Disp)) {
    return FALSE;
  }
  A->Vsyncs++;
  if (A->VsyncToWindows) {
    RtlZeroMemory (&Notify, sizeof (Notify));
    Notify.InterruptType                  = DXGK_INTERRUPT_DISPLAYONLY_VSYNC;
    Notify.DisplayOnlyVsync.VidPnTargetId = 0;
    A->Dxgk.DxgkCbNotifyInterrupt (A->Dxgk.DeviceHandle, &Notify);
  }
  if (!A->VsyncToWindows && !A->FlipPending) {
    FimdVsyncIrq (&A->Disp, A->Combiner, FALSE);     // nothing waits: stop the frame interrupts
  }
  A->Dxgk.DxgkCbQueueDpc (A->Dxgk.DeviceHandle);
  return TRUE;
}

static VOID APIENTRY
ExDpcRoutine (IN_CONST_PVOID MiniportDeviceContext)
{
  ADAPTER  *A = (ADAPTER *)MiniportDeviceContext;

  if (InterlockedExchange (&A->FlipPending, 0) != 0) {
    KeSetEvent (&A->FlipDone, IO_NO_INCREMENT, FALSE);
  }
  A->Dxgk.DxgkCbNotifyDpc (A->Dxgk.DeviceHandle);
}

static VOID APIENTRY
ExResetDevice (IN_CONST_PVOID MiniportDeviceContext)
{
  UNREFERENCED_PARAMETER (MiniportDeviceContext);
}

static VOID APIENTRY
ExUnload (VOID)
{
}

//
// Adapter D3 / D0 and monitor D3 / D0 switch the panel; everything else
// (D1, D2) keeps it.
//
static NTSTATUS APIENTRY
ExSetPowerState (IN_CONST_PVOID MiniportDeviceContext, IN_ULONG DeviceUid, IN_DEVICE_POWER_STATE DevicePowerState,
                 IN_POWER_ACTION ActionType)
{
  ADAPTER  *A = (ADAPTER *)MiniportDeviceContext;

  UNREFERENCED_PARAMETER (ActionType);

  OsLog ("exynosdisp2: power %s D%u", (DeviceUid == DISPLAY_ADAPTER_HW_ID) ? "adapter" : "monitor",
         (ULONG)DevicePowerState - 1);
  if (A->Disp.Fimd == 0) {
    return STATUS_SUCCESS;
  }
  if (DeviceUid != DISPLAY_ADAPTER_HW_ID) {
    //
    // Monitor off / on: the panel completely off (backlight, DP, FIMD,
    // LCD_EN) and brought up again from cold.
    //
    if (DevicePowerState == PowerDeviceD0) {
      PanelOn (A);
    } else if (DevicePowerState == PowerDeviceD3) {
      PanelOff (A);
    }
    return STATUS_SUCCESS;
  }
  if (DevicePowerState == PowerDeviceD0) {
    PanelOn (A);
  } else if (DevicePowerState == PowerDeviceD3) {
    PanelOff (A);
  }
  return STATUS_SUCCESS;
}

//
// ---------------------------------------------------------------------------
// Monitor: one embedded DisplayPort panel, always connected.
// ---------------------------------------------------------------------------
//

static NTSTATUS APIENTRY
ExQueryChildRelationsImpl (IN_CONST_PVOID MiniportDeviceContext, PDXGK_CHILD_DESCRIPTOR ChildRelations,
                       _In_ ULONG ChildRelationsSize)
{
  UNREFERENCED_PARAMETER (MiniportDeviceContext);

  if (ChildRelationsSize < 2 * sizeof (DXGK_CHILD_DESCRIPTOR)) {
    return STATUS_BUFFER_TOO_SMALL;
  }
  ChildRelations[0].ChildDeviceType = TypeVideoOutput;
  //
  // As Microsoft's display-only sample (Basic Display) describes an
  // internal panel: "internal", hot-plug interruptible (connection status
  // through QueryChildStatus).  Embedded DisplayPort / always connected
  // made RT 8.1's dxgkrnl stop the device right after this call (it wants
  // an ACPI display output device for those).
  //
  ChildRelations[0].ChildCapabilities.Type.VideoOutput.InterfaceTechnology = D3DKMDT_VOT_INTERNAL;
  ChildRelations[0].ChildCapabilities.Type.VideoOutput.MonitorOrientationAwareness = D3DKMDT_MOA_NONE;
  ChildRelations[0].ChildCapabilities.Type.VideoOutput.SupportsSdtvModes = FALSE;
  ChildRelations[0].ChildCapabilities.HpdAwareness = HpdAwarenessInterruptible;
  ChildRelations[0].AcpiUid  = 0;
  ChildRelations[0].ChildUid = 0;
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExQueryChildRelations (IN_CONST_PVOID MiniportDeviceContext, PDXGK_CHILD_DESCRIPTOR ChildRelations,
                       _In_ ULONG ChildRelationsSize)
{
  NTSTATUS  Status;

  Status = ExQueryChildRelationsImpl (MiniportDeviceContext, ChildRelations, ChildRelationsSize);
  DDI_LOG ("exynosdisp2: QueryChildRelations - 0x%08lx", Status);
  return Status;
}

static NTSTATUS APIENTRY
ExQueryChildStatusImpl (IN_CONST_PVOID MiniportDeviceContext, INOUT_PDXGK_CHILD_STATUS ChildStatus,
                    IN_BOOLEAN NonDestructiveOnly)
{
  UNREFERENCED_PARAMETER (MiniportDeviceContext);
  UNREFERENCED_PARAMETER (NonDestructiveOnly);

  switch (ChildStatus->Type) {
    case StatusConnection:
      ChildStatus->HotPlug.Connected = TRUE;
      return STATUS_SUCCESS;
    case StatusRotation:
      ChildStatus->Rotation.Angle = 0;
      return STATUS_SUCCESS;
    default:
      return STATUS_NOT_SUPPORTED;
  }
}

static NTSTATUS APIENTRY
ExQueryChildStatus (IN_CONST_PVOID MiniportDeviceContext, INOUT_PDXGK_CHILD_STATUS ChildStatus,
                    IN_BOOLEAN NonDestructiveOnly)
{
  NTSTATUS  Status;

  Status = ExQueryChildStatusImpl (MiniportDeviceContext, ChildStatus, NonDestructiveOnly);
  DDI_LOG ("exynosdisp2: QueryChildStatus type %lu - 0x%08lx", (ULONG)ChildStatus->Type, Status);
  return Status;
}

//
// EDID 1.4 for the LSL122DL01: "SEC", digital 8 bpc DisplayPort, 26.2 x
// 16.4 cm, one detailed timing (2560x1600, 266 MHz, HFP 62 / HS 22 /
// HBP 70, VFP 6 / VS 6 / VBP 21), the name; checksum filled in at run time.
//
static UCHAR mEdid[128] = {
  0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,     // header
  0x4C, 0xA3, 0x22, 0x12, 0x00, 0x00, 0x00, 0x00,     // "SEC", product 0x1222, serial 0
  0x01, 0x18, 0x01, 0x04,                             // week 1 2014, EDID 1.4
  0xA5, 0x1A, 0x10, 0x78, 0x0A,                       // digital 8 bpc DP; 26 x 16 cm; gamma 2.2; RGB, preferred timing
  0xEE, 0x91, 0xA3, 0x54, 0x4C, 0x99, 0x26, 0x0F, 0x50, 0x54,   // sRGB chromaticity
  0x00, 0x00, 0x00,                                   // no established timings
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,     // no standard timings
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  // detailed timing: 266.00 MHz; 2560 + 154; 1600 + 33; offsets / widths; 262 x 164 mm
  0xE8, 0x67, 0x00, 0x9A, 0xA0, 0x40, 0x21, 0x60,
  0x3E, 0x16, 0x66, 0x00, 0x06, 0xA4, 0x10, 0x00, 0x00, 0x1E,
  // monitor name
  0x00, 0x00, 0x00, 0xFC, 0x00, 'L', 'S', 'L', '1', '2', '2', 'D', 'L', '0', '1', 0x0A, 0x20, 0x20,
  // dummy descriptors
  0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00,                                               // no extensions
  0x00                                                // checksum
};

static NTSTATUS APIENTRY
ExQueryDeviceDescriptorImpl (IN_CONST_PVOID MiniportDeviceContext, IN_ULONG ChildUid,
                         INOUT_PDXGK_DEVICE_DESCRIPTOR DeviceDescriptor)
{
  ULONG  Index;
  UCHAR  Sum;

  UNREFERENCED_PARAMETER (MiniportDeviceContext);

  if (ChildUid != 0) {
    return STATUS_INVALID_PARAMETER;
  }
  if (DeviceDescriptor->DescriptorOffset >= sizeof (mEdid)) {
    return STATUS_MONITOR_NO_MORE_DESCRIPTOR_DATA;
  }
  for (Sum = 0, Index = 0; Index < 127; Index++) {
    Sum = (UCHAR)(Sum + mEdid[Index]);
  }
  mEdid[127] = (UCHAR)(0x100 - Sum);
  RtlCopyMemory (DeviceDescriptor->DescriptorBuffer, mEdid + DeviceDescriptor->DescriptorOffset,
                 min (DeviceDescriptor->DescriptorLength, sizeof (mEdid) - DeviceDescriptor->DescriptorOffset));
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExQueryDeviceDescriptor (IN_CONST_PVOID MiniportDeviceContext, IN_ULONG ChildUid,
                         INOUT_PDXGK_DEVICE_DESCRIPTOR DeviceDescriptor)
{
  NTSTATUS  Status;

  Status = ExQueryDeviceDescriptorImpl (MiniportDeviceContext, ChildUid, DeviceDescriptor);
  DDI_LOG ("exynosdisp2: QueryDeviceDescriptor - 0x%08lx", Status);
  return Status;
}

//
// ---------------------------------------------------------------------------
// Brightness (DXGK_BRIGHTNESS_INTERFACE): 0..100 in steps of 5.
// ---------------------------------------------------------------------------
//

static VOID
BrRef (PVOID Context)
{
  UNREFERENCED_PARAMETER (Context);
}

static NTSTATUS
BrGetPossible (PVOID Context, ULONG BufferSize, PUCHAR LevelCount, PUCHAR Levels)
{
  ULONG  Index;

  UNREFERENCED_PARAMETER (Context);

  if (BufferSize < 21) {
    return STATUS_BUFFER_TOO_SMALL;
  }
  for (Index = 0; Index <= 20; Index++) {
    Levels[Index] = (UCHAR)(Index * 5);
  }
  *LevelCount = 21;
  return STATUS_SUCCESS;
}

static NTSTATUS
BrSet (PVOID Context, UCHAR Brightness)
{
  ADAPTER  *A = (ADAPTER *)Context;

  A->Level = min (Brightness, (UCHAR)100);
  if (A->PanelOn) {
    ExynosDisplaySetBrightness (&A->Disp, PwmLevel (A->Level));
  }
  return STATUS_SUCCESS;
}

static NTSTATUS
BrGet (PVOID Context, PUCHAR Brightness)
{
  *Brightness = ((ADAPTER *)Context)->Level;
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExQueryInterfaceImpl (IN_CONST_PVOID MiniportDeviceContext, IN_PQUERY_INTERFACE QueryInterface)
{
  DXGK_BRIGHTNESS_INTERFACE  *Br;

  if (IsEqualGUID (QueryInterface->InterfaceType, &GUID_DEVINTERFACE_BRIGHTNESS) &&
      (QueryInterface->Size >= sizeof (DXGK_BRIGHTNESS_INTERFACE))) {
    Br = (DXGK_BRIGHTNESS_INTERFACE *)QueryInterface->Interface;
    Br->Size                  = sizeof (DXGK_BRIGHTNESS_INTERFACE);
    Br->Version               = DXGK_BRIGHTNESS_INTERFACE_VERSION_1;
    Br->Context               = (PVOID)MiniportDeviceContext;
    Br->InterfaceReference    = BrRef;
    Br->InterfaceDereference  = BrRef;
    Br->GetPossibleBrightness = BrGetPossible;
    Br->SetBrightness         = BrSet;
    Br->GetBrightness         = BrGet;
    OsLog ("exynosdisp2: brightness interface handed out");
    return STATUS_SUCCESS;
  }
  return STATUS_NOT_SUPPORTED;
}

static NTSTATUS APIENTRY
ExQueryInterface (IN_CONST_PVOID MiniportDeviceContext, IN_PQUERY_INTERFACE QueryInterface)
{
  NTSTATUS  Status;

  Status = ExQueryInterfaceImpl (MiniportDeviceContext, QueryInterface);
  DDI_LOG ("exynosdisp2: QueryInterface - 0x%08lx", Status);
  return Status;
}

//
// ---------------------------------------------------------------------------
// Adapter info, pointer
// ---------------------------------------------------------------------------
//

static NTSTATUS APIENTRY
ExQueryAdapterInfoImpl (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_QUERYADAPTERINFO pQueryAdapterInfo)
{
  DXGK_DRIVERCAPS  *Caps;

  if (pQueryAdapterInfo->Type != DXGKQAITYPE_DRIVERCAPS) {
    return STATUS_NOT_SUPPORTED;
  }
  if (pQueryAdapterInfo->OutputDataSize < sizeof (DXGK_DRIVERCAPS)) {
    return STATUS_BUFFER_TOO_SMALL;
  }
  Caps = (DXGK_DRIVERCAPS *)pQueryAdapterInfo->pOutputData;
  RtlZeroMemory (Caps, sizeof (*Caps));
  Caps->HighestAcceptableAddress.QuadPart = -1;
  //
  // WDDM 1.2 (interface 0x300E): RT 8.1's dxgkrnl stops a WDDM 1.3 adapter
  // with an internal panel unless it is the POST device or Basic Display
  // (DpiFdoEnumChildDevices); a 1.2 driver is not checked.
  //
  if (((ADAPTER *)hAdapter)->Cursor.Va != NULL) {
    Caps->MaxPointerWidth         = FIMD_CURSOR_SIZE;
    Caps->MaxPointerHeight        = FIMD_CURSOR_SIZE;
    Caps->PointerCaps.Monochrome  = 1;
    Caps->PointerCaps.Color       = 1;
    Caps->PointerCaps.MaskedColor = 1;
  }
  Caps->WDDMVersion                       = DXGKDDI_WDDMv1_2;
  Caps->SupportNonVGA                     = TRUE;
  Caps->SupportSmoothRotation             = TRUE;
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExQueryAdapterInfo (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_QUERYADAPTERINFO pQueryAdapterInfo)
{
  NTSTATUS  Status;

  Status = ExQueryAdapterInfoImpl (hAdapter, pQueryAdapterInfo);
  DDI_LOG ("exynosdisp2: QueryAdapterInfo type %lu - 0x%08lx", (ULONG)pQueryAdapterInfo->Type, Status);
  return Status;
}

static NTSTATUS APIENTRY
ExSetPointerPosition (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_SETPOINTERPOSITION pSetPointerPosition)
{
  ADAPTER  *A = (ADAPTER *)hAdapter;

  A->CursorX       = pSetPointerPosition->X;
  A->CursorY       = pSetPointerPosition->Y;
  A->CursorVisible = pSetPointerPosition->Flags.Visible ? TRUE : FALSE;
  CursorUpdate (A, FALSE);
  return STATUS_SUCCESS;
}

/**
  Pointer shapes into 64 x 64 ARGB: monochrome (AND mask then XOR mask,
  1 bpp; screen-inverting pixels become black), colour (ARGB) and masked
  colour (XRGB plus the AND mask after it).
**/
static NTSTATUS APIENTRY
ExSetPointerShapeImpl (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_SETPOINTERSHAPE pSetPointerShape)
{
  ADAPTER        *A = (ADAPTER *)hAdapter;
  CONST UCHAR    *Pixels = pSetPointerShape->pPixels;
  UINT           Width   = pSetPointerShape->Width;
  UINT           Height  = pSetPointerShape->Height;
  UINT           Pitch   = pSetPointerShape->Pitch;
  UINT           X;
  UINT           Y;

  if (A->Cursor.Va == NULL) {
    return STATUS_NOT_SUPPORTED;
  }
  if (pSetPointerShape->Flags.Monochrome) {
    Height /= 2;                        // AND mask, then XOR mask
  }
  if ((Width > FIMD_CURSOR_SIZE) || (Height > FIMD_CURSOR_SIZE)) {
    return STATUS_INVALID_PARAMETER;
  }
  RtlZeroMemory (A->Shape, sizeof (A->Shape));
  for (Y = 0; Y < Height; Y++) {
    for (X = 0; X < Width; X++) {
      UINT32  Pixel;

      if (pSetPointerShape->Flags.Monochrome) {
        UINT  And = (Pixels[Y * Pitch + X / 8] >> (7 - (X % 8))) & 1;
        UINT  Xor = (Pixels[(Y + Height) * Pitch + X / 8] >> (7 - (X % 8))) & 1;

        Pixel = And ? (Xor ? 0xFF000000u : 0) : (Xor ? 0xFFFFFFFFu : 0xFF000000u);
      } else if (pSetPointerShape->Flags.Color) {
        Pixel = ((CONST UINT32 *)(Pixels + Y * Pitch))[X];
      } else {
        // masked colour: XRGB, the AND mask (1 bpp, pitch (Width + 7) / 8) after the colour rows
        CONST UCHAR  *Mask = Pixels + Height * Pitch;
        UINT         And   = (Mask[Y * ((Width + 7) / 8) + X / 8] >> (7 - (X % 8))) & 1;
        UINT32       Rgb   = ((CONST UINT32 *)(Pixels + Y * Pitch))[X] & 0x00FFFFFFu;

        Pixel = (!And || (Rgb != 0)) ? (0xFF000000u | Rgb) : 0;
      }
      A->Shape[Y * FIMD_CURSOR_SIZE + X] = Pixel;
    }
  }
  CursorUpdate (A, TRUE);
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExSetPointerShape (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_SETPOINTERSHAPE pSetPointerShape)
{
  NTSTATUS  Status;

  Status = ExSetPointerShapeImpl (hAdapter, pSetPointerShape);
  DDI_LOG ("exynosdisp2: SetPointerShape %ux%u flags 0x%x - 0x%08lx", pSetPointerShape->Width, pSetPointerShape->Height,
           pSetPointerShape->Flags.Value, Status);
  return Status;
}

//
// ---------------------------------------------------------------------------
// VidPn: source 0 -> target 0, 2560x1600 (or 1600x2560 rotated) A8R8G8B8.
// ---------------------------------------------------------------------------
//

static BOOLEAN
Rotated (D3DKMDT_VIDPN_PRESENT_PATH_ROTATION Rotation)
{
  return (Rotation == D3DKMDT_VPPR_ROTATE90) || (Rotation == D3DKMDT_VPPR_ROTATE270);
}

static NTSTATUS
AddSourceMode (D3DKMDT_HVIDPNSOURCEMODESET Set, CONST DXGK_VIDPNSOURCEMODESET_INTERFACE *If, BOOLEAN Portrait)
{
  D3DKMDT_VIDPN_SOURCE_MODE  *Mode;
  NTSTATUS                   Status;

  Status = If->pfnCreateNewModeInfo (Set, &Mode);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Mode->Type = D3DKMDT_RMT_GRAPHICS;
  Mode->Format.Graphics.PrimSurfSize.cx      = Portrait ? HEIGHT : WIDTH;
  Mode->Format.Graphics.PrimSurfSize.cy      = Portrait ? WIDTH : HEIGHT;
  Mode->Format.Graphics.VisibleRegionSize    = Mode->Format.Graphics.PrimSurfSize;
  Mode->Format.Graphics.Stride               = Mode->Format.Graphics.PrimSurfSize.cx * 4;
  Mode->Format.Graphics.PixelFormat          = D3DDDIFMT_A8R8G8B8;
  Mode->Format.Graphics.ColorBasis           = D3DKMDT_CB_SCRGB;
  Mode->Format.Graphics.PixelValueAccessMode = D3DKMDT_PVAM_DIRECT;
  Status = If->pfnAddMode (Set, Mode);
  if (!NT_SUCCESS (Status)) {
    If->pfnReleaseModeInfo (Set, Mode);
  }
  return Status;
}

static VOID
FillSignal (D3DKMDT_VIDEO_SIGNAL_INFO *Signal)
{
  Signal->VideoStandard         = D3DKMDT_VSS_OTHER;
  Signal->TotalSize.cx          = HTOTAL;
  Signal->TotalSize.cy          = VTOTAL;
  Signal->ActiveSize.cx         = WIDTH;
  Signal->ActiveSize.cy         = HEIGHT;
  Signal->VSyncFreq.Numerator   = PIXEL_RATE;
  Signal->VSyncFreq.Denominator = HTOTAL * VTOTAL;
  Signal->HSyncFreq.Numerator   = PIXEL_RATE;
  Signal->HSyncFreq.Denominator = HTOTAL;
  Signal->PixelRate             = PIXEL_RATE;
  Signal->ScanLineOrdering      = D3DDDI_VSSLO_PROGRESSIVE;
}

static NTSTATUS
AddTargetMode (D3DKMDT_HVIDPNTARGETMODESET Set, CONST DXGK_VIDPNTARGETMODESET_INTERFACE *If)
{
  D3DKMDT_VIDPN_TARGET_MODE  *Mode;
  NTSTATUS                   Status;

  Status = If->pfnCreateNewModeInfo (Set, &Mode);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  FillSignal (&Mode->VideoSignalInfo);
  Mode->Preference = D3DKMDT_MP_PREFERRED;
  Status = If->pfnAddMode (Set, Mode);
  if (!NT_SUCCESS (Status)) {
    If->pfnReleaseModeInfo (Set, Mode);
  }
  return Status;
}

static NTSTATUS APIENTRY
ExIsSupportedVidPnImpl (IN_CONST_HANDLE hAdapter, INOUT_PDXGKARG_ISSUPPORTEDVIDPN pIsSupportedVidPn)
{
  ADAPTER                               *A = (ADAPTER *)hAdapter;
  CONST DXGK_VIDPN_INTERFACE            *VidPn;
  D3DKMDT_HVIDPNTOPOLOGY                Topology;
  CONST DXGK_VIDPNTOPOLOGY_INTERFACE    *TopologyIf;
  SIZE_T                                Paths;
  NTSTATUS                              Status;

  pIsSupportedVidPn->IsVidPnSupported = FALSE;
  if (pIsSupportedVidPn->hDesiredVidPn == 0) {
    pIsSupportedVidPn->IsVidPnSupported = TRUE;
    return STATUS_SUCCESS;
  }
  Status = A->Dxgk.DxgkCbQueryVidPnInterface (pIsSupportedVidPn->hDesiredVidPn, DXGK_VIDPN_INTERFACE_VERSION_V1, &VidPn);
  if (NT_SUCCESS (Status)) {
    Status = VidPn->pfnGetTopology (pIsSupportedVidPn->hDesiredVidPn, &Topology, &TopologyIf);
  }
  if (NT_SUCCESS (Status)) {
    Status = TopologyIf->pfnGetNumPaths (Topology, &Paths);
  }
  if (NT_SUCCESS (Status)) {
    pIsSupportedVidPn->IsVidPnSupported = (Paths <= 1);
  }
  return Status;
}

static NTSTATUS APIENTRY
ExIsSupportedVidPn (IN_CONST_HANDLE hAdapter, INOUT_PDXGKARG_ISSUPPORTEDVIDPN pIsSupportedVidPn)
{
  NTSTATUS  Status;

  Status = ExIsSupportedVidPnImpl (hAdapter, pIsSupportedVidPn);
  DDI_LOG ("exynosdisp2: IsSupportedVidPn - 0x%08lx", Status);
  return Status;
}

static NTSTATUS APIENTRY
ExRecommendFunctionalVidPnImpl (IN_CONST_HANDLE hAdapter,
                            IN_CONST_PDXGKARG_RECOMMENDFUNCTIONALVIDPN_CONST pRecommendFunctionalVidPn)
{
  UNREFERENCED_PARAMETER (hAdapter);
  UNREFERENCED_PARAMETER (pRecommendFunctionalVidPn);
  return STATUS_GRAPHICS_NO_RECOMMENDED_FUNCTIONAL_VIDPN;
}

static NTSTATUS APIENTRY
ExRecommendFunctionalVidPn (IN_CONST_HANDLE hAdapter,
                            IN_CONST_PDXGKARG_RECOMMENDFUNCTIONALVIDPN_CONST pRecommendFunctionalVidPn)
{
  NTSTATUS  Status;

  Status = ExRecommendFunctionalVidPnImpl (hAdapter, pRecommendFunctionalVidPn);
  DDI_LOG ("exynosdisp2: RecommendFunctionalVidPn - 0x%08lx", Status);
  return Status;
}

static NTSTATUS APIENTRY
ExRecommendMonitorModesImpl (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_RECOMMENDMONITORMODES_CONST pRecommendMonitorModes)
{
  D3DKMDT_MONITOR_SOURCE_MODE  *Mode;
  NTSTATUS                     Status;

  UNREFERENCED_PARAMETER (hAdapter);

  Status = pRecommendMonitorModes->pMonitorSourceModeSetInterface->pfnCreateNewModeInfo (
             pRecommendMonitorModes->hMonitorSourceModeSet, &Mode);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  FillSignal (&Mode->VideoSignalInfo);
  Mode->ColorBasis                              = D3DKMDT_CB_SRGB;
  Mode->ColorCoeffDynamicRanges.FirstChannel    = 8;
  Mode->ColorCoeffDynamicRanges.SecondChannel   = 8;
  Mode->ColorCoeffDynamicRanges.ThirdChannel    = 8;
  Mode->ColorCoeffDynamicRanges.FourthChannel   = 0;
  Mode->Origin                                  = D3DKMDT_MCO_DRIVER;
  Mode->Preference                              = D3DKMDT_MP_PREFERRED;
  Status = pRecommendMonitorModes->pMonitorSourceModeSetInterface->pfnAddMode (
             pRecommendMonitorModes->hMonitorSourceModeSet, Mode);
  if (!NT_SUCCESS (Status)) {
    pRecommendMonitorModes->pMonitorSourceModeSetInterface->pfnReleaseModeInfo (
      pRecommendMonitorModes->hMonitorSourceModeSet, Mode);
    if (Status == STATUS_GRAPHICS_MODE_ALREADY_IN_MODESET) {
      Status = STATUS_SUCCESS;
    }
  }
  return Status;
}

static NTSTATUS APIENTRY
ExRecommendMonitorModes (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_RECOMMENDMONITORMODES_CONST pRecommendMonitorModes)
{
  NTSTATUS  Status;

  Status = ExRecommendMonitorModesImpl (hAdapter, pRecommendMonitorModes);
  DDI_LOG ("exynosdisp2: RecommendMonitorModes - 0x%08lx", Status);
  return Status;
}

/**
  For the (single) path: offer the source modes (landscape, or portrait
  when the path is rotated by 90 / 270) and the target mode where they are
  not pinned, and every rotation.
**/
static NTSTATUS APIENTRY
ExEnumVidPnCofuncModalityImpl (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_ENUMVIDPNCOFUNCMODALITY_CONST pEnumCofuncModality)
{
  ADAPTER                                *A = (ADAPTER *)hAdapter;
  D3DKMDT_HVIDPN                         VidPnHandle = pEnumCofuncModality->hConstrainingVidPn;
  CONST DXGK_VIDPN_INTERFACE             *VidPn;
  D3DKMDT_HVIDPNTOPOLOGY                 Topology;
  CONST DXGK_VIDPNTOPOLOGY_INTERFACE     *TopologyIf;
  CONST D3DKMDT_VIDPN_PRESENT_PATH       *Path;
  D3DKMDT_VIDPN_PRESENT_PATH             Update;
  D3DKMDT_HVIDPNSOURCEMODESET            SourceSet;
  CONST DXGK_VIDPNSOURCEMODESET_INTERFACE *SourceIf;
  CONST D3DKMDT_VIDPN_SOURCE_MODE        *SourcePinned;
  D3DKMDT_HVIDPNTARGETMODESET            TargetSet;
  CONST DXGK_VIDPNTARGETMODESET_INTERFACE *TargetIf;
  CONST D3DKMDT_VIDPN_TARGET_MODE        *TargetPinned;
  BOOLEAN                                Portrait;
  NTSTATUS                               Status;

  Status = A->Dxgk.DxgkCbQueryVidPnInterface (VidPnHandle, DXGK_VIDPN_INTERFACE_VERSION_V1, &VidPn);
  if (NT_SUCCESS (Status)) {
    Status = VidPn->pfnGetTopology (VidPnHandle, &Topology, &TopologyIf);
  }
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Status = TopologyIf->pfnAcquireFirstPathInfo (Topology, &Path);
  if (Status == STATUS_GRAPHICS_DATASET_IS_EMPTY) {
    return STATUS_SUCCESS;
  }
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  //
  // Rotation / scaling support (only what is not pinned yet).
  //
  Update   = *Path;
  Portrait = Rotated (Path->ContentTransformation.Rotation);
  if (Path->ContentTransformation.Rotation == D3DKMDT_VPPR_UNPINNED) {
    Update.ContentTransformation.RotationSupport.Identity  = 1;
    Update.ContentTransformation.RotationSupport.Rotate90  = 1;
    Update.ContentTransformation.RotationSupport.Rotate180 = 1;
    Update.ContentTransformation.RotationSupport.Rotate270 = 1;
  }
  if (Path->ContentTransformation.Scaling == D3DKMDT_VPPS_UNPINNED) {
    RtlZeroMemory (&Update.ContentTransformation.ScalingSupport, sizeof (Update.ContentTransformation.ScalingSupport));
    Update.ContentTransformation.ScalingSupport.Identity = 1;
  }
  Status = TopologyIf->pfnUpdatePathSupportInfo (Topology, &Update);

  //
  // Source modes.
  //
  if (NT_SUCCESS (Status)) {
    Status = VidPn->pfnAcquireSourceModeSet (VidPnHandle, Path->VidPnSourceId, &SourceSet, &SourceIf);
  }
  if (NT_SUCCESS (Status)) {
    SourcePinned = NULL;
    SourceIf->pfnAcquirePinnedModeInfo (SourceSet, &SourcePinned);
    if (SourcePinned != NULL) {
      SourceIf->pfnReleaseModeInfo (SourceSet, SourcePinned);
    }
    VidPn->pfnReleaseSourceModeSet (VidPnHandle, SourceSet);
    if (SourcePinned == NULL) {
      Status = VidPn->pfnCreateNewSourceModeSet (VidPnHandle, Path->VidPnSourceId, &SourceSet, &SourceIf);
      if (NT_SUCCESS (Status)) {
        //
        // Portrait sources only through a pinned 90 / 270 rotation: offered
        // while the rotation is open, Windows picked the 1600x2560 source
        // with an identity path (no rotation for the driver to do).
        //
        Status = AddSourceMode (SourceSet, SourceIf, Portrait);
        if (NT_SUCCESS (Status)) {
          Status = VidPn->pfnAssignSourceModeSet (VidPnHandle, Path->VidPnSourceId, SourceSet);
        } else {
          VidPn->pfnReleaseSourceModeSet (VidPnHandle, SourceSet);
        }
      }
    }
  }

  //
  // Target mode.
  //
  if (NT_SUCCESS (Status)) {
    Status = VidPn->pfnAcquireTargetModeSet (VidPnHandle, Path->VidPnTargetId, &TargetSet, &TargetIf);
  }
  if (NT_SUCCESS (Status)) {
    TargetPinned = NULL;
    TargetIf->pfnAcquirePinnedModeInfo (TargetSet, &TargetPinned);
    if (TargetPinned != NULL) {
      TargetIf->pfnReleaseModeInfo (TargetSet, TargetPinned);
    }
    VidPn->pfnReleaseTargetModeSet (VidPnHandle, TargetSet);
    if (TargetPinned == NULL) {
      Status = VidPn->pfnCreateNewTargetModeSet (VidPnHandle, Path->VidPnTargetId, &TargetSet, &TargetIf);
      if (NT_SUCCESS (Status)) {
        Status = AddTargetMode (TargetSet, TargetIf);
        if (NT_SUCCESS (Status)) {
          Status = VidPn->pfnAssignTargetModeSet (VidPnHandle, Path->VidPnTargetId, TargetSet);
        } else {
          VidPn->pfnReleaseTargetModeSet (VidPnHandle, TargetSet);
        }
      }
    }
  }

  TopologyIf->pfnReleasePathInfo (Topology, Path);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosdisp2: EnumVidPnCofuncModality failed: 0x%08lx", Status);
  }
  return Status;
}

static NTSTATUS APIENTRY
ExEnumVidPnCofuncModality (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_ENUMVIDPNCOFUNCMODALITY_CONST pEnumCofuncModality)
{
  NTSTATUS  Status;

  Status = ExEnumVidPnCofuncModalityImpl (hAdapter, pEnumCofuncModality);
  DDI_LOG ("exynosdisp2: EnumVidPnCofuncModality - 0x%08lx", Status);
  return Status;
}

static NTSTATUS APIENTRY
ExSetVidPnSourceVisibilityImpl (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_SETVIDPNSOURCEVISIBILITY pSetVidPnSourceVisibility)
{
  ADAPTER  *A = (ADAPTER *)hAdapter;

  A->SourceVisible = pSetVidPnSourceVisibility->Visible;
  if (!A->SourceVisible && (A->Fb != NULL)) {
    RtlZeroMemory (A->Fb, (SIZE_T)A->Pitch * HEIGHT);
  }
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExSetVidPnSourceVisibility (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_SETVIDPNSOURCEVISIBILITY pSetVidPnSourceVisibility)
{
  NTSTATUS  Status;

  Status = ExSetVidPnSourceVisibilityImpl (hAdapter, pSetVidPnSourceVisibility);
  DDI_LOG ("exynosdisp2: SetVidPnSourceVisibility - 0x%08lx", Status);
  return Status;
}

static NTSTATUS APIENTRY
ExCommitVidPnImpl (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_COMMITVIDPN_CONST pCommitVidPn)
{
  ADAPTER                               *A = (ADAPTER *)hAdapter;
  CONST DXGK_VIDPN_INTERFACE            *VidPn;
  D3DKMDT_HVIDPNTOPOLOGY                Topology;
  CONST DXGK_VIDPNTOPOLOGY_INTERFACE    *TopologyIf;
  CONST D3DKMDT_VIDPN_PRESENT_PATH      *Path;
  NTSTATUS                              Status;

  Status = A->Dxgk.DxgkCbQueryVidPnInterface (pCommitVidPn->hFunctionalVidPn, DXGK_VIDPN_INTERFACE_VERSION_V1, &VidPn);
  if (NT_SUCCESS (Status)) {
    Status = VidPn->pfnGetTopology (pCommitVidPn->hFunctionalVidPn, &Topology, &TopologyIf);
  }
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Status = TopologyIf->pfnAcquireFirstPathInfo (Topology, &Path);
  if (Status == STATUS_GRAPHICS_DATASET_IS_EMPTY) {
    A->SourceVisible = FALSE;
    return STATUS_SUCCESS;
  }
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  {
    D3DKMDT_VIDPN_PRESENT_PATH_ROTATION  Rotation = Path->ContentTransformation.Rotation;

    if ((Rotation == D3DKMDT_VPPR_UNINITIALIZED) || (Rotation == D3DKMDT_VPPR_UNPINNED)) {
      Rotation = D3DKMDT_VPPR_IDENTITY;
    }
    if (Rotation != A->Rotation) {
      A->Rotation = Rotation;
      A->NeedFull = TRUE;
    }
  }
  //
  // Diagnostics: the source and target modes Windows pinned.
  //
  {
    D3DKMDT_HVIDPNSOURCEMODESET              SourceSet;
    CONST DXGK_VIDPNSOURCEMODESET_INTERFACE  *SourceIf;
    CONST D3DKMDT_VIDPN_SOURCE_MODE          *Source = NULL;
    D3DKMDT_HVIDPNTARGETMODESET              TargetSet;
    CONST DXGK_VIDPNTARGETMODESET_INTERFACE  *TargetIf;
    CONST D3DKMDT_VIDPN_TARGET_MODE          *Target = NULL;

    if (NT_SUCCESS (VidPn->pfnAcquireSourceModeSet (pCommitVidPn->hFunctionalVidPn, Path->VidPnSourceId, &SourceSet, &SourceIf))) {
      SourceIf->pfnAcquirePinnedModeInfo (SourceSet, &Source);
      if (Source != NULL) {
        OsLog ("exynosdisp2: pinned source %ux%u visible %ux%u stride %u format %u", Source->Format.Graphics.PrimSurfSize.cx,
               Source->Format.Graphics.PrimSurfSize.cy, Source->Format.Graphics.VisibleRegionSize.cx,
               Source->Format.Graphics.VisibleRegionSize.cy, Source->Format.Graphics.Stride, Source->Format.Graphics.PixelFormat);
        SourceIf->pfnReleaseModeInfo (SourceSet, Source);
      }
      VidPn->pfnReleaseSourceModeSet (pCommitVidPn->hFunctionalVidPn, SourceSet);
    }
    if (NT_SUCCESS (VidPn->pfnAcquireTargetModeSet (pCommitVidPn->hFunctionalVidPn, Path->VidPnTargetId, &TargetSet, &TargetIf))) {
      TargetIf->pfnAcquirePinnedModeInfo (TargetSet, &Target);
      if (Target != NULL) {
        OsLog ("exynosdisp2: pinned target %ux%u", Target->VideoSignalInfo.ActiveSize.cx, Target->VideoSignalInfo.ActiveSize.cy);
        TargetIf->pfnReleaseModeInfo (TargetSet, Target);
      }
      VidPn->pfnReleaseTargetModeSet (pCommitVidPn->hFunctionalVidPn, TargetSet);
    }
  }
  TopologyIf->pfnReleasePathInfo (Topology, Path);
  A->LogNextPresent = TRUE;
  OsLog ("exynosdisp2: commit, rotation %u", (ULONG)A->Rotation);
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExCommitVidPn (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_COMMITVIDPN_CONST pCommitVidPn)
{
  NTSTATUS  Status;

  Status = ExCommitVidPnImpl (hAdapter, pCommitVidPn);
  DDI_LOG ("exynosdisp2: CommitVidPn - 0x%08lx", Status);
  return Status;
}

static NTSTATUS APIENTRY
ExUpdateActiveVidPnPresentPathImpl (IN_CONST_HANDLE hAdapter,
                                IN_CONST_PDXGKARG_UPDATEACTIVEVIDPNPRESENTPATH_CONST pUpdateActiveVidPnPresentPath)
{
  ADAPTER  *A = (ADAPTER *)hAdapter;

  if (A->Rotation != pUpdateActiveVidPnPresentPath->VidPnPresentPathInfo.ContentTransformation.Rotation) {
    A->Rotation = pUpdateActiveVidPnPresentPath->VidPnPresentPathInfo.ContentTransformation.Rotation;
    A->NeedFull = TRUE;
  }
  A->LogNextPresent = TRUE;
  OsLog ("exynosdisp2: path update, rotation %u", (ULONG)A->Rotation);
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExUpdateActiveVidPnPresentPath (IN_CONST_HANDLE hAdapter,
                                IN_CONST_PDXGKARG_UPDATEACTIVEVIDPNPRESENTPATH_CONST pUpdateActiveVidPnPresentPath)
{
  NTSTATUS  Status;

  Status = ExUpdateActiveVidPnPresentPathImpl (hAdapter, pUpdateActiveVidPnPresentPath);
  DDI_LOG ("exynosdisp2: UpdateActiveVidPnPresentPath - 0x%08lx", Status);
  return Status;
}

static NTSTATUS APIENTRY
ExQueryVidPnHWCapabilityImpl (IN_CONST_HANDLE i_hAdapter, INOUT_PDXGKARG_QUERYVIDPNHWCAPABILITY io_pVidPnHWCaps)
{
  UNREFERENCED_PARAMETER (i_hAdapter);

  RtlZeroMemory (&io_pVidPnHWCaps->VidPnHWCaps, sizeof (io_pVidPnHWCaps->VidPnHWCaps));
  io_pVidPnHWCaps->VidPnHWCaps.DriverRotation     = 1;
  io_pVidPnHWCaps->VidPnHWCaps.DriverColorConvert = 1;
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExQueryVidPnHWCapability (IN_CONST_HANDLE i_hAdapter, INOUT_PDXGKARG_QUERYVIDPNHWCAPABILITY io_pVidPnHWCaps)
{
  NTSTATUS  Status;

  Status = ExQueryVidPnHWCapabilityImpl (i_hAdapter, io_pVidPnHWCaps);
  DDI_LOG ("exynosdisp2: QueryVidPnHWCapability - 0x%08lx", Status);
  return Status;
}

//
// ---------------------------------------------------------------------------
// Present: copy the dirty / moved rectangles from the source image into the
// scanout, rotated into the panel's landscape orientation.
// ---------------------------------------------------------------------------
//

static VOID
CopyRect (D3DKMDT_VIDPN_PRESENT_PATH_ROTATION Rotation, PUCHAR Fb, ULONG FbPitch, CONST UCHAR *Source, LONG SourcePitch,
          CONST RECT *R)
{
  LONG    X;
  LONG    Y;
  UINT32  *Dst;
  CONST UINT32 *Src;

  if ((Rotation != D3DKMDT_VPPR_ROTATE90) && (Rotation != D3DKMDT_VPPR_ROTATE180) && (Rotation != D3DKMDT_VPPR_ROTATE270)) {
    for (Y = R->top; Y < R->bottom; Y++) {
      RtlCopyMemory (Fb + (SIZE_T)Y * FbPitch + R->left * 4, Source + (SIZE_T)Y * SourcePitch + R->left * 4,
                     (SIZE_T)(R->right - R->left) * 4);
    }
    return;
  }
  //
  // Rotated: loop over panel lines so the writes into the write-combined
  // scanout are sequential (a pixel-per-line pattern took seconds per frame
  // and tripped the display timeout); the source is cached memory.
  //
  switch (Rotation) {
    case D3DKMDT_VPPR_ROTATE90:          // Tx = W - 1 - Y, Ty = X
      for (X = R->left; X < R->right; X++) {
        Dst = (UINT32 *)(Fb + (SIZE_T)X * FbPitch);
        Src = (CONST UINT32 *)(Source + (SIZE_T)R->top * SourcePitch) + X;
        for (Y = R->top; Y < R->bottom; Y++) {
          Dst[WIDTH - 1 - Y] = *Src;
          Src = (CONST UINT32 *)((CONST UCHAR *)Src + SourcePitch);
        }
      }
      break;
    case D3DKMDT_VPPR_ROTATE270:         // Tx = Y, Ty = H - 1 - X
      for (X = R->left; X < R->right; X++) {
        Dst = (UINT32 *)(Fb + (SIZE_T)(HEIGHT - 1 - X) * FbPitch);
        Src = (CONST UINT32 *)(Source + (SIZE_T)R->top * SourcePitch) + X;
        for (Y = R->top; Y < R->bottom; Y++) {
          Dst[Y] = *Src;
          Src = (CONST UINT32 *)((CONST UCHAR *)Src + SourcePitch);
        }
      }
      break;
    default:                             // D3DKMDT_VPPR_ROTATE180: Tx = W - 1 - X, Ty = H - 1 - Y
      for (Y = R->top; Y < R->bottom; Y++) {
        Dst = (UINT32 *)(Fb + (SIZE_T)(HEIGHT - 1 - Y) * FbPitch);
        Src = (CONST UINT32 *)(Source + (SIZE_T)Y * SourcePitch);
        for (X = R->left; X < R->right; X++) {
          Dst[WIDTH - 1 - X] = Src[X];
        }
      }
      break;
  }
}

/**
  With a rotated path Windows gives the dirty / move rectangles in panel
  (unrotated, 2560x1600) coordinates while the image is the rotated source
  (seen on RT 8.1: pitch 6400 with rectangles up to 2560,1600).  Back to
  source coordinates, the inverse of CopyRect's mapping.
**/
static VOID
PanelRectToSource (D3DKMDT_VIDPN_PRESENT_PATH_ROTATION Rotation, CONST RECT *P, RECT *S)
{
  switch (Rotation) {
    case D3DKMDT_VPPR_ROTATE90:          // Tx = W - 1 - Y, Ty = X
      S->left   = P->top;
      S->right  = P->bottom;
      S->top    = WIDTH - P->right;
      S->bottom = WIDTH - P->left;
      break;
    case D3DKMDT_VPPR_ROTATE180:         // Tx = W - 1 - X, Ty = H - 1 - Y
      S->left   = WIDTH - P->right;
      S->right  = WIDTH - P->left;
      S->top    = HEIGHT - P->bottom;
      S->bottom = HEIGHT - P->top;
      break;
    case D3DKMDT_VPPR_ROTATE270:         // Tx = Y, Ty = H - 1 - X
      S->left   = HEIGHT - P->bottom;
      S->right  = HEIGHT - P->top;
      S->top    = P->left;
      S->bottom = P->right;
      break;
    default:
      *S = *P;
      break;
  }
}

/**
  Under the interrupt's lock (DxgkCbSynchronizeExecution): the ISR switches
  the frame interrupt off when nothing waits, so arming a flip and turning
  it on must not interleave with it (a lost race left the interrupt off and
  every later flip timed out).
**/
typedef struct {
  ADAPTER  *A;
  ULONG    Phys;          // 0: only enable the frame interrupt
} FLIP_SYNC;

static BOOLEAN
FlipSync (PVOID Context)
{
  FLIP_SYNC  *F = (FLIP_SYNC *)Context;

  if (F->Phys != 0) {
    InterlockedExchange (&F->A->FlipPending, 1);
    FimdFlip (&F->A->Disp, F->Phys);      // latched at the next vsync
  }
  FimdVsyncIrq (&F->A->Disp, F->A->Combiner, TRUE);
  return TRUE;
}

static VOID
ArmVsync (ADAPTER *A, ULONG Phys)
{
  FLIP_SYNC  F;
  BOOLEAN    Ret;

  F.A    = A;
  F.Phys = Phys;
  if (!NT_SUCCESS (A->Dxgk.DxgkCbSynchronizeExecution (A->Dxgk.DeviceHandle, FlipSync, &F, 0, &Ret))) {
    FlipSync (&F);
  }
}

static VOID
ClipRect (RECT *R, LONG W, LONG H)
{
  R->left   = max (R->left, 0);
  R->top    = max (R->top, 0);
  R->right  = min (R->right, W);
  R->bottom = min (R->bottom, H);
}

/**
  Draw into the back buffer (this present's rectangles plus the previous
  one's, which the back buffer has not seen yet), then flip to it at the
  next vsync.  A present first waits for the previous flip, so it never
  writes into the buffer being scanned out.

  The image's shape decides the copy, not the path rotation (1.0.0.51):
  with a 90 / 270 path RT 8.1 hands over a 1600x2560 image (pitch 6400)
  only for the first present after the commit, later ones are 2560x1600
  (pitch 10240) already in panel orientation, rectangles up to 2560,1600.
**/
static NTSTATUS APIENTRY
ExPresentDisplayOnly (IN_CONST_HANDLE hAdapter, IN_CONST_PDXGKARG_PRESENT_DISPLAYONLY pPresentDisplayOnly)
{
  ADAPTER        *A = (ADAPTER *)hAdapter;
  CONST UCHAR    *Source = (CONST UCHAR *)pPresentDisplayOnly->pSource;
  LONG           SourcePitch = pPresentDisplayOnly->Pitch;
  ULONG          Index;
  LONG           SourceW;
  LONG           SourceH;
  BOOLEAN        PanelShaped;
  D3DKMDT_VIDPN_PRESENT_PATH_ROTATION  Copy;
  RECT           R;
  RECT           Now[MAX_RECTS];
  ULONG          NowCount = 0;
  BOOLEAN        NowFull;
  BOOLEAN        Flip;
  ULONG          Back;
  ULONG          BackPitch;
  LARGE_INTEGER  Timeout;

  if ((A->Buf[0].Va == NULL) || !A->SourceVisible || (pPresentDisplayOnly->BytesPerPixel != 4)) {
    return STATUS_SUCCESS;
  }

  PanelShaped = (SourcePitch >= WIDTH * 4);
  if (Rotated (A->Rotation) && !PanelShaped) {
    SourceW = HEIGHT;
    SourceH = WIDTH;
    Copy    = A->Rotation;
  } else if (Rotated (A->Rotation)) {
    SourceW = WIDTH;
    SourceH = HEIGHT;
    Copy    = D3DKMDT_VPPR_IDENTITY;
  } else {
    SourceW = WIDTH;
    SourceH = HEIGHT;
    Copy    = A->Rotation;
  }
  if (Rotated (A->Rotation) && ((PanelShaped != A->LastPanelShaped) || (A->RotatedPresentLogs == 0))) {
    A->LastPanelShaped = PanelShaped;
    A->RotatedPresentLogs++;
    OsLog ("exynosdisp2: rotation %u, image pitch %ld: %s", (ULONG)A->Rotation, SourcePitch,
           PanelShaped ? "panel-shaped, copied as is" : "portrait-shaped, rotated");
  }

  Flip = (A->Buf[0].Va != A->Buf[1].Va) && A->PanelOn;
  if (A->Presents++ == 0) {
    OsLog ("exynosdisp2: first present (%lu moves, %lu dirty, pitch %ld, rotation %u, %s)", pPresentDisplayOnly->NumMoves,
           pPresentDisplayOnly->NumDirtyRects, SourcePitch, (ULONG)A->Rotation, Flip ? "double buffered" : "single buffer");
  }
  if (A->LogNextPresent) {
    A->LogNextPresent = FALSE;
    OsLog ("exynosdisp2: present after commit: rotation %u, %ldx%ld source, pitch %ld, %lu moves, %lu dirty",
           (ULONG)A->Rotation, SourceW, SourceH, SourcePitch, pPresentDisplayOnly->NumMoves,
           pPresentDisplayOnly->NumDirtyRects);
  }

  //
  // A change of copy (rotation, image shape) redraws everything, in both
  // buffers (the previous one is marked as fully stale).
  //
  NowFull     = A->NeedFull || (Copy != A->PrevCopy);
  A->NeedFull = FALSE;
  if (Copy != A->PrevCopy) {
    A->PrevCopy = Copy;
    A->PrevFull = TRUE;
  }

  // This present's rectangles (moves: their destinations; the source image is complete).
  for (Index = 0; !NowFull && (Index < pPresentDisplayOnly->NumMoves + pPresentDisplayOnly->NumDirtyRects); Index++) {
    RECT  P = (Index < pPresentDisplayOnly->NumMoves) ? pPresentDisplayOnly->pMoves[Index].DestRect :
                                                         pPresentDisplayOnly->pDirtyRect[Index - pPresentDisplayOnly->NumMoves];

    if (Copy == D3DKMDT_VPPR_ROTATE180) {
      PanelRectToSource (Copy, &P, &R);    // 180: panel coordinates (as in v1)
    } else {
      R = P;
    }
    ClipRect (&R, SourceW, SourceH);
    if ((R.right <= R.left) || (R.bottom <= R.top)) {
      continue;
    }
    if (NowCount < MAX_RECTS) {
      Now[NowCount++] = R;
    } else {
      NowFull = TRUE;
    }
  }
  if ((NowCount == 0) && !NowFull) {
    return STATUS_SUCCESS;              // nothing new: the front buffer is complete
  }

  if (Flip) {
    Timeout.QuadPart = -10000LL * FLIP_WAIT_MS;
    if (KeWaitForSingleObject (&A->FlipDone, Executive, KernelMode, FALSE, &Timeout) == STATUS_TIMEOUT) {
      if (A->FlipTimeouts++ < 10) {
        OsLog ("exynosdisp2: flip did not complete in %u ms (%lu vsyncs so far): VIDINTCON0 0x%08lx VIDINTCON1 0x%08lx "
               "VIDCON1 0x%08lx, combiner 18 enable 0x%08lx status 0x%08lx masked 0x%08lx",
               FLIP_WAIT_MS, A->Vsyncs, MmioRead32 (A->Disp.Fimd + 0x130), MmioRead32 (A->Disp.Fimd + 0x134),
               MmioRead32 (A->Disp.Fimd + 0x20004), MmioRead32 (A->Combiner + 0x40), MmioRead32 (A->Combiner + 0x48),
               MmioRead32 (A->Combiner + 0x4C));
      }
      InterlockedExchange (&A->FlipPending, 0);
    }
    Back      = A->Front ^ 1;
    BackPitch = WIDTH * 4;
  } else {
    Back      = A->Front;
    BackPitch = (A->Buf[Back].Va == A->Fb) ? A->Pitch : WIDTH * 4;
  }

  if (NowFull || (Flip && A->PrevFull)) {
    R.left   = 0;
    R.top    = 0;
    R.right  = SourceW;
    R.bottom = SourceH;
    CopyRect (Copy, A->Buf[Back].Va, BackPitch, Source, SourcePitch, &R);
  } else {
    for (Index = 0; Index < NowCount; Index++) {
      CopyRect (Copy, A->Buf[Back].Va, BackPitch, Source, SourcePitch, &Now[Index]);
    }
    for (Index = 0; Flip && (Index < A->PrevCount); Index++) {
      CopyRect (Copy, A->Buf[Back].Va, BackPitch, Source, SourcePitch, &A->Prev[Index]);
    }
  }

  if (Flip) {
    RtlCopyMemory (A->Prev, Now, NowCount * sizeof (RECT));
    A->PrevCount = NowCount;
    A->PrevFull  = NowFull;
    KeClearEvent (&A->FlipDone);
    ArmVsync (A, A->Buf[Back].Pa.LowPart);
    A->Front = Back;
  } else {
    //
    // Single buffer, or the panel is off (drawn into the front buffer
    // only): the other buffer misses this, redraw it fully next time.
    //
    A->PrevCount = 0;
    A->PrevFull  = (A->Buf[0].Va != A->Buf[1].Va);
  }
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExGetScanLineImpl (IN_CONST_HANDLE hAdapter, INOUT_PDXGKARG_GETSCANLINE pGetScanLine)
{
  UNREFERENCED_PARAMETER (hAdapter);
  UNREFERENCED_PARAMETER (pGetScanLine);
  return STATUS_NOT_SUPPORTED;
}

static NTSTATUS APIENTRY
ExGetScanLine (IN_CONST_HANDLE hAdapter, INOUT_PDXGKARG_GETSCANLINE pGetScanLine)
{
  NTSTATUS  Status;

  Status = ExGetScanLineImpl (hAdapter, pGetScanLine);
  DDI_LOG ("exynosdisp2: GetScanLine - 0x%08lx", Status);
  return Status;
}

static NTSTATUS APIENTRY
ExControlInterruptImpl (IN_CONST_HANDLE hAdapter, IN_CONST_DXGK_INTERRUPT_TYPE InterruptType, IN_BOOLEAN EnableInterrupt)
{
  ADAPTER  *A = (ADAPTER *)hAdapter;

  if (InterruptType != DXGK_INTERRUPT_DISPLAYONLY_VSYNC) {
    return STATUS_NOT_IMPLEMENTED;
  }
  InterlockedExchange (&A->VsyncToWindows, EnableInterrupt ? 1 : 0);
  if (EnableInterrupt && A->PanelOn) {
    ArmVsync (A, 0);
  }
  return STATUS_SUCCESS;
}

static NTSTATUS APIENTRY
ExControlInterrupt (IN_CONST_HANDLE hAdapter, IN_CONST_DXGK_INTERRUPT_TYPE InterruptType, IN_BOOLEAN EnableInterrupt)
{
  NTSTATUS  Status;

  Status = ExControlInterruptImpl (hAdapter, InterruptType, EnableInterrupt);
  DDI_LOG ("exynosdisp2: ControlInterrupt type %u %s - 0x%08lx", (ULONG)InterruptType, EnableInterrupt ? "on" : "off", Status);
  return Status;
}

//
// ---------------------------------------------------------------------------
// Bugcheck screen: the landscape framebuffer as it is.
// ---------------------------------------------------------------------------
//

static NTSTATUS APIENTRY
ExSystemDisplayEnable (_In_ PVOID MiniportDeviceContext, _In_ D3DDDI_VIDEO_PRESENT_TARGET_ID TargetId,
                       _In_ PDXGKARG_SYSTEM_DISPLAY_ENABLE_FLAGS Flags, _Out_ UINT* Width, _Out_ UINT* Height,
                       _Out_ D3DDDIFORMAT* ColorFormat)
{
  ADAPTER  *A = (ADAPTER *)MiniportDeviceContext;

  UNREFERENCED_PARAMETER (TargetId);
  UNREFERENCED_PARAMETER (Flags);

  if (A->Buf[A->Front].Va == NULL) {
    return STATUS_UNSUCCESSFUL;
  }
  if (A->PanelOn) {
    FimdCursorMove (&A->Disp, 0, 0, FALSE);
  }
  A->Rotation  = D3DKMDT_VPPR_IDENTITY;
  *Width       = WIDTH;
  *Height      = HEIGHT;
  *ColorFormat = D3DDDIFMT_A8R8G8B8;
  return STATUS_SUCCESS;
}

static VOID APIENTRY
ExSystemDisplayWrite (_In_ PVOID MiniportDeviceContext, PVOID Source, _In_ UINT SourceWidth, _In_ UINT SourceHeight,
                      _In_ UINT SourceStride, _In_ UINT PositionX, _In_ UINT PositionY)
{
  ADAPTER  *A = (ADAPTER *)MiniportDeviceContext;
  UINT     Y;
  UINT     Width;
  PUCHAR   Fb = A->Buf[A->Front].Va;
  ULONG    Pitch = (Fb == A->Fb) ? A->Pitch : WIDTH * 4;

  if ((Fb == NULL) || (PositionX >= WIDTH) || (PositionY >= HEIGHT)) {
    return;
  }
  Width = min (SourceWidth, WIDTH - PositionX);
  for (Y = 0; (Y < SourceHeight) && (PositionY + Y < HEIGHT); Y++) {
    RtlCopyMemory (Fb + (SIZE_T)(PositionY + Y) * Pitch + PositionX * 4, (PUCHAR)Source + (SIZE_T)Y * SourceStride,
                   (SIZE_T)Width * 4);
  }
}

//
// Registration with dxgkrnl without displib.lib: the WDK 10 (19041)
// displib wraps DxgkDdiStartDevice and touches the registry in ways RT
// 8.1's dxgkrnl does not expect (DriverEntry failed with
// STATUS_OBJECT_PATH_INVALID).  The classic sequence: load DXGKrnl, open
// \Device\DxgKrnl, ask it (internal IOCTL, function 0x10) for its
// display-only initialisation entry, call that.  Each step is logged.
//
#define IOCTL_VIDEO_KMDOD_DDI_REGISTER  0x00230043     // CTL_CODE (FILE_DEVICE_VIDEO, 0x10, METHOD_NEITHER, FILE_ANY_ACCESS)

typedef NTSTATUS (*DXGK_DOD_INITIALIZE)(PDRIVER_OBJECT, PUNICODE_STRING, PKMDDOD_INITIALIZATION_DATA);

static NTSTATUS
DodRegister (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath, PKMDDOD_INITIALIZATION_DATA Init)
{
  UNICODE_STRING       Name;
  PFILE_OBJECT         File;
  PDEVICE_OBJECT       Device;
  KEVENT               Event;
  IO_STATUS_BLOCK      Iosb;
  PIRP                 Irp;
  DXGK_DOD_INITIALIZE  DodInitialize = NULL;
  NTSTATUS             Status;

  RtlInitUnicodeString (&Name, L"\\Registry\\Machine\\System\\CurrentControlSet\\Services\\DXGKrnl");
  Status = ZwLoadDriver (&Name);
  OsLog ("exynosdisp2: ZwLoadDriver (DXGKrnl) - 0x%08lx", Status);
  if (!NT_SUCCESS (Status) && (Status != STATUS_IMAGE_ALREADY_LOADED)) {
    return Status;
  }

  RtlInitUnicodeString (&Name, L"\\Device\\DxgKrnl");
  Status = IoGetDeviceObjectPointer (&Name, FILE_ALL_ACCESS, &File, &Device);
  OsLog ("exynosdisp2: \\Device\\DxgKrnl - 0x%08lx", Status);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  KeInitializeEvent (&Event, NotificationEvent, FALSE);
  Irp = IoBuildDeviceIoControlRequest (IOCTL_VIDEO_KMDOD_DDI_REGISTER, Device, NULL, 0, &DodInitialize,
                                       sizeof (DodInitialize), TRUE, &Event, &Iosb);
  if (Irp == NULL) {
    ObDereferenceObject (File);
    return STATUS_INSUFFICIENT_RESOURCES;
  }
  Status = IoCallDriver (Device, Irp);
  if (Status == STATUS_PENDING) {
    KeWaitForSingleObject (&Event, Executive, KernelMode, FALSE, NULL);
    Status = Iosb.Status;
  }
  ObDereferenceObject (File);
  OsLog ("exynosdisp2: display-only registration IOCTL - 0x%08lx, entry %p", Status, DodInitialize);
  if (!NT_SUCCESS (Status) || (DodInitialize == NULL)) {
    return NT_SUCCESS (Status) ? STATUS_PROCEDURE_NOT_FOUND : Status;
  }

  Status = DodInitialize (DriverObject, RegistryPath, Init);
  OsLog ("exynosdisp2: dxgkrnl display-only initialisation - 0x%08lx", Status);
  return Status;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  KMDDOD_INITIALIZATION_DATA  Init;

  OsLogInit ();
  OsLog ("exynosdisp2: DriverEntry (built " __DATE__ " " __TIME__ ")");

  RtlZeroMemory (&Init, sizeof (Init));
  Init.Version                               = DXGKDDI_INTERFACE_VERSION;
  Init.DxgkDdiAddDevice                      = ExAddDevice;
  Init.DxgkDdiStartDevice                    = ExStartDevice;
  Init.DxgkDdiStopDevice                     = ExStopDevice;
  Init.DxgkDdiRemoveDevice                   = ExRemoveDevice;
  Init.DxgkDdiDispatchIoRequest              = ExDispatchIoRequest;
  Init.DxgkDdiInterruptRoutine               = ExInterruptRoutine;
  Init.DxgkDdiDpcRoutine                     = ExDpcRoutine;
  Init.DxgkDdiQueryChildRelations            = ExQueryChildRelations;
  Init.DxgkDdiQueryChildStatus               = ExQueryChildStatus;
  Init.DxgkDdiQueryDeviceDescriptor          = ExQueryDeviceDescriptor;
  Init.DxgkDdiSetPowerState                  = ExSetPowerState;
  Init.DxgkDdiResetDevice                    = ExResetDevice;
  Init.DxgkDdiUnload                         = ExUnload;
  Init.DxgkDdiQueryInterface                 = ExQueryInterface;
  Init.DxgkDdiQueryAdapterInfo               = ExQueryAdapterInfo;
  Init.DxgkDdiSetPointerPosition             = ExSetPointerPosition;
  Init.DxgkDdiSetPointerShape                = ExSetPointerShape;
  Init.DxgkDdiIsSupportedVidPn               = ExIsSupportedVidPn;
  Init.DxgkDdiRecommendFunctionalVidPn       = ExRecommendFunctionalVidPn;
  Init.DxgkDdiEnumVidPnCofuncModality        = ExEnumVidPnCofuncModality;
  Init.DxgkDdiSetVidPnSourceVisibility       = ExSetVidPnSourceVisibility;
  Init.DxgkDdiCommitVidPn                    = ExCommitVidPn;
  Init.DxgkDdiUpdateActiveVidPnPresentPath   = ExUpdateActiveVidPnPresentPath;
  Init.DxgkDdiRecommendMonitorModes          = ExRecommendMonitorModes;
  Init.DxgkDdiGetScanLine                    = ExGetScanLine;
  Init.DxgkDdiQueryVidPnHWCapability         = ExQueryVidPnHWCapability;
  Init.DxgkDdiPresentDisplayOnly             = ExPresentDisplayOnly;
  Init.DxgkDdiStopDeviceAndReleasePostDisplayOwnership = ExStopDeviceAndReleasePostDisplayOwnership;
  Init.DxgkDdiSystemDisplayEnable            = ExSystemDisplayEnable;
  Init.DxgkDdiSystemDisplayWrite             = ExSystemDisplayWrite;
  Init.DxgkDdiControlInterrupt               = ExControlInterrupt;
  OsLog ("exynosdisp2: service %wZ", RegistryPath);
  return DodRegister (DriverObject, RegistryPath, &Init);
}
