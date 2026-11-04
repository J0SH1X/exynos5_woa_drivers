/** @file
  Exynos5420 camera for Windows RT 8.1: an AVStream capture minidriver for
  the rear IMX134 (EXYN5015 = FIMC-IS), so the Camera app and Media
  Foundation see a camera.

  One filter (KSCATEGORY_VIDEO_CAMERA / CAPTURE / VIDEO) with one capture
  pin: YUY2 640 x 480.  No ISP yet: the sensor's raw Bayer frames come
  through CSIS0 + FIMC-Lite A (the path verified in UEFI and Windows by the
  raw-frame milestone) and are debayered / scaled in software, with a crude
  auto exposure and white balance, into the stream buffers.

  Power: the sensor, the ISP and its A5 firmware come up when the pin
  leaves STOP (a worker thread; the first frame follows ~2 s later) and go
  down again when it returns to STOP.  All hardware code is the shared UEFI
  tree (exynoscam/cores.c).  Replies from the A5 are polled; the DSDT
  interrupt resource is unused for now.

  The earlier KMDF version (one raw frame to C:\malikm\camera.raw) is kept
  as exynoscam-raw-frame-kmdf.c.txt.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#define OSLOG_GLOBAL
#include <ExynosEnv.h>
#include "kscompat.h"
#include <ks.h>
#include <ksmedia.h>
#include "../../uefi/Exynos5420Pkg/Drivers/Camera/FimcIsHost.h"
#include "../../uefi/Exynos5420Pkg/Drivers/Camera/CamBoard.h"
#include "../../uefi/Exynos5420Pkg/Drivers/Camera/CamCapture.h"
#include "../../uefi/Exynos5420Pkg/Drivers/Camera/CamAe.h"
#include <initguid.h>
#include "../common/ExPmicIf.h"

#define CAM_TAG              'mCxE'
#define FW_PATH              L"\\SystemRoot\\System32\\drivers\\fimc_is_fw2_IMX134.bin"
#define SETFILE_PATH         L"\\SystemRoot\\System32\\drivers\\setfile_imx134.bin"
#define SETFILE_FRONT_PATH   L"\\SystemRoot\\System32\\drivers\\setfile_6b2.bin"
#define FILE_MAX             (4 * 1024 * 1024)
#define ALIVE_TIMEOUT_MS     3000
#define PMIC_WAIT_S          60

#define RAW_W                3280
#define RAW_H                2458
#define FRONT_W              1936
#define FRONT_H              1090
#define RAW_BYTES            (RAW_W * RAW_H * 2)          // the larger (rear) frame; also holds the front one
#define BLACK                65                 // RAW10 black level (64) + 1 so the noise floor does not count as signal

#define OUT_W                640
#define OUT_H                480
#define OUT_BYTES            (OUT_W * OUT_H * 2)       // YUY2
#define OUT_FPS              15

#define PMU_SIZE             0x5000
#define CMU_SIZE             0x11000
#define SYSREG_SIZE          0x1000

#define SMC_CMD_REG          ((ULONG)-101)
#define SMC_SFR_W(a)         ((1u << 30) | ((a) >> 2))
#define SMC_SFR_R(a)         ((3u << 30) | ((a) >> 2))

typedef struct {
  PKSDEVICE          Device;
  FIMC_IS            Is;
  CAM_BOARD          Board;
  CAM_CAPTURE        Cap;
  EX_PMIC_INTERFACE  Pmic;
  PUCHAR             Fw;
  ULONG              FwSize;
  PUCHAR             Setfile;
  ULONG              SetfileSize;
  PUCHAR             SetfileFront;
  ULONG              SetfileFrontSize;
  BOOLEAN            Front;              // camera of the open pin: FALSE rear IMX134, TRUE front S5K6B2
  UINTN              CsisBase[2];        // [0] CSIS0 / [1] CSIS1 mappings
  UINTN              FliteBase[2];       // FIMC-Lite A / B
  ULONG              RawW, RawH;         // raw frame size of the camera in use
  ULONG              CropX, CropW;       // 4:3 window of it that is scaled to OUT_W x OUT_H
  PUSHORT            Raw;                // RAW_BYTES, non-cached, DMA target
  PHYSICAL_ADDRESS   RawPa;
  PUCHAR             Work;               // YUY2 being converted
  PUCHAR             Latest;             // last complete YUY2 frame
  KSPIN_LOCK         FrameLock;
  volatile LONG      FrameReady;
  PKSPIN             Pin;
  PKTHREAD           Thread;
  volatile BOOLEAN   Stop;
  BOOLEAN            HwUp;
  LONGLONG           RunStart;
  ULONG              Frames;
  ULONG              Delivered;
  LONGLONG           StreamStart;
  CAM_I2C            Sensor;             // the sensor's own I2C bus, bit-banged for exposure control
  CAM_AE             Ae;
  BOOLEAN            AeOn;
  LONGLONG           FirstFrameTime;
  ULONG              YMean;
  BOOLEAN            Restarted;
  ULONG              Starved;
  LONG               LastMean[3];
  ULONG              RawNonBlack;
  UCHAR              Gamma[1024];
} CAM_DEV, *PCAM_DEV;

//
// A5 memory and the DMA buffer are non-cached.
//
VOID
EnvCacheClean (
  IN VOID   *Va,
  IN UINTN  Size
  )
{
  UNREFERENCED_PARAMETER (Va);
  UNREFERENCED_PARAMETER (Size);
  __builtin_arm_dsb (15);
}

VOID
EnvCacheInvalidate (
  IN VOID   *Va,
  IN UINTN  Size
  )
{
  UNREFERENCED_PARAMETER (Va);
  UNREFERENCED_PARAMETER (Size);
  __builtin_arm_dsb (15);
}

//
// ---------------------------------------------------------------------------
// Secure monitor (exynos_smc / exynos_smc_readsfr): r0 = SMC_CMD_REG,
// r1 = class | address >> 2, r2 = value.  The monitor leaves r0 =
// SMC_CMD_REG and returns the status in r1 (0 = done), a read value in r2.
// ---------------------------------------------------------------------------
//
static ULONG
Smc (ULONG A0, ULONG *A1, ULONG *A2)
{
  register ULONG  R0 __asm__ ("r0") = A0;
  register ULONG  R1 __asm__ ("r1") = *A1;
  register ULONG  R2 __asm__ ("r2") = *A2;
  register ULONG  R3 __asm__ ("r3") = 0;

  __asm__ volatile (
    ".arch_extension sec\n\t"
    "dsb\n\t"
    "smc #0\n\t"
    "dsb\n\t"
    : "+r" (R0), "+r" (R1), "+r" (R2), "+r" (R3)
    :
    : "memory", "r12", "lr"
    );
  *A1 = R1;
  *A2 = R2;
  return (R0 == SMC_CMD_REG) ? R1 : R0;
}

static INTN
SecureWrite (UINT32 PhysAddr, UINT32 Value)
{
  ULONG  R1 = SMC_SFR_W (PhysAddr);
  ULONG  R2 = Value;

  return (Smc (SMC_CMD_REG, &R1, &R2) == 0) ? 0 : -1;
}

static INTN
SecureRead (UINT32 PhysAddr, UINT32 *Value)
{
  ULONG  R1 = SMC_SFR_R (PhysAddr);
  ULONG  R2 = 0;

  if (Smc (SMC_CMD_REG, &R1, &R2) != 0) {
    return -1;
  }
  *Value = R2;
  return 0;
}

//
// PMIC through exynospmic (CamBoard callbacks).
//
static INTN
PmicRead (VOID *Context, UINT8 Reg, UINT8 *Value)
{
  PCAM_DEV  D = (PCAM_DEV)Context;

  return NT_SUCCESS (D->Pmic.Read (D->Pmic.Header.Context, Reg, Value)) ? 0 : -1;
}

static INTN
PmicWrite (VOID *Context, UINT8 Reg, UINT8 Value)
{
  PCAM_DEV  D = (PCAM_DEV)Context;

  return NT_SUCCESS (D->Pmic.Write (D->Pmic.Header.Context, Reg, Value)) ? 0 : -1;
}

//
// ---------------------------------------------------------------------------
// Files, memory
// ---------------------------------------------------------------------------
//

static NTSTATUS
LoadFile (PCWSTR Path, PUCHAR *Data, ULONG *Size)
{
  UNICODE_STRING             Name;
  OBJECT_ATTRIBUTES          Attr;
  IO_STATUS_BLOCK            Io;
  FILE_STANDARD_INFORMATION  Info;
  HANDLE                     File;
  NTSTATUS                   Status;

  *Data = NULL;
  RtlInitUnicodeString (&Name, Path);
  InitializeObjectAttributes (&Attr, &Name, OBJ_CASE_INSENSITIVE | OBJ_KERNEL_HANDLE, NULL, NULL);
  Status = ZwCreateFile (&File, GENERIC_READ | SYNCHRONIZE, &Attr, &Io, NULL, FILE_ATTRIBUTE_NORMAL, FILE_SHARE_READ,
                         FILE_OPEN, FILE_SYNCHRONOUS_IO_NONALERT | FILE_NON_DIRECTORY_FILE, NULL, 0);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Status = ZwQueryInformationFile (File, &Io, &Info, sizeof (Info), FileStandardInformation);
  if (NT_SUCCESS (Status) && ((Info.EndOfFile.HighPart != 0) || (Info.EndOfFile.LowPart > FILE_MAX))) {
    Status = STATUS_FILE_TOO_LARGE;
  }
  if (NT_SUCCESS (Status)) {
    *Size = Info.EndOfFile.LowPart;
    *Data = ExAllocatePoolWithTag (NonPagedPool, *Size, CAM_TAG);
    Status = (*Data != NULL) ? ZwReadFile (File, NULL, NULL, NULL, &Io, *Data, *Size, NULL, NULL)
                             : STATUS_INSUFFICIENT_RESOURCES;
  }
  ZwClose (File);
  if (!NT_SUCCESS (Status) && (*Data != NULL)) {
    ExFreePoolWithTag (*Data, CAM_TAG);
    *Data = NULL;
  }
  return Status;
}

/**
  20 MiB at a 64 MiB boundary: ask for exactly the window [k * 64 MiB,
  k * 64 MiB + 20 MiB) for each k below 4 GiB, highest first.
**/
static PVOID
AllocA5Memory (PHYSICAL_ADDRESS *Pa)
{
  PHYSICAL_ADDRESS  Low;
  PHYSICAL_ADDRESS  High;
  PHYSICAL_ADDRESS  Boundary;
  PVOID             Va;
  LONG              K;

  Boundary.QuadPart = 0;
  for (K = (LONG)(0x100000000ULL / FIMC_IS_A5_MEM_ALIGN) - 1; K >= 0; K--) {
    Low.QuadPart  = (LONGLONG)K * FIMC_IS_A5_MEM_ALIGN;
    High.QuadPart = Low.QuadPart + FIMC_IS_A5_MEM_SIZE - 1;
    Va = MmAllocateContiguousMemorySpecifyCache (FIMC_IS_A5_MEM_SIZE, Low, High, Boundary, MmNonCached);
    if (Va != NULL) {
      *Pa = MmGetPhysicalAddress (Va);
      if ((Pa->QuadPart & (FIMC_IS_A5_MEM_ALIGN - 1)) == 0) {
        return Va;
      }
      MmFreeContiguousMemorySpecifyCache (Va, FIMC_IS_A5_MEM_SIZE, MmNonCached);
    }
  }
  return NULL;
}

static PVOID
MapPhys (ULONG Address, SIZE_T Size)
{
  PHYSICAL_ADDRESS  Pa;

  Pa.QuadPart = Address;
  return MmMapIoSpace (Pa, Size, MmNonCached);
}

//
// ---------------------------------------------------------------------------
// Raw -> YUY2
// ---------------------------------------------------------------------------
//

static ULONG
ISqrt (ULONG N)
{
  ULONG  Root = 0;
  ULONG  Bit = 1u << 30;

  while (Bit > N) {
    Bit >>= 2;
  }
  while (Bit != 0) {
    if (N >= Root + Bit) {
      N    -= Root + Bit;
      Root  = (Root >> 1) + Bit;
    } else {
      Root >>= 1;
    }
    Bit >>= 2;
  }
  return Root;
}

//
// One RGGB quad at even (X, Y) of the raw frame; levels 0..1023 above black.
//
static __inline VOID
Quad (volatile PUCHAR Raw, ULONG Stride, ULONG X, ULONG Y, LONG *R, LONG *G, LONG *B)
{
  volatile ULONG  *Row0 = (volatile ULONG *)(Raw + ((SIZE_T)Y * Stride + X) * 2);
  volatile ULONG  *Row1 = (volatile ULONG *)(Raw + ((SIZE_T)(Y + 1) * Stride + X) * 2);
  ULONG           W0 = *Row0;
  ULONG           W1 = *Row1;
  LONG            Rr = (LONG)(W0 & 0xFFFF) - BLACK;
  LONG            G1 = (LONG)(W0 >> 16) - BLACK;
  LONG            G2 = (LONG)(W1 & 0xFFFF) - BLACK;
  LONG            Bb = (LONG)(W1 >> 16) - BLACK;

  *R = (Rr < 0) ? 0 : Rr;
  *B = (Bb < 0) ? 0 : Bb;
  G1 = (G1 < 0) ? 0 : G1;
  G2 = (G2 < 0) ? 0 : G2;
  *G = (G1 + G2) >> 1;
}

static __inline ULONG
Clamp8 (LONG V)
{
  return (V < 0) ? 0 : ((V > 255) ? 255 : (ULONG)V);
}

static VOID
Convert (PCAM_DEV D)
{
  volatile PUCHAR  Raw = (volatile PUCHAR)D->Raw;
  PUCHAR           Out = D->Work;
  ULONGLONG        SumR = 0, SumG = 0, SumB = 0;
  ULONG            Count = 0;
  ULONG            X, Y, GX, GY, Lin;
  LONG             R, G, B, MeanR, MeanG, MeanB, KR, KG, KB, Expo, GainR, GainB;

  //
  // Auto exposure / white balance (digital): means over a sparse grid.
  //
  for (GY = 0; GY < 60; GY++) {
    for (GX = 0; GX < 80; GX++) {
      Quad (Raw, D->RawW, (D->CropX + ((GX * 8) * D->CropW) / OUT_W) & ~1u, (((GY * 8) * D->RawH) / OUT_H) & ~1u, &R, &G, &B);
      SumR += (ULONG)R;
      SumG += (ULONG)G;
      SumB += (ULONG)B;
      Count++;
    }
  }
  MeanR = (LONG)(SumR / Count) + 1;
  MeanG = (LONG)(SumG / Count) + 1;
  MeanB = (LONG)(SumB / Count) + 1;
  GainR = (MeanG * 256) / MeanR;
  GainB = (MeanG * 256) / MeanB;
  GainR = (GainR < 64) ? 64 : ((GainR > 1024) ? 1024 : GainR);
  GainB = (GainB < 64) ? 64 : ((GainB > 1024) ? 1024 : GainB);
  Expo  = (184 * 256) / MeanG;                       // mean green -> 18 % grey
  Expo  = (Expo < 256) ? 256 : ((Expo > 256 * 220) ? 256 * 220 : Expo);
  D->LastMean[0] = MeanR;
  D->LastMean[1] = MeanG;
  D->LastMean[2] = MeanB;
  KG    = Expo;
  KR    = (Expo * GainR) >> 8;
  KB    = (Expo * GainB) >> 8;

  for (Y = 0; Y < OUT_H; Y++) {
    ULONG  SrcY = ((Y * D->RawH) / OUT_H) & ~1u;

    for (X = 0; X < OUT_W; X += 2) {
      LONG   R0, G0, B0, R1, G1, B1;
      ULONG  Y0, Y1, U, V, R8[2], G8[2], B8[2];
      ULONG  Index;

      Quad (Raw, D->RawW, (D->CropX + (X * D->CropW) / OUT_W) & ~1u, SrcY, &R0, &G0, &B0);
      Quad (Raw, D->RawW, (D->CropX + ((X + 1) * D->CropW) / OUT_W) & ~1u, SrcY, &R1, &G1, &B1);

      Lin = (ULONG)((R0 * KR) >> 8);  R8[0] = D->Gamma[(Lin > 1023) ? 1023 : Lin];
      Lin = (ULONG)((G0 * KG) >> 8);  G8[0] = D->Gamma[(Lin > 1023) ? 1023 : Lin];
      Lin = (ULONG)((B0 * KB) >> 8);  B8[0] = D->Gamma[(Lin > 1023) ? 1023 : Lin];
      Lin = (ULONG)((R1 * KR) >> 8);  R8[1] = D->Gamma[(Lin > 1023) ? 1023 : Lin];
      Lin = (ULONG)((G1 * KG) >> 8);  G8[1] = D->Gamma[(Lin > 1023) ? 1023 : Lin];
      Lin = (ULONG)((B1 * KB) >> 8);  B8[1] = D->Gamma[(Lin > 1023) ? 1023 : Lin];

      Y0 = Clamp8 ((66 * (LONG)R8[0] + 129 * (LONG)G8[0] + 25 * (LONG)B8[0] + 128) / 256 + 16);
      Y1 = Clamp8 ((66 * (LONG)R8[1] + 129 * (LONG)G8[1] + 25 * (LONG)B8[1] + 128) / 256 + 16);
      {
        LONG  Ra = (LONG)(R8[0] + R8[1]) / 2;
        LONG  Ga = (LONG)(G8[0] + G8[1]) / 2;
        LONG  Ba = (LONG)(B8[0] + B8[1]) / 2;

        U = Clamp8 ((-38 * Ra - 74 * Ga + 112 * Ba + 128) / 256 + 128);
        V = Clamp8 ((112 * Ra - 94 * Ga - 18 * Ba + 128) / 256 + 128);
      }
      Index = (Y * OUT_W + X) * 2;
      Out[Index + 0] = (UCHAR)Y0;
      Out[Index + 1] = (UCHAR)U;
      Out[Index + 2] = (UCHAR)Y1;
      Out[Index + 3] = (UCHAR)V;
    }
  }
}

//
// ---------------------------------------------------------------------------
// Hardware sequence (worker thread, PASSIVE_LEVEL)
// ---------------------------------------------------------------------------
//

static VOID
LogBoard (PCAM_DEV D, PCSTR When)
{
  CAM_BOARD_REGS  R;

  CamBoardSnapshot (&D->Board, &R);
  OsLog ("exynoscam: %s: GPE0 con %08lx dat %02lx GPF0 con %08lx GPF1 con %08lx GPH0 con %08lx SCLK src %08lx div %08lx "
         "gate %08lx LDO18 0x%02x LDO24 0x%02x", When, R.Gpe0Con, R.Gpe0Dat, R.Gpf0Con, R.Gpf1Con, R.Gph0Con,
         R.SclkSrcIsp, R.SclkDivIsp, R.GateTopSclkIsp, R.Ldo18, R.Ldo24);
}

static VOID
LogIsp (PCAM_DEV D, PCSTR When)
{
  FIMC_IS_REGS  R;

  FimcIsSnapshot (&D->Is, &R);
  OsLog ("exynoscam: %s: PD 0x%lx/0x%lx A5 st 0x%lx SRC_TOP1 %08lx TOP3 %08lx TOP4 %08lx DIV_TOP0 %08lx DIV_ISP %08lx %08lx %08lx "
         "GATE_ISP %08lx %08lx GSCL0 %08lx", When, R.PdConfig, R.PdStatus, R.ArmStatus, R.SrcTop1, R.SrcTop3, R.SrcTop4,
         R.DivTop0, R.DivIsp0, R.DivIsp1, R.DivIsp2, R.GateIsp0, R.GateIsp1, R.GateGscl0);
}

static VOID
Delay (ULONG Ms)
{
  LARGE_INTEGER  Interval;

  Interval.QuadPart = -10000LL * Ms;
  KeDelayExecutionThread (KernelMode, FALSE, &Interval);
}

//
// One power-up attempt.  0 = streaming, 1 = the sensor's I2C did not answer
// (retry), < 0 = give up.
//
static INTN
HwTry (PCAM_DEV D)
{
  FIMC_IS_MSG  Msg;
  INTN         Result = -1;

  D->Board.PmicRead    = PmicRead;
  D->Board.PmicWrite   = PmicWrite;
  D->Board.PmicContext = D;
  D->Is.SensorI2cFail  = FALSE;
  LogBoard (D, "module before");
  if ((D->Front ? CamFrontPowerOn (&D->Board) : CamRearPowerOn (&D->Board)) != 0) {
    OsLog ("exynoscam: %s module power failed at step %lu", D->Front ? "front" : "rear", D->Board.Step);
    return -1;
  }
  Delay (30);                                   // rails and MCLK settle before the A5 talks to the sensor
  LogBoard (D, "module on");
  if ((FimcIsPowerUp (&D->Is, TRUE) != 0) || (FimcIsLoadFirmware (&D->Is, D->Fw, D->FwSize) != 0)) {
    OsLog ("exynoscam: ISP power-up / firmware failed at step %lu", D->Is.Step);
    goto Fail;
  }
  D->Is.LogPos = 0;
  LogIsp (D, "isp up");
  if (FimcIsStartA5 (&D->Is) != 0) {
    OsLog ("exynoscam: A5 start failed at step %lu", D->Is.Step);
    goto Fail;
  }
  if ((FimcIsWaitMessage (&D->Is, ALIVE_TIMEOUT_MS, &Msg) != 0) || (Msg.Command != IHC_GET_SENSOR_NUMBER)) {
    OsLog ("exynoscam: A5 did not report in %u ms", ALIVE_TIMEOUT_MS);
    goto Fail;
  }
  if (((D->Front ? FimcIsOpenFrontSensor (&D->Is, FIMC_IS_GROUP_3A0) : FimcIsOpenRearSensor (&D->Is, FIMC_IS_GROUP_3A0)) != 0) ||
      (FimcIsLoadSetfile (&D->Is, FIMC_IS_GROUP_3A0, D->Front ? D->SetfileFront : D->Setfile,
                          D->Front ? D->SetfileFrontSize : D->SetfileSize) != 0)) {
    OsLog ("exynoscam: sensor open failed");
    Result = -2;
    goto Down;
  }
  FimcIsLogDump (&D->Is);
  if (D->Is.SensorI2cFail) {
    LogIsp (D, "i2c fail");
    LogBoard (D, "i2c fail");
    OsLog ("exynoscam: the A5 reports the sensor's I2C does not answer");
    Result = 1;
    goto Down;
  }
  D->Cap.Csis   = D->CsisBase[D->Front ? 1 : 0];
  D->Cap.Flite  = D->FliteBase[D->Front ? 1 : 0];
  D->Cap.FliteB = D->Front;
  D->Cap.Dphy   = D->Front ? 1 : 0;
  D->Cap.Width  = D->RawW;
  D->Cap.Height = D->RawH;
  D->Cap.Settle = D->Front ? 16 : 15;
  D->Cap.Lanes  = 4;
  D->Cap.BufPa  = (UINT32)D->RawPa.QuadPart;
  CamCaptureConfigure (&D->Cap);
  if (FimcIsCommand (&D->Is, "stream on", HIC_STREAM_ON, 0, 0, 0, 0, 0, NULL) != 0) {
    OsLog ("exynoscam: stream on refused");
    CamCaptureStop (&D->Cap);
    Result = D->Is.SensorI2cFail ? 1 : -3;
    goto Down;
  }
  //
  // The A5 sends the sensor no exposure control without shot requests, so the
  // host does it (registers over the sensor's I2C, see CamAe.c).
  //
  {
    UCHAR   V8 = 0;
    UINT16  Frame = 0;
    UCHAR   Addr;

    D->Sensor.Gpio = D->Board.GpioLb;
    D->Sensor.Bank = 0x40;                 // GPF0
    D->Sensor.Sda  = D->Front ? 4 : 0;     // ISP I2C2 = GPF0(4/5) front, I2C0 = GPF0(0/1) rear
    D->Sensor.Scl  = D->Front ? 5 : 1;
    D->AeOn        = FALSE;
    for (Addr = D->Front ? 0x35 : 0x1A; ; Addr = (Addr == (D->Front ? 0x35 : 0x1A)) ? 0x08 : Addr + 1) {
      if (Addr >= 0x78) {
        break;
      }
      D->Sensor.Addr = Addr;
      if (CamI2cRead (&D->Sensor, 0x0000, &V8) == 0) {
        D->AeOn = TRUE;
        break;
      }
    }
    if (D->AeOn) {
      CamI2cRead16 (&D->Sensor, 0x0340, (UINT16 *)&Frame);
      if (Frame > 100) {
        CamAeInit (&D->Ae, (ULONG)Frame - 6, 150, D->Front ? CAM_AE_S5K6B2 : CAM_AE_IMX134);
        OsLog ("exynoscam: sensor I2C at 0x%02x, frame length %lu: auto exposure on", D->Sensor.Addr, (ULONG)Frame);
      } else {
        D->AeOn = FALSE;
      }
    }
    if (!D->AeOn) {
      OsLog ("exynoscam: no sensor on I2C, no exposure control");
    }
  }
  D->HwUp = TRUE;
  D->StreamStart = KeQueryInterruptTime ();
  OsLog ("exynoscam: sensor streaming");
  return 0;

Down:
  FimcIsCommand (&D->Is, "power down", HIC_POWER_DOWN, 0, 0, 0, 0, 0, NULL);
Fail:
  FimcIsPowerDown (&D->Is);
  if (D->Front) {
    CamFrontPowerOff (&D->Board);
  } else {
    CamRearPowerOff (&D->Board);
  }
  return Result;
}

static BOOLEAN
HwUp (PCAM_DEV D)
{
  ULONG   Second;
  ULONG   Attempt;
  INTN    Result;

  D->RawW  = D->Front ? FRONT_W : RAW_W;
  D->RawH  = D->Front ? FRONT_H : RAW_H;
  D->CropW = D->Front ? ((FRONT_H * 4 / 3) & ~1u) : RAW_W;
  D->CropX = D->Front ? (((FRONT_W - D->CropW) / 2) & ~1u) : 0;
  for (Second = 0; !ExPmicConnect (&D->Pmic); Second++) {
    if (D->Stop || (Second >= PMIC_WAIT_S)) {
      OsLog ("exynoscam: exynospmic not there after %lu s", Second);
      return FALSE;
    }
    Delay (1000);
  }
  for (Attempt = 1; Attempt <= 4 && !D->Stop; Attempt++) {
    Result = HwTry (D);
    if (Result == 0) {
      return TRUE;
    }
    if (Result < 0) {
      return FALSE;
    }
    OsLog ("exynoscam: attempt %lu failed (sensor I2C), retrying", Attempt);
    Delay (500);
  }
  return FALSE;
}

static VOID
HwDown (PCAM_DEV D)
{
  if (!D->HwUp) {
    return;
  }
  CamCaptureDisable (&D->Cap);
  FimcIsCommand (&D->Is, "stream off", HIC_STREAM_OFF, 0, 0, 0, 0, 0, NULL);
  CamCaptureStop (&D->Cap);
  FimcIsCommand (&D->Is, "power down", HIC_POWER_DOWN, 0, 0, 0, 0, 0, NULL);
  FimcIsPowerDown (&D->Is);
  if (D->Front) {
    CamFrontPowerOff (&D->Board);
  } else {
    CamRearPowerOff (&D->Board);
  }
  D->HwUp = FALSE;
  OsLog ("exynoscam: sensor off (%lu frames)", D->Frames);
}

static VOID
Worker (PVOID Context)
{
  PCAM_DEV       D = (PCAM_DEV)Context;
  LARGE_INTEGER  Interval;
  KIRQL          Irql;
  ULONG          Status, CsisSrc, Wait;
  LONGLONG       Last = 0;

  if (HwUp (D)) {
    while (!D->Stop) {
      LONGLONG  Now;

      //
      // One frame: capture on at the next frame start, off at its end.
      //
      CamCaptureEnable (&D->Cap);
      Status = 0;
      for (Wait = 0; (Wait < 500) && !D->Stop; Wait++) {
        Status = CamCaptureTakeStatus (&D->Cap, (UINT32 *)&CsisSrc);
        if (Status & (1u << 4)) {
          break;
        }
        Interval.QuadPart = -10000;                  // 1 ms
        KeDelayExecutionThread (KernelMode, FALSE, &Interval);
      }
      CamCaptureDisable (&D->Cap);
      if (D->Stop) {
        break;
      }
      if ((Status & (1u << 4)) == 0) {
        OsLog ("exynoscam: no frame end in 500 ms");
        continue;
      }
      Convert (D);
      {
        ULONG  Index, Sum = 0, N = 0;

        for (Index = 0; Index < OUT_BYTES; Index += 64) {
          Sum += D->Work[Index];
          N++;
        }
        D->YMean = Sum / N;
      }
      if (D->AeOn && ((D->Frames % 3) == 0)) {
        CamAeStep (&D->Ae, (UINT32)((D->LastMean[1] > 1) ? D->LastMean[1] - 1 : 1));
        if (CamAeApply (&D->Sensor, &D->Ae) != 0) {
          OsLog ("exynoscam: exposure write failed");
        }
        if ((D->Frames % 30) == 0) {
          OsLog ("exynoscam: AE: mean %ld, %lu lines, analog %lu/256, digital %lu/256", D->LastMean[1] - 1, D->Ae.Lines,
                 D->Ae.AnalogX256, D->Ae.DigitalX256);
        }
      }
      KeAcquireSpinLock (&D->FrameLock, &Irql);
      RtlCopyMemory (D->Latest, D->Work, OUT_BYTES);
      KeReleaseSpinLock (&D->FrameLock, Irql);
      InterlockedExchange (&D->FrameReady, 1);
      D->Frames++;
      if (D->Frames <= 40) {
        OsLog ("exynoscam: frame %lu at %ld ms: mean R %ld G %ld B %ld", D->Frames,
               (LONG)((KeQueryInterruptTime () - D->StreamStart) / 10000), D->LastMean[0], D->LastMean[1], D->LastMean[2]);
      }
      if (!D->Restarted && (D->Frames == 25) && (D->LastMean[1] < 3)) {
        //
        // Black: does a fresh stream start give bright first frames?
        //
        D->Restarted = TRUE;
        OsLog ("exynoscam: black after 25 frames, restarting the stream");
        FimcIsCommand (&D->Is, "stream off", HIC_STREAM_OFF, 0, 0, 0, 0, 0, NULL);
        FimcIsCommand (&D->Is, "stream on", HIC_STREAM_ON, 0, 0, 0, 0, 0, NULL);
        D->StreamStart = KeQueryInterruptTime ();
        D->Frames = 0;
      }
      if ((D->Frames % 30) == 1) {
        Now = KeQueryInterruptTime ();
        OsLog ("exynoscam: frame %lu%s: mean R %ld G %ld B %ld (above black), delivered %lu, no-buffer %lu, Ymean %lu, Y[0..3] %02x %02x %02x %02x, "
               "raw[0..3] %04x %04x %04x %04x", D->Frames, Last ? "" : " (first)", D->LastMean[0], D->LastMean[1],
               D->LastMean[2], D->Delivered, D->Starved, D->YMean, D->Latest[0], D->Latest[1], D->Latest[2], D->Latest[3],
               (ULONG)D->Raw[0], (ULONG)D->Raw[1], (ULONG)D->Raw[2], (ULONG)D->Raw[3]);
        Last = Now;
      }
      if (D->Pin != NULL) {
        KsPinAttemptProcessing (D->Pin, TRUE);
      }
      //
      // OUT_FPS: pace the loop (conversion alone usually takes longer).
      //
    }
  }
  HwDown (D);
  PsTerminateSystemThread (STATUS_SUCCESS);
}

static NTSTATUS
StartStreaming (PCAM_DEV D)
{
  HANDLE             Handle;
  OBJECT_ATTRIBUTES  Attr;
  NTSTATUS           Status;

  if (D->Thread != NULL) {
    return STATUS_SUCCESS;
  }
  D->Stop       = FALSE;
  D->Frames     = 0;
  D->FrameReady = 0;
  InitializeObjectAttributes (&Attr, NULL, OBJ_KERNEL_HANDLE, NULL, NULL);
  Status = PsCreateSystemThread (&Handle, THREAD_ALL_ACCESS, &Attr, NULL, NULL, Worker, D);
  if (NT_SUCCESS (Status)) {
    ObReferenceObjectByHandle (Handle, THREAD_ALL_ACCESS, NULL, KernelMode, (PVOID *)&D->Thread, NULL);
    ZwClose (Handle);
  }
  OsLog ("exynoscam: streaming thread - 0x%08lx", Status);
  return Status;
}

static VOID
StopStreaming (PCAM_DEV D)
{
  D->Stop = TRUE;
  if (D->Thread != NULL) {
    KeWaitForSingleObject (D->Thread, Executive, KernelMode, FALSE, NULL);
    ObDereferenceObject (D->Thread);
    D->Thread = NULL;
  }
}

//
// ---------------------------------------------------------------------------
// AVStream: pin
// ---------------------------------------------------------------------------
//

static const GUID  mSubtypeYuy2 = { 0x32595559, 0x0000, 0x0010, { 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71 } };

static const KS_DATARANGE_VIDEO  mRange = {
  {
    sizeof (KS_DATARANGE_VIDEO),
    0,
    OUT_BYTES,
    0,
    STATIC_KSDATAFORMAT_TYPE_VIDEO,
    0x32595559, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71,
    STATIC_KSDATAFORMAT_SPECIFIER_VIDEOINFO
  },
  TRUE,                         // bFixedSizeSamples
  FALSE,                        // bTemporalCompression
  0,
  0,
  {
    STATIC_KSDATAFORMAT_SPECIFIER_VIDEOINFO,
    KS_AnalogVideo_None,
    { OUT_W, OUT_H },           // InputSize
    { OUT_W, OUT_H },           // MinCroppingSize
    { OUT_W, OUT_H },           // MaxCroppingSize
    1, 1, 1, 1,                 // crop granularity / alignment
    { OUT_W, OUT_H },           // MinOutputSize
    { OUT_W, OUT_H },           // MaxOutputSize
    1, 1,                       // output granularity
    0, 0, 0, 0,                 // stretch / shrink taps
    10000000 / OUT_FPS,         // MinFrameInterval
    10000000 / OUT_FPS,         // MaxFrameInterval
    OUT_BYTES * 8 * OUT_FPS,    // MinBitsPerSecond
    OUT_BYTES * 8 * OUT_FPS     // MaxBitsPerSecond
  },
  {
    { 0, 0, OUT_W, OUT_H },     // rcSource
    { 0, 0, OUT_W, OUT_H },     // rcTarget
    OUT_BYTES * 8 * OUT_FPS,    // dwBitRate
    0,
    10000000 / OUT_FPS,         // AvgTimePerFrame
    {
      sizeof (KS_BITMAPINFOHEADER), OUT_W, OUT_H, 1, 16, 0x32595559, OUT_BYTES, 0, 0, 0, 0
    }
  }
};

static const PKSDATARANGE  mRanges[] = { (PKSDATARANGE)&mRange };

static
NTSTATUS
PinIntersect (
  PVOID        Context,
  PIRP         Irp,
  PKSP_PIN     Pin,
  PKSDATARANGE DataRange,
  PKSDATARANGE MatchingDataRange,
  ULONG        DataBufferSize,
  PVOID        Data,
  PULONG       DataSize
  )
{
  KS_DATAFORMAT_VIDEOINFOHEADER  *Format = (KS_DATAFORMAT_VIDEOINFOHEADER *)Data;

  UNREFERENCED_PARAMETER (Context);
  UNREFERENCED_PARAMETER (Irp);
  UNREFERENCED_PARAMETER (Pin);
  UNREFERENCED_PARAMETER (MatchingDataRange);

  if (!IsEqualGUID (&DataRange->Specifier, &KSDATAFORMAT_SPECIFIER_VIDEOINFO)) {
    return STATUS_NO_MATCH;
  }
  if (DataBufferSize == 0) {
    *DataSize = sizeof (KS_DATAFORMAT_VIDEOINFOHEADER);
    return STATUS_BUFFER_OVERFLOW;
  }
  if (DataBufferSize < sizeof (KS_DATAFORMAT_VIDEOINFOHEADER)) {
    return STATUS_BUFFER_TOO_SMALL;
  }
  *DataSize = sizeof (KS_DATAFORMAT_VIDEOINFOHEADER);
  RtlZeroMemory (Format, sizeof (*Format));
  Format->DataFormat.FormatSize  = sizeof (KS_DATAFORMAT_VIDEOINFOHEADER);
  Format->DataFormat.SampleSize  = OUT_BYTES;
  Format->DataFormat.MajorFormat = KSDATAFORMAT_TYPE_VIDEO;
  Format->DataFormat.SubFormat   = mSubtypeYuy2;
  Format->DataFormat.Specifier   = KSDATAFORMAT_SPECIFIER_VIDEOINFO;
  Format->VideoInfoHeader        = mRange.VideoInfoHeader;
  return STATUS_SUCCESS;
}

static const KSFILTER_DESCRIPTOR  mFilterFront;      // defined with the filter descriptors below

static
NTSTATUS
PinCreate (PKSPIN Pin, PIRP Irp)
{
  PCAM_DEV  D = (PCAM_DEV)KsPinGetDevice (Pin)->Context;
  PKSFILTER Filter = KsPinGetParentFilter (Pin);

  UNREFERENCED_PARAMETER (Irp);
  if (D->Pin != NULL) {
    OsLog ("exynoscam: pin refused, the other camera is in use");
    return STATUS_DEVICE_BUSY;                // one camera at a time: they share the A5
  }
  D->Front     = (Filter->Descriptor == &mFilterFront);
  Pin->Context = D;
  D->Pin       = Pin;
  D->Delivered = 0;
  if ((Pin->ConnectionFormat != NULL) && (Pin->ConnectionFormat->FormatSize >= sizeof (KS_DATAFORMAT_VIDEOINFOHEADER))) {
    KS_DATAFORMAT_VIDEOINFOHEADER  *F = (KS_DATAFORMAT_VIDEOINFOHEADER *)Pin->ConnectionFormat;

    OsLog ("exynoscam: %s pin created, format %lux%ld fourcc %08lx, %lu bytes, sample size %lu", D->Front ? "front" : "rear", (ULONG)F->VideoInfoHeader.bmiHeader.biWidth,
           F->VideoInfoHeader.bmiHeader.biHeight, F->DataFormat.SubFormat.Data1, F->VideoInfoHeader.bmiHeader.biSizeImage,
           F->DataFormat.SampleSize);
  } else {
    OsLog ("exynoscam: pin created, no connection format yet");
  }
  return STATUS_SUCCESS;
}

static
NTSTATUS
PinClose (PKSPIN Pin, PIRP Irp)
{
  PCAM_DEV  D = (PCAM_DEV)Pin->Context;

  UNREFERENCED_PARAMETER (Irp);
  StopStreaming (D);
  D->Pin = NULL;
  OsLog ("exynoscam: pin closed");
  return STATUS_SUCCESS;
}

static
NTSTATUS
PinSetDataFormat (
  PKSPIN                 Pin,
  PKSDATAFORMAT          OldFormat,
  PKSMULTIPLE_ITEM       OldAttributeList,
  const KSDATARANGE      *DataRange,
  const KSATTRIBUTE_LIST *AttributeRange
  )
{
  UNREFERENCED_PARAMETER (Pin);
  UNREFERENCED_PARAMETER (OldFormat);
  UNREFERENCED_PARAMETER (OldAttributeList);
  UNREFERENCED_PARAMETER (AttributeRange);

  if (!IsEqualGUID (&DataRange->Specifier, &KSDATAFORMAT_SPECIFIER_VIDEOINFO)) {
    return STATUS_NO_MATCH;
  }
  return STATUS_SUCCESS;
}

static
NTSTATUS
PinSetDeviceState (PKSPIN Pin, KSSTATE To, KSSTATE From)
{
  PCAM_DEV  D = (PCAM_DEV)Pin->Context;

  OsLog ("exynoscam: pin state %ld -> %ld", (LONG)From, (LONG)To);
  if (To == KSSTATE_STOP) {
    StopStreaming (D);
  } else if (From == KSSTATE_STOP) {
    D->RunStart = KeQueryInterruptTime ();
    return StartStreaming (D);
  }
  if (To == KSSTATE_RUN) {
    D->RunStart = KeQueryInterruptTime ();
  }
  return STATUS_SUCCESS;
}

static
NTSTATUS
PinProcess (PKSPIN Pin)
{
  PCAM_DEV           D = (PCAM_DEV)Pin->Context;
  PKSSTREAM_POINTER  Leading;
  PKSSTREAM_HEADER   Header;
  KIRQL              Irql;

  Leading = KsPinGetLeadingEdgeStreamPointer (Pin, KSSTREAM_POINTER_STATE_LOCKED);
  if (Leading == NULL) {
    D->Starved++;
    return STATUS_SUCCESS;
  }
  if (InterlockedCompareExchange (&D->FrameReady, 0, 1) == 0) {
    KsStreamPointerUnlock (Leading, FALSE);
    return STATUS_PENDING;                   // the worker calls KsPinAttemptProcessing
  }
  D->Delivered++;
  if (Leading->OffsetOut.Remaining < OUT_BYTES) {
    KsStreamPointerAdvanceOffsetsAndUnlock (Leading, 0, Leading->OffsetOut.Remaining, TRUE);
    return STATUS_SUCCESS;
  }
  KeAcquireSpinLock (&D->FrameLock, &Irql);
  RtlCopyMemory (Leading->OffsetOut.Data, D->Latest, OUT_BYTES);
  KeReleaseSpinLock (&D->FrameLock, Irql);

  Header = Leading->StreamHeader;
  {
    //
    // Frame times are system time (100 ns since boot), the time base of the
    // presentation clock the capture engine compares samples with; times
    // relative to the stream start look hours late to its preview sink.
    //
    LONGLONG  Now = KeQueryInterruptTime ();

    if ((D->Delivered % 120) == 1) {
      OsLog ("exynoscam: delivering frame %lu at system time %lld (header size %lu, extent %lu, remaining %lu)", D->Delivered,
             Now, Header->Size, Header->FrameExtent, Leading->OffsetOut.Remaining);
    }
    Header->PresentationTime.Numerator   = 1;
    Header->PresentationTime.Denominator = 1;
    Header->PresentationTime.Time        = Now;
    Header->Duration                     = 10000000 / OUT_FPS;
    Header->OptionsFlags                |= KSSTREAM_HEADER_OPTIONSF_SPLICEPOINT | KSSTREAM_HEADER_OPTIONSF_TIMEVALID |
                                            KSSTREAM_HEADER_OPTIONSF_DURATIONVALID;
  }
    Header->DataUsed                     = OUT_BYTES;
    //
    // Clients such as the Camera app ask for the extended frame info
    // (stream header size 112 = header + KS_FRAME_INFO) and expect it filled
    // in like the AVStream capture samples do.
    //
    if (Header->Size >= sizeof (KSSTREAM_HEADER) + sizeof (KS_FRAME_INFO)) {
      KS_FRAME_INFO  *Info = (KS_FRAME_INFO *)(Header + 1);

      RtlZeroMemory (Info, sizeof (*Info));
      Info->ExtendedHeaderSize = sizeof (KS_FRAME_INFO);
      Info->dwFrameFlags       = KS_VIDEO_FLAG_FRAME | KS_VIDEO_FLAG_I_FRAME;
      Info->PictureNumber      = D->Delivered;
      Info->DropCount          = 0;
    }
  KsStreamPointerAdvanceOffsetsAndUnlock (Leading, 0, OUT_BYTES, TRUE);
  return STATUS_SUCCESS;
}

static const KSPIN_DISPATCH  mPinDispatch = {
  PinCreate,
  PinClose,
  PinProcess,
  NULL,                         // Reset
  PinSetDataFormat,
  PinSetDeviceState,
  NULL, NULL, NULL, NULL        // Connect, Disconnect, Clock, Allocator
};

static DECLARE_SIMPLE_FRAMING_EX (
  mFraming,
  STATIC_KSMEMORY_TYPE_KERNEL_NONPAGED,
  KSALLOCATOR_REQUIREMENTF_SYSTEM_MEMORY | KSALLOCATOR_REQUIREMENTF_PREFERENCES_ONLY,
  4,
  0,
  OUT_BYTES,
  OUT_BYTES
  );

static const KSPIN_DESCRIPTOR_EX  mPins[1] = {
  {
    &mPinDispatch,
    NULL,
    {
      DEFINE_KSPIN_DEFAULT_INTERFACES,
      DEFINE_KSPIN_DEFAULT_MEDIUMS,
      1,
      mRanges,
      KSPIN_DATAFLOW_OUT,
      KSPIN_COMMUNICATION_BOTH,
      &PINNAME_VIDEO_CAPTURE,
      &PINNAME_VIDEO_CAPTURE,
      0
    },
    KSPIN_FLAG_PROCESS_IN_RUN_STATE_ONLY,
    1,                          // InstancesPossible
    0,                          // InstancesNecessary
    &mFraming,
    PinIntersect
  }
};

static const GUID  mCategories[] = {
  STATIC_KSCATEGORY_VIDEO_CAMERA,
  STATIC_KSCATEGORY_CAPTURE,
  STATIC_KSCATEGORY_VIDEO
};

static const KSFILTER_DESCRIPTOR  mFilter = {
  NULL,                         // dispatch
  NULL,                         // automation
  KSFILTER_DESCRIPTOR_VERSION,
  0,
  &KSCATEGORY_VIDEO,            // ReferenceGuid
  1,
  sizeof (KSPIN_DESCRIPTOR_EX),
  mPins,
  3,
  mCategories,
  0,
  sizeof (KSNODE_DESCRIPTOR),
  NULL,
  0,
  NULL,
  NULL
};

//
// The front camera: same filter shape, its own factory ("FRONT"), told apart in PinCreate by its descriptor.
//
static const KSFILTER_DESCRIPTOR  mFilterFront = {
  NULL,                         // dispatch
  NULL,                         // automation
  KSFILTER_DESCRIPTOR_VERSION,
  0,
  &KSCATEGORY_VIDEO,            // ReferenceGuid
  1,
  sizeof (KSPIN_DESCRIPTOR_EX),
  mPins,
  3,
  mCategories,
  0,
  sizeof (KSNODE_DESCRIPTOR),
  NULL,
  0,
  NULL,
  NULL
};

//
// ---------------------------------------------------------------------------
// AVStream: device
// ---------------------------------------------------------------------------
//

static
NTSTATUS
DevAdd (PKSDEVICE Device)
{
  PCAM_DEV  D;
  NTSTATUS  Status;
  ULONG     Index;

  D = ExAllocatePoolWithTag (NonPagedPool, sizeof (CAM_DEV), CAM_TAG);
  if (D == NULL) {
    return STATUS_INSUFFICIENT_RESOURCES;
  }
  RtlZeroMemory (D, sizeof (*D));
  Status = KsAddItemToObjectBag (Device->Bag, D, NULL);
  if (!NT_SUCCESS (Status)) {
    ExFreePoolWithTag (D, CAM_TAG);
    return Status;
  }
  D->Device      = Device;
  Device->Context = D;
  KeInitializeSpinLock (&D->FrameLock);
  for (Index = 0; Index < 1024; Index++) {
    D->Gamma[Index] = (UCHAR)ISqrt (Index * 65025 / 1023);        // gamma 2.0
  }
  Status = KsCreateFilterFactory (Device->FunctionalDeviceObject, &mFilter, L"GLOBAL", NULL, KSCREATE_ITEM_FREEONSTOP,
                                  NULL, NULL, NULL);
  OsLog ("exynoscam: device added, rear filter factory - 0x%08lx", Status);
  if (NT_SUCCESS (Status)) {
    Status = KsCreateFilterFactory (Device->FunctionalDeviceObject, &mFilterFront, L"FRONT", NULL, KSCREATE_ITEM_FREEONSTOP,
                                    NULL, NULL, NULL);
    OsLog ("exynoscam: front filter factory - 0x%08lx", Status);
  }
  return Status;
}

static
NTSTATUS
DevStart (
  PKSDEVICE        Device,
  PIRP             Irp,
  PCM_RESOURCE_LIST Translated,
  PCM_RESOURCE_LIST Untranslated
  )
{
  PCAM_DEV                          D = (PCAM_DEV)Device->Context;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR   Desc;
  PHYSICAL_ADDRESS                  Pa;
  PHYSICAL_ADDRESS                  Low;
  PHYSICAL_ADDRESS                  High;
  PHYSICAL_ADDRESS                  Boundary;
  ULONG                             Index;
  NTSTATUS                          Status;

  UNREFERENCED_PARAMETER (Irp);
  UNREFERENCED_PARAMETER (Untranslated);

  if (D->Is.Is != 0) {
    return STATUS_SUCCESS;                      // restart
  }
  if ((Translated != NULL) && (Translated->Count > 0)) {
    PCM_PARTIAL_RESOURCE_LIST  List = &Translated->List[0].PartialResourceList;

    for (Index = 0; Index < List->Count; Index++) {
      Desc = &List->PartialDescriptors[Index];
      if ((Desc->Type == CmResourceTypeMemory) && (D->Is.Is == 0)) {
        D->Is.Is = (UINTN)MmMapIoSpace (Desc->u.Memory.Start, FIMC_IS_SIZE, MmNonCached);
      }
    }
  }
  D->Is.Pmu       = (UINTN)MapPhys (FIMC_IS_PMU_PHYS, PMU_SIZE);
  D->Is.Cmu       = (UINTN)MapPhys (FIMC_IS_CMU_PHYS, CMU_SIZE);
  D->Is.Sysreg    = (UINTN)MapPhys (FIMC_IS_SYSREG_PHYS, SYSREG_SIZE);
  D->Board.GpioLb = (UINTN)MapPhys (CAM_GPIO_LB_PHYS, 0x1000);
  D->Board.GpioB  = (UINTN)MapPhys (CAM_GPIO_B_PHYS, 0x1000);
  D->Board.CmuTop = (UINTN)MapPhys (CAM_CMU_TOP_PHYS, 0x1000);
  D->Board.GpioRt = (UINTN)MapPhys (CAM_GPIO_RT_PHYS, 0x1000);
  D->CsisBase[0]  = (UINTN)MapPhys (CAM_CSIS0_PHYS, 0x1000);
  D->CsisBase[1]  = (UINTN)MapPhys (CAM_CSIS1_PHYS, 0x1000);
  D->FliteBase[0] = (UINTN)MapPhys (CAM_FLITE_A_PHYS, 0x1000);
  D->FliteBase[1] = (UINTN)MapPhys (CAM_FLITE_B_PHYS, 0x1000);
  D->Cap.Pmu      = D->Is.Pmu;
  if ((D->Is.Is == 0) || (D->Is.Pmu == 0) || (D->Is.Cmu == 0) || (D->Is.Sysreg == 0) || (D->Board.GpioLb == 0) ||
      (D->Board.GpioB == 0) || (D->Board.GpioRt == 0) || (D->Board.CmuTop == 0) || (D->CsisBase[0] == 0) ||
      (D->CsisBase[1] == 0) || (D->FliteBase[0] == 0) || (D->FliteBase[1] == 0)) {
    OsLog ("exynoscam: register windows missing (IS %p)", (PVOID)D->Is.Is);
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }
  D->Is.SecureWrite = SecureWrite;
  D->Is.SecureRead  = SecureRead;

  Status = LoadFile (FW_PATH, &D->Fw, &D->FwSize);
  OsLog ("exynoscam: firmware %lu bytes - 0x%08lx", D->FwSize, Status);
  if (NT_SUCCESS (Status)) {
    Status = LoadFile (SETFILE_PATH, &D->Setfile, &D->SetfileSize);
    OsLog ("exynoscam: setfile %lu bytes - 0x%08lx", D->SetfileSize, Status);
    if (NT_SUCCESS (Status)) {
      Status = LoadFile (SETFILE_FRONT_PATH, &D->SetfileFront, &D->SetfileFrontSize);
      OsLog ("exynoscam: front setfile %lu bytes - 0x%08lx", D->SetfileFrontSize, Status);
    }
  }
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  D->Is.MemVa = AllocA5Memory (&Pa);
  if (D->Is.MemVa == NULL) {
    OsLog ("exynoscam: no 20 MiB / 64 MiB-aligned memory");
    return STATUS_INSUFFICIENT_RESOURCES;
  }
  D->Is.MemPa = (UINT32)Pa.QuadPart;
  OsLog ("exynoscam: A5 memory at 0x%08lx", D->Is.MemPa);

  Low.QuadPart      = 0;
  High.QuadPart     = 0xFFFFFFFF;
  Boundary.QuadPart = 0;
  D->Raw = MmAllocateContiguousMemorySpecifyCache (RAW_BYTES, Low, High, Boundary, MmNonCached);
  D->Work   = ExAllocatePoolWithTag (NonPagedPool, OUT_BYTES, CAM_TAG);
  D->Latest = ExAllocatePoolWithTag (NonPagedPool, OUT_BYTES, CAM_TAG);
  if ((D->Raw == NULL) || (D->Work == NULL) || (D->Latest == NULL)) {
    OsLog ("exynoscam: no frame buffers");
    return STATUS_INSUFFICIENT_RESOURCES;
  }
  D->RawPa = MmGetPhysicalAddress (D->Raw);
  RtlZeroMemory (D->Latest, OUT_BYTES);
  OsLog ("exynoscam: started; raw buffer at 0x%08lx", (ULONG)D->RawPa.QuadPart);
  return STATUS_SUCCESS;
}

static VOID
Teardown (PCAM_DEV D)
{
  StopStreaming (D);
  if (D->Is.Is != 0) { MmUnmapIoSpace ((PVOID)D->Is.Is, FIMC_IS_SIZE); D->Is.Is = 0; }
  if (D->Is.Pmu != 0) { MmUnmapIoSpace ((PVOID)D->Is.Pmu, PMU_SIZE); D->Is.Pmu = 0; D->Cap.Pmu = 0; }
  if (D->Is.Cmu != 0) { MmUnmapIoSpace ((PVOID)D->Is.Cmu, CMU_SIZE); D->Is.Cmu = 0; }
  if (D->Is.Sysreg != 0) { MmUnmapIoSpace ((PVOID)D->Is.Sysreg, SYSREG_SIZE); D->Is.Sysreg = 0; }
  if (D->Board.GpioLb != 0) { MmUnmapIoSpace ((PVOID)D->Board.GpioLb, 0x1000); D->Board.GpioLb = 0; }
  if (D->Board.GpioB != 0) { MmUnmapIoSpace ((PVOID)D->Board.GpioB, 0x1000); D->Board.GpioB = 0; }
  if (D->Board.CmuTop != 0) { MmUnmapIoSpace ((PVOID)D->Board.CmuTop, 0x1000); D->Board.CmuTop = 0; }
  {
    ULONG  I;

    for (I = 0; I < 2; I++) {
      if (D->CsisBase[I] != 0) { MmUnmapIoSpace ((PVOID)D->CsisBase[I], 0x1000); D->CsisBase[I] = 0; }
      if (D->FliteBase[I] != 0) { MmUnmapIoSpace ((PVOID)D->FliteBase[I], 0x1000); D->FliteBase[I] = 0; }
    }
    if (D->Board.GpioRt != 0) { MmUnmapIoSpace ((PVOID)D->Board.GpioRt, 0x1000); D->Board.GpioRt = 0; }
  }
  if (D->Is.MemVa != NULL) {
    MmFreeContiguousMemorySpecifyCache (D->Is.MemVa, FIMC_IS_A5_MEM_SIZE, MmNonCached);
    D->Is.MemVa = NULL;
  }
  if (D->Raw != NULL) { MmFreeContiguousMemorySpecifyCache (D->Raw, RAW_BYTES, MmNonCached); D->Raw = NULL; }
  if (D->Work != NULL) { ExFreePoolWithTag (D->Work, CAM_TAG); D->Work = NULL; }
  if (D->Latest != NULL) { ExFreePoolWithTag (D->Latest, CAM_TAG); D->Latest = NULL; }
  if (D->Fw != NULL) { ExFreePoolWithTag (D->Fw, CAM_TAG); D->Fw = NULL; }
  if (D->Setfile != NULL) { ExFreePoolWithTag (D->Setfile, CAM_TAG); D->Setfile = NULL; }
  if (D->SetfileFront != NULL) { ExFreePoolWithTag (D->SetfileFront, CAM_TAG); D->SetfileFront = NULL; }
}

static
VOID
DevStop (PKSDEVICE Device, PIRP Irp)
{
  UNREFERENCED_PARAMETER (Irp);
  Teardown ((PCAM_DEV)Device->Context);
}

static
VOID
DevRemove (PKSDEVICE Device, PIRP Irp)
{
  UNREFERENCED_PARAMETER (Irp);
  Teardown ((PCAM_DEV)Device->Context);
}

static const KSDEVICE_DISPATCH  mDeviceDispatch = {
  DevAdd,
  DevStart,
  NULL,                         // PostStart
  NULL,                         // QueryStop
  NULL,                         // CancelStop
  DevStop,
  NULL,                         // QueryRemove
  NULL,                         // CancelRemove
  DevRemove,
  NULL,                         // QueryCapabilities
  NULL,                         // SurpriseRemoval
  NULL,                         // QueryPower
  NULL,                         // SetPower
  NULL                          // QueryInterface
};

static const KSDEVICE_DESCRIPTOR  mDevice = {
  &mDeviceDispatch,
  0,
  NULL,
  KSDEVICE_DESCRIPTOR_VERSION
};

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  OsLogInit ();
  OsLog ("exynoscam: DriverEntry, AVStream (built " __DATE__ " " __TIME__ ")");
  return KsInitializeDriver (DriverObject, RegistryPath, &mDevice);
}
