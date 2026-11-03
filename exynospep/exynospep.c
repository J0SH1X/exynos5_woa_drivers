/** @file
  Exynos5420 Power Engine Plugin (PEP) for Windows RT 8.1, v0.

  A PEP is the SoC-specific half of the Windows power framework (PoFx): the
  kernel offers it every power-managed device (DPM notifications) and the
  processor driver, fxppm.sys, asks it for the processors' idle and
  performance states (PPM notifications).  Without one, RT 8.1 has no
  processor power management of its own on this SoC.

  v0 (diagnostic + safe):
    - registers in DriverEntry (boot start), kernel information V2;
    - logs every device the kernel offers;
    - accepts the processors (ACPI0007) with one idle state, WFI (C1):
      the only runtime state that is practical on the 5420 (cpuidle.md:
      C2 is disabled for the 5420 downstream, AFTR/LPA only for the last
      A7 core);
    - performance states not offered yet (PERF_SET is where CPU DVFS goes:
      APLL + MAX77802 BUCK2, CpuFreqDxe's sequence);
    - every other device is declined, so its power handling is unchanged.

  Interface, from RT 8.1's own binaries (ntoskrnl PopFxRegisterPluginEx,
  fxppm.sys): PEP_INFORMATION must be version 2, 12 bytes (device and
  processor callbacks); kernel information version 1 (28 bytes) or 2 (44).
  fxppm sends PPM notifications 0x01 query capabilities, 0x02 / 0x12 query
  idle states (v1 / v2), 0x03 idle select, 0x04 cancel, 0x05 execute,
  0x06 complete, 0x07 is halted, 0x08 initiate wake, 0x09 / 0x0A feedback
  counters, 0x0B..0x0E performance, 0x10 / 0x13 / 0x17 platform states,
  0x11 LP settings, 0x14 test idle state.  The WDK10 pepfx.h does not
  build for 8.1 (PEP_INFORMATION_VERSION is WINTHRESHOLD-only), so the 8.1
  layouts are declared here, from pep_x.h.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#define OSLOG_GLOBAL
#include <ntddk.h>
#include "oslog.h"

//
// ---------------------------------------------------------------------------
// PoFx plugin interface, Windows 8.1 layouts (pep_x.h).
// ---------------------------------------------------------------------------
//

typedef PVOID  POHANDLE_;
typedef PVOID  PEPHANDLE_;

typedef BOOLEAN (*PEP_NOTIFY_DPM_FN) (ULONG Notification, PVOID Data);
typedef BOOLEAN (*PEP_NOTIFY_PPM_FN) (PEPHANDLE_ Handle, ULONG Notification, PVOID Data);

typedef struct {
  USHORT             Version;            // 2
  USHORT             Size;               // 12
  PEP_NOTIFY_DPM_FN  AcceptDeviceNotification;
  PEP_NOTIFY_PPM_FN  AcceptProcessorNotification;
} PEP_INFORMATION_81;

typedef struct {
  USHORT    Version;                     // 2
  USHORT    Size;                        // 44
  POHANDLE_ Plugin;
  PVOID     RequestWorker;
  PVOID     EnumerateUnmaskedInterrupts;
  PVOID     ProcessorHalt;
  PVOID     RequestInterrupt;
  PVOID     TransitionCriticalResource;
  PVOID     ProcessorIdleVeto;
  PVOID     PlatformIdleVeto;
  PVOID     UpdateProcessorIdleState;
  PVOID     UpdatePlatformIdleState;
} PEP_KERNEL_INFORMATION_81;

NTKERNELAPI NTSTATUS PoFxRegisterPlugin (PEP_INFORMATION_81 *PepInformation, PEP_KERNEL_INFORMATION_81 *KernelInformation);

//
// DPM notifications.
//
#define PEP_DPM_PREPARE_DEVICE        0x01
#define PEP_DPM_ABANDON_DEVICE        0x02
#define PEP_DPM_REGISTER_DEVICE       0x03
#define PEP_DPM_UNREGISTER_DEVICE     0x04
#define PEP_DPM_DEVICE_POWER_STATE    0x05

typedef struct {
  PCUNICODE_STRING  DeviceId;
  BOOLEAN           DeviceAccepted;
} PEP_PREPARE_DEVICE_81;

typedef struct {
  PCUNICODE_STRING  DeviceId;
  POHANDLE_         KernelHandle;
  PVOID             Register;
  PEPHANDLE_        DeviceHandle;
  ULONG             DeviceAccepted;       // PEP_DEVICE_ACCEPTANCE_TYPE: 0 not, 2 accepted
} PEP_REGISTER_DEVICE_81;

typedef struct {
  PEPHANDLE_          DeviceHandle;
  DEVICE_POWER_STATE  PowerState;
  BOOLEAN             Complete;
  BOOLEAN             SystemTransition;
} PEP_DEVICE_POWER_STATE_81;

//
// PPM notifications (numbers as RT 8.1's fxppm.sys sends them).
//
#define PPM_QUERY_CAPABILITIES        0x01
#define PPM_QUERY_IDLE_STATES         0x02
#define PPM_IDLE_SELECT               0x03
#define PPM_IDLE_CANCEL               0x04
#define PPM_IDLE_EXECUTE              0x05
#define PPM_IDLE_COMPLETE             0x06
#define PPM_IS_PROCESSOR_HALTED       0x07
#define PPM_INITIATE_WAKE             0x08
#define PPM_QUERY_PLATFORM_STATES     0x10
#define PPM_QUERY_IDLE_STATES_V2      0x12
#define PPM_TEST_IDLE_STATE           0x14

typedef struct {
  ULONG    FeedbackCounterCount;
  ULONG    IdleStateCount;
  BOOLEAN  PerformanceStatesSupported;
  BOOLEAN  ParkingSupported;
  UCHAR    DiscretePerformanceStateCount;
  BOOLEAN  Reserved;
} PEP_PPM_QUERY_CAPABILITIES_81;

// Idle state flags: [0] interruptible, [1] cache coherent, [2] context
// retained, [6:3] C-state type.
#define IDLE_WFI_FLAGS  ((1u << 0) | (1u << 1) | (1u << 2) | (1u << 3))   // C1

typedef struct {
  ULONG  Count;
  ULONG  MaximumCoordinatedProcessors;
  ULONG  IdleStates[1];
} PEP_PPM_QUERY_IDLE_STATES_81;

typedef struct {
  ULONG  Flags;
  ULONG  Latency;                           // µs
  ULONG  BreakEvenDuration;                 // µs
} PEP_PROCESSOR_IDLE_STATE_V2_81;

typedef struct {
  ULONG                           Count;
  PEP_PROCESSOR_IDLE_STATE_V2_81  IdleStates[1];
} PEP_PPM_QUERY_IDLE_STATES_V2_81;

typedef struct {
  PVOID    Constraints;
  BOOLEAN  AbortTransition;
  ULONG    IdleStateIndex;
  ULONG    DependencyArrayUsed;
  ULONG    DependencyArrayCount;
  PVOID    DependencyArray;
  ULONG    PlatformIdleStateIndex;
} PEP_PPM_IDLE_SELECT_81;

typedef struct {
  NTSTATUS  Status;
  ULONG     ProcessorState;
  ULONG     PlatformState;
} PEP_PPM_IDLE_EXECUTE_81;

typedef struct {
  BOOLEAN  Halted;
} PEP_PPM_IS_PROCESSOR_HALTED_81;

typedef struct {
  BOOLEAN  NeedInterruptForCompletion;
} PEP_PPM_INITIATE_WAKE_81;

typedef struct {
  ULONG  PlatformStateCount;
} PEP_PPM_QUERY_PLATFORM_STATES_81;

typedef struct {
  ULONG  ProcessorState;
  ULONG  PlatformState;
  ULONG  VetoReason;
} PEP_PPM_TEST_IDLE_STATE_81;

#define PLATFORM_IDLE_STATE_NONE  0xFFFFFFFF

//
// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
//

#define MAX_CPUS  8

typedef struct {
  ULONG      Index;
  POHANDLE_  KernelHandle;
  ULONG      Idles;
} PEP_CPU;

static PEP_INFORMATION_81         mPep;
static PEP_KERNEL_INFORMATION_81  mKernel;
static PEP_CPU                    mCpu[MAX_CPUS];
static volatile LONG              mCpuCount;
static ULONG                      mDeviceLogs;
static ULONG                      mPpmLogs;

static BOOLEAN
IsProcessor (PCUNICODE_STRING Id)
{
  static CONST WCHAR  Pattern[] = L"ACPI0007";
  USHORT              Length = (Id != NULL) ? Id->Length / sizeof (WCHAR) : 0;
  USHORT              Index;
  USHORT              Match;

  for (Index = 0; Index + 8 <= Length; Index++) {
    for (Match = 0; (Match < 8) && (Id->Buffer[Index + Match] == Pattern[Match]); Match++) {
    }
    if (Match == 8) {
      return TRUE;
    }
  }
  return FALSE;
}

//
// ---------------------------------------------------------------------------
// Devices (DPM): processors accepted, everything else declined.
// ---------------------------------------------------------------------------
//

static BOOLEAN
AcceptDeviceNotification (ULONG Notification, PVOID Data)
{
  switch (Notification) {
    case PEP_DPM_PREPARE_DEVICE: {
      PEP_PREPARE_DEVICE_81  *Prep = Data;

      Prep->DeviceAccepted = IsProcessor (Prep->DeviceId);
      if (mDeviceLogs++ < 80) {
        OsLog ("exynospep: device %wZ %s", Prep->DeviceId, Prep->DeviceAccepted ? "accepted" : "declined");
      }
      return TRUE;
    }
    case PEP_DPM_REGISTER_DEVICE: {
      PEP_REGISTER_DEVICE_81  *Reg = Data;
      LONG                    Index;

      if (!IsProcessor (Reg->DeviceId) || ((Index = InterlockedIncrement (&mCpuCount) - 1) >= MAX_CPUS)) {
        Reg->DeviceAccepted = 0;
        return TRUE;
      }
      mCpu[Index].Index        = (ULONG)Index;
      mCpu[Index].KernelHandle = Reg->KernelHandle;
      Reg->DeviceHandle        = &mCpu[Index];
      Reg->DeviceAccepted      = 2;
      OsLog ("exynospep: processor %ld registered (%wZ)", Index, Reg->DeviceId);
      return TRUE;
    }
    case PEP_DPM_DEVICE_POWER_STATE: {
      PEP_DEVICE_POWER_STATE_81  *State = Data;

      State->Complete = TRUE;                      // nothing to sequence for the cores
      return TRUE;
    }
    case PEP_DPM_ABANDON_DEVICE:
    case PEP_DPM_UNREGISTER_DEVICE:
      return TRUE;
    default:
      return FALSE;
  }
}

//
// ---------------------------------------------------------------------------
// Processors (PPM): one idle state, WFI.
// ---------------------------------------------------------------------------
//

static BOOLEAN
AcceptProcessorNotification (PEPHANDLE_ Handle, ULONG Notification, PVOID Data)
{
  PEP_CPU  *Cpu = Handle;

  if ((Notification != PPM_IDLE_SELECT) && (Notification != PPM_IDLE_EXECUTE) && (Notification != PPM_IDLE_COMPLETE) &&
      (Notification != PPM_TEST_IDLE_STATE) && (mPpmLogs < 60)) {
    mPpmLogs++;
    OsLog ("exynospep: cpu %lu PPM notification 0x%02lx", (Cpu != NULL) ? Cpu->Index : 99, Notification);
  }

  switch (Notification) {
    case PPM_QUERY_CAPABILITIES: {
      PEP_PPM_QUERY_CAPABILITIES_81  *Caps = Data;

      RtlZeroMemory (Caps, sizeof (*Caps));
      Caps->IdleStateCount = 1;
      return TRUE;
    }
    case PPM_QUERY_IDLE_STATES: {
      PEP_PPM_QUERY_IDLE_STATES_81  *States = Data;

      if (States->Count < 1) {
        return FALSE;
      }
      States->MaximumCoordinatedProcessors = 1;
      States->IdleStates[0] = IDLE_WFI_FLAGS;
      return TRUE;
    }
    case PPM_QUERY_IDLE_STATES_V2: {
      PEP_PPM_QUERY_IDLE_STATES_V2_81  *States = Data;

      if (States->Count < 1) {
        return FALSE;
      }
      States->IdleStates[0].Flags             = IDLE_WFI_FLAGS;
      States->IdleStates[0].Latency           = 1;
      States->IdleStates[0].BreakEvenDuration = 1;
      return TRUE;
    }
    case PPM_IDLE_SELECT: {
      PEP_PPM_IDLE_SELECT_81  *Select = Data;

      Select->AbortTransition        = FALSE;
      Select->IdleStateIndex         = 0;
      Select->DependencyArrayUsed    = 0;
      Select->PlatformIdleStateIndex = PLATFORM_IDLE_STATE_NONE;
      return TRUE;
    }
    case PPM_IDLE_EXECUTE: {
      PEP_PPM_IDLE_EXECUTE_81  *Exec = Data;

      //
      // Interrupts are disabled here; WFI still wakes on a pending one.
      //
      __builtin_arm_dsb (15);
      __builtin_arm_wfi ();
      Exec->Status = STATUS_SUCCESS;
      if (Cpu != NULL) {
        Cpu->Idles++;
      }
      return TRUE;
    }
    case PPM_IDLE_CANCEL:
    case PPM_IDLE_COMPLETE:
      return TRUE;
    case PPM_IS_PROCESSOR_HALTED:
      ((PEP_PPM_IS_PROCESSOR_HALTED_81 *)Data)->Halted = FALSE;
      return TRUE;
    case PPM_INITIATE_WAKE:
      ((PEP_PPM_INITIATE_WAKE_81 *)Data)->NeedInterruptForCompletion = TRUE;
      return TRUE;
    case PPM_QUERY_PLATFORM_STATES:
      ((PEP_PPM_QUERY_PLATFORM_STATES_81 *)Data)->PlatformStateCount = 0;
      return TRUE;
    case PPM_TEST_IDLE_STATE:
      ((PEP_PPM_TEST_IDLE_STATE_81 *)Data)->VetoReason = 0;
      return TRUE;
    default:
      return FALSE;                                // feedback / perf / parking: not offered
  }
}

//
// ---------------------------------------------------------------------------
// The ACPI device (EXYN5014) the driver loads for: a pass-through FDO.
// ---------------------------------------------------------------------------
//

typedef struct {
  PDEVICE_OBJECT  Lower;
} PEP_FDO;

static NTSTATUS
DispatchPass (PDEVICE_OBJECT Device, PIRP Irp)
{
  PEP_FDO  *Fdo = Device->DeviceExtension;

  IoSkipCurrentIrpStackLocation (Irp);
  return IoCallDriver (Fdo->Lower, Irp);
}

static NTSTATUS
DispatchPnp (PDEVICE_OBJECT Device, PIRP Irp)
{
  PEP_FDO   *Fdo = Device->DeviceExtension;
  UCHAR     Minor = IoGetCurrentIrpStackLocation (Irp)->MinorFunction;
  NTSTATUS  Status;

  if (Minor == IRP_MN_REMOVE_DEVICE) {
    Irp->IoStatus.Status = STATUS_SUCCESS;
    IoSkipCurrentIrpStackLocation (Irp);
    Status = IoCallDriver (Fdo->Lower, Irp);
    IoDetachDevice (Fdo->Lower);
    IoDeleteDevice (Device);
    return Status;
  }
  if ((Minor == IRP_MN_START_DEVICE) || (Minor == IRP_MN_QUERY_STOP_DEVICE) || (Minor == IRP_MN_QUERY_REMOVE_DEVICE)) {
    Irp->IoStatus.Status = STATUS_SUCCESS;
  }
  IoSkipCurrentIrpStackLocation (Irp);
  return IoCallDriver (Fdo->Lower, Irp);
}

static NTSTATUS
DispatchPower (PDEVICE_OBJECT Device, PIRP Irp)
{
  PEP_FDO  *Fdo = Device->DeviceExtension;

  PoStartNextPowerIrp (Irp);
  IoSkipCurrentIrpStackLocation (Irp);
  return PoCallDriver (Fdo->Lower, Irp);
}

static NTSTATUS
AddDevice (PDRIVER_OBJECT DriverObject, PDEVICE_OBJECT Pdo)
{
  PDEVICE_OBJECT  Device;
  PEP_FDO         *Fdo;
  NTSTATUS        Status;

  Status = IoCreateDevice (DriverObject, sizeof (PEP_FDO), NULL, FILE_DEVICE_UNKNOWN, 0, FALSE, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  Fdo = Device->DeviceExtension;
  Fdo->Lower = IoAttachDeviceToDeviceStack (Device, Pdo);
  if (Fdo->Lower == NULL) {
    IoDeleteDevice (Device);
    return STATUS_NO_SUCH_DEVICE;
  }
  Device->Flags |= DO_POWER_PAGABLE;
  Device->Flags &= ~DO_DEVICE_INITIALIZING;
  OsLog ("exynospep: device added (%ld processors registered so far)", mCpuCount);
  return STATUS_SUCCESS;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  NTSTATUS  Status;
  ULONG     Index;

  UNREFERENCED_PARAMETER (RegistryPath);

  for (Index = 0; Index <= IRP_MJ_MAXIMUM_FUNCTION; Index++) {
    DriverObject->MajorFunction[Index] = DispatchPass;
  }
  DriverObject->MajorFunction[IRP_MJ_PNP]   = DispatchPnp;
  DriverObject->MajorFunction[IRP_MJ_POWER] = DispatchPower;
  DriverObject->DriverExtension->AddDevice  = AddDevice;

  OsLogInit ();
  OsLog ("exynospep: DriverEntry (built " __DATE__ " " __TIME__ ")");

  mPep.Version                     = 2;
  mPep.Size                        = sizeof (mPep);
  mPep.AcceptDeviceNotification    = AcceptDeviceNotification;
  mPep.AcceptProcessorNotification = AcceptProcessorNotification;
  mKernel.Version = 2;
  mKernel.Size    = sizeof (mKernel);
  Status = PoFxRegisterPlugin (&mPep, &mKernel);
  OsLog ("exynospep: PoFxRegisterPlugin (PEP v2 %u bytes, kernel v2 %u bytes) - 0x%08lx", mPep.Size, mKernel.Size,
         Status);
  return Status;
}
