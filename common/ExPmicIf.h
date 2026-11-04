/** @file
  Direct-call interface of exynospmic (MAX77802 PMIC on HS-I2C3, 0x09).

  The DSDT I2cSerialBus connection to the PMIC is exclusive, so exynospmic
  is its only owner; drivers that need a rail (camera LDO18/19/24, GPU
  BUCK4, key backlight LDO23, later CPU DVFS BUCK2) find exynospmic by its
  device interface (GUID_EXYNOS_PMIC_INTERFACE), open it and ask for this
  table with IRP_MN_QUERY_INTERFACE (InterfaceType = the same GUID).
  ExPmicConnect () below does that; call it at PASSIVE_LEVEL and retry
  later while it returns FALSE (exynospmic may start after the caller).

  Calls are PASSIVE_LEVEL; exynospmic serializes them.  Register numbers
  are the MAX77802's (LDOnCTRL1 = 0x60 + n - 1, BUCK4 DVS 0x37.., ...).

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef __EX_PMIC_IF_H__
#define __EX_PMIC_IF_H__

// {6b7b7f1e-5420-4c5a-9b1d-3e2a0a6f7d02}
DEFINE_GUID (GUID_EXYNOS_PMIC_INTERFACE, 0x6b7b7f1e, 0x5420, 0x4c5a, 0x9b, 0x1d, 0x3e, 0x2a, 0x0a, 0x6f, 0x7d, 0x02);

#define EX_PMIC_IF_VERSION  1

typedef NTSTATUS (*EX_PMIC_READ)(PVOID Context, UCHAR Reg, PUCHAR Value);
typedef NTSTATUS (*EX_PMIC_WRITE)(PVOID Context, UCHAR Reg, UCHAR Value);
// read-modify-write under the driver's lock: Reg = (Reg & ~Mask) | (Value & Mask)
typedef NTSTATUS (*EX_PMIC_UPDATE)(PVOID Context, UCHAR Reg, UCHAR Mask, UCHAR Value);

typedef struct {
  INTERFACE       Header;    // Size, Version (EX_PMIC_IF_VERSION), Context, Reference / Dereference
  EX_PMIC_READ    Read;
  EX_PMIC_WRITE   Write;
  EX_PMIC_UPDATE  Update;
} EX_PMIC_INTERFACE;

//
// Consumer side.  Fills *If and returns TRUE once exynospmic is there.
//
static __inline BOOLEAN
ExPmicConnect (EX_PMIC_INTERFACE *If)
{
  PWSTR               Links = NULL;
  UNICODE_STRING      Name;
  PFILE_OBJECT        File;
  PDEVICE_OBJECT      Device;
  PDEVICE_OBJECT      Top;
  KEVENT              Event;
  IO_STATUS_BLOCK     Iosb;
  PIRP                Irp;
  PIO_STACK_LOCATION  Stack;
  NTSTATUS            Status;

  if (KeGetCurrentIrql () != PASSIVE_LEVEL) {
    return FALSE;
  }
  Status = IoGetDeviceInterfaces (&GUID_EXYNOS_PMIC_INTERFACE, NULL, 0, &Links);
  if (!NT_SUCCESS (Status) || (Links == NULL) || (Links[0] == 0)) {
    if (Links != NULL) {
      ExFreePool (Links);
    }
    return FALSE;
  }
  RtlInitUnicodeString (&Name, Links);
  Status = IoGetDeviceObjectPointer (&Name, FILE_READ_DATA, &File, &Device);
  ExFreePool (Links);
  if (!NT_SUCCESS (Status)) {
    return FALSE;
  }
  RtlZeroMemory (If, sizeof (*If));
  Top = IoGetAttachedDeviceReference (Device);
  KeInitializeEvent (&Event, NotificationEvent, FALSE);
  Irp = IoBuildSynchronousFsdRequest (IRP_MJ_PNP, Top, NULL, 0, NULL, &Event, &Iosb);
  if (Irp != NULL) {
    Irp->IoStatus.Status = STATUS_NOT_SUPPORTED;
    Stack = IoGetNextIrpStackLocation (Irp);
    Stack->MinorFunction                                   = IRP_MN_QUERY_INTERFACE;
    Stack->Parameters.QueryInterface.InterfaceType         = &GUID_EXYNOS_PMIC_INTERFACE;
    Stack->Parameters.QueryInterface.Size                  = sizeof (*If);
    Stack->Parameters.QueryInterface.Version               = EX_PMIC_IF_VERSION;
    Stack->Parameters.QueryInterface.Interface             = (PINTERFACE)If;
    Stack->Parameters.QueryInterface.InterfaceSpecificData = NULL;
    Status = IoCallDriver (Top, Irp);
    if (Status == STATUS_PENDING) {
      KeWaitForSingleObject (&Event, Executive, KernelMode, FALSE, NULL);
      Status = Iosb.Status;
    }
  } else {
    Status = STATUS_INSUFFICIENT_RESOURCES;
  }
  ObDereferenceObject (Top);
  ObDereferenceObject (File);
  return NT_SUCCESS (Status) && (If->Write != NULL);
}

#endif // __EX_PMIC_IF_H__
