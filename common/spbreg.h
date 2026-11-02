/** @file
  Register access to an I2C chip through SpbCx, for KMDF drivers whose
  device has one I2cSerialBus connection (the resource hub turns it into a
  path to open).  8-bit register addresses; reads are one SPB sequence
  (write the register, repeated start, read).

  Header-only, include after oslog.h.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef EXYNOS_SPBREG_H_
#define EXYNOS_SPBREG_H_

#include <spb.h>
#ifndef RESHUB_USE_HELPER_ROUTINES
#define RESHUB_USE_HELPER_ROUTINES
#endif
#include <reshub.h>

/**
  Find the first I2C connection in the translated resources and open it.
**/
static __inline NTSTATUS
SpbOpenI2c (WDFDEVICE Device, WDFCMRESLIST ResourcesTranslated, const char *Name, WDFIOTARGET *Target)
{
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  WDF_IO_TARGET_OPEN_PARAMS        Open;
  ULONG                            Index;
  ULONG                            Low = 0;
  ULONG                            High = 0;
  BOOLEAN                          Found = FALSE;
  NTSTATUS                         Status;

  DECLARE_UNICODE_STRING_SIZE (Path, RESOURCE_HUB_PATH_SIZE);

  for (Index = 0; (Index < WdfCmResourceListGetCount (ResourcesTranslated)) && !Found; Index++) {
    Desc = WdfCmResourceListGetDescriptor (ResourcesTranslated, Index);
    if ((Desc->Type == CmResourceTypeConnection) && (Desc->u.Connection.Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL) &&
        (Desc->u.Connection.Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C)) {
      Low   = Desc->u.Connection.IdLowPart;
      High  = Desc->u.Connection.IdHighPart;
      Found = TRUE;
    }
  }
  if (!Found) {
    OsLog ("%s: no I2C connection resource", Name);
    return STATUS_NOT_FOUND;
  }
  Status = RESOURCE_HUB_CREATE_PATH_FROM_ID (&Path, Low, High);
  if (NT_SUCCESS (Status)) {
    Status = WdfIoTargetCreate (Device, WDF_NO_OBJECT_ATTRIBUTES, Target);
  }
  if (NT_SUCCESS (Status)) {
    WDF_IO_TARGET_OPEN_PARAMS_INIT_OPEN_BY_NAME (&Open, &Path, GENERIC_READ | GENERIC_WRITE);
    Status = WdfIoTargetOpen (*Target, &Open);
  }
  OsLog ("%s: I2C connection 0x%08lx opened - 0x%08lx", Name, Low, Status);
  return Status;
}

static __inline NTSTATUS
SpbRegRead (WDFIOTARGET Target, UCHAR Reg, PVOID Buffer, ULONG Length)
{
  SPB_TRANSFER_LIST_AND_ENTRIES (2) Seq;
  WDF_MEMORY_DESCRIPTOR          Input;
  ULONG_PTR                      Bytes;

  SPB_TRANSFER_LIST_INIT (&Seq.List, 2);
  Seq.List.Transfers[0] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE (SpbTransferDirectionToDevice, 0, &Reg, 1);
  Seq.List.Transfers[1] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE (SpbTransferDirectionFromDevice, 0, Buffer, Length);
  WDF_MEMORY_DESCRIPTOR_INIT_BUFFER (&Input, &Seq, sizeof (Seq));
  return WdfIoTargetSendIoctlSynchronously (Target, NULL, IOCTL_SPB_EXECUTE_SEQUENCE, &Input, NULL, NULL, &Bytes);
}

/**
  Write Length bytes of Data after the register address (Length <= 4).
**/
static __inline NTSTATUS
SpbRegWriteBytes (WDFIOTARGET Target, UCHAR Reg, const UCHAR *Data, ULONG Length)
{
  UCHAR                  Buf[5];
  WDF_MEMORY_DESCRIPTOR  Input;
  ULONG_PTR              Bytes;
  ULONG                  Index;

  if (Length > 4) {
    return STATUS_INVALID_PARAMETER;
  }
  Buf[0] = Reg;
  for (Index = 0; Index < Length; Index++) {
    Buf[1 + Index] = Data[Index];
  }
  WDF_MEMORY_DESCRIPTOR_INIT_BUFFER (&Input, Buf, 1 + Length);
  return WdfIoTargetSendWriteSynchronously (Target, NULL, &Input, NULL, NULL, &Bytes);
}

static __inline NTSTATUS
SpbRegWrite (WDFIOTARGET Target, UCHAR Reg, UCHAR Value)
{
  return SpbRegWriteBytes (Target, Reg, &Value, 1);
}

static __inline VOID
SpbSleepMs (ULONG Ms)
{
  LARGE_INTEGER  Delay;

  Delay.QuadPart = -10000LL * Ms;
  KeDelayExecutionThread (KernelMode, FALSE, &Delay);
}

#endif // EXYNOS_SPBREG_H_
