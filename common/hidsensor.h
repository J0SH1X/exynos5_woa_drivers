/** @file
  HID sensor plumbing shared by the v1awifi sensor drivers (bma255,
  bmg160, ak8963, cm3323).  Each driver is a KMDF lower filter under
  mshidkmdf.sys (the HID function driver) and presents one HID sensor
  collection; Windows' inbox HID sensor class driver
  (sensorshidclassdriver.inf, HID_DEVICE_UP:0020_U:00xx) sits on top and
  exposes it through the Sensor API.

  Header-only (one translation unit per driver, like oslog.h).  Include it
  after oslog.h, then:
    - build the report descriptor from HIDSENSOR_PROPERTIES () plus the
      sensor's input fields (HIDSENSOR_INPUT_STATE_EVENT () and data);
    - give the device context a HIDSENSOR member and call HidSensorInit ();
    - route EvtIoInternalDeviceControl to HidSensorInternalIoctl ();
    - complete input reports with HidSensorSendInput () (pended reads).

  Feature report (all sensors, report ID HIDSENSOR_REPORT_ID), in order:
    connection type, reporting state, power state, sensor state (1 byte
    each, selector index), report interval (ULONG, ms), change sensitivity
    (USHORT, in the sensor's data unit and exponent).
  The input report starts with sensor state and event (1 byte each), then
  the sensor's data fields.

  Usage values from the HID Usage Tables, Sensors page (0x20), as used by
  Microsoft's HID sensor samples for Windows 8.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef EXYNOS_HIDSENSOR_H_
#define EXYNOS_HIDSENSOR_H_

#include <hidport.h>

#define HIDSENSOR_REPORT_ID  1

//
// Selector indices (values of the array fields below).
//
#define HIDSENSOR_REPORTING_NONE       0
#define HIDSENSOR_REPORTING_ALL        1
#define HIDSENSOR_REPORTING_THRESHOLD  2
#define HIDSENSOR_REPORTING_NONE_WAKE  3
#define HIDSENSOR_REPORTING_ALL_WAKE   4
#define HIDSENSOR_REPORTING_THRESHOLD_WAKE 5

#define HIDSENSOR_POWER_UNDEFINED  0
#define HIDSENSOR_POWER_D0         1
#define HIDSENSOR_POWER_D1         2
#define HIDSENSOR_POWER_D2         3
#define HIDSENSOR_POWER_D3         4
#define HIDSENSOR_POWER_D4         5

#define HIDSENSOR_STATE_UNKNOWN       0
#define HIDSENSOR_STATE_READY         1
#define HIDSENSOR_STATE_NOT_AVAILABLE 2
#define HIDSENSOR_STATE_NO_DATA       3
#define HIDSENSOR_STATE_INITIALIZING  4
#define HIDSENSOR_STATE_ACCESS_DENIED 5
#define HIDSENSOR_STATE_ERROR         6

#define HIDSENSOR_EVENT_UNKNOWN        0
#define HIDSENSOR_EVENT_STATE_CHANGED  1
#define HIDSENSOR_EVENT_PROPERTY_CHANGED 2
#define HIDSENSOR_EVENT_DATA_UPDATED   3
#define HIDSENSOR_EVENT_POLL_RESPONSE  4
#define HIDSENSOR_EVENT_SENSITIVITY    5

//
// Descriptor building blocks.  Usages above 0xFF go in as 16-bit local
// usages (0x0A, lo, hi).
//
#define HIDSENSOR_U16(u)  0x0A, (UCHAR)((u) & 0xFF), (UCHAR)((u) >> 8)

#define HIDSENSOR_SELECTOR8(Field, Max, ...)                                  \
  HIDSENSOR_U16 (Field),                                                     \
  0x15, 0x00,                       /* Logical Minimum (0)            */      \
  0x25, (Max),                      /* Logical Maximum                */      \
  0x75, 0x08,                       /* Report Size (8)                */      \
  0x95, 0x01,                       /* Report Count (1)               */      \
  0xA1, 0x02,                       /* Collection (Logical)           */      \
  __VA_ARGS__,                                                               \
  0xC0                              /* End Collection                 */

#define HIDSENSOR_CONNECTION_TYPE(Main)                                      \
  HIDSENSOR_SELECTOR8 (0x0309, 2,                                            \
    HIDSENSOR_U16 (0x0830), HIDSENSOR_U16 (0x0831), HIDSENSOR_U16 (0x0832), \
    (Main), 0x00)

#define HIDSENSOR_REPORTING_STATE(Main)                                      \
  HIDSENSOR_SELECTOR8 (0x0316, 5,                                            \
    HIDSENSOR_U16 (0x0840), HIDSENSOR_U16 (0x0841), HIDSENSOR_U16 (0x0842), \
    HIDSENSOR_U16 (0x0843), HIDSENSOR_U16 (0x0844), HIDSENSOR_U16 (0x0845), \
    (Main), 0x00)

#define HIDSENSOR_POWER_STATE(Main)                                          \
  HIDSENSOR_SELECTOR8 (0x0319, 5,                                            \
    HIDSENSOR_U16 (0x0850), HIDSENSOR_U16 (0x0851), HIDSENSOR_U16 (0x0852), \
    HIDSENSOR_U16 (0x0853), HIDSENSOR_U16 (0x0854), HIDSENSOR_U16 (0x0855), \
    (Main), 0x00)

#define HIDSENSOR_SENSOR_STATE(Main)                                         \
  HIDSENSOR_SELECTOR8 (0x0201, 6,                                            \
    HIDSENSOR_U16 (0x0800), HIDSENSOR_U16 (0x0801), HIDSENSOR_U16 (0x0802), \
    HIDSENSOR_U16 (0x0803), HIDSENSOR_U16 (0x0804), HIDSENSOR_U16 (0x0805), \
    HIDSENSOR_U16 (0x0806), (Main), 0x00)

#define HIDSENSOR_EVENT(Main)                                                \
  HIDSENSOR_SELECTOR8 (0x0202, 5,                                            \
    HIDSENSOR_U16 (0x0810), HIDSENSOR_U16 (0x0811), HIDSENSOR_U16 (0x0812), \
    HIDSENSOR_U16 (0x0813), HIDSENSOR_U16 (0x0814), HIDSENSOR_U16 (0x0815), \
    (Main), 0x00)

//
// The feature report: SensitivityUsage is the data field's usage with the
// Change Sensitivity Absolute modifier (| 0x1000); Exponent is the HID unit
// exponent nibble (0x0E = -2, 0x0D = -3, ...).
//
#define HIDSENSOR_PROPERTIES(SensitivityUsage, Exponent)                     \
  0x85, HIDSENSOR_REPORT_ID,        /* Report ID                      */      \
  0x05, 0x20,                       /* Usage Page (Sensor)            */      \
  HIDSENSOR_CONNECTION_TYPE (0xB1),                                          \
  HIDSENSOR_REPORTING_STATE (0xB1),                                          \
  HIDSENSOR_POWER_STATE (0xB1),                                              \
  HIDSENSOR_SENSOR_STATE (0xB1),                                             \
  HIDSENSOR_U16 (0x030E),           /* Report Interval (ms)           */      \
  0x15, 0x00,                                                                \
  0x27, 0xFF, 0xFF, 0xFF, 0xFF,     /* Logical Maximum (0xFFFFFFFF)   */      \
  0x75, 0x20,                       /* Report Size (32)               */      \
  0x95, 0x01,                                                                \
  0x55, 0x00,                       /* Unit Exponent (0)              */      \
  0xB1, 0x02,                       /* Feature (Data,Var,Abs)         */      \
  HIDSENSOR_U16 (SensitivityUsage),                                          \
  0x15, 0x00,                                                                \
  0x26, 0xFF, 0xFF,                 /* Logical Maximum (0xFFFF)       */      \
  0x75, 0x10,                       /* Report Size (16)               */      \
  0x95, 0x01,                                                                \
  0x55, (Exponent),                                                          \
  0xB1, 0x02                        /* Feature (Data,Var,Abs)         */

//
// Start of the input report (same report ID): sensor state, event.
//
#define HIDSENSOR_INPUT_STATE_EVENT()                                        \
  0x05, 0x20,                       /* Usage Page (Sensor)            */      \
  HIDSENSOR_SENSOR_STATE (0x81),                                             \
  HIDSENSOR_EVENT (0x81)

//
// One signed 16-bit input data field.
//
#define HIDSENSOR_INPUT_S16(Usage, Exponent)                                 \
  HIDSENSOR_U16 (Usage),                                                     \
  0x16, 0x01, 0x80,                 /* Logical Minimum (-32767)       */      \
  0x26, 0xFF, 0x7F,                 /* Logical Maximum (32767)        */      \
  0x75, 0x10,                       /* Report Size (16)               */      \
  0x95, 0x01,                                                                \
  0x55, (Exponent),                                                          \
  0x81, 0x02                        /* Input (Data,Var,Abs)           */

//
// One unsigned 16-bit input data field.
//
#define HIDSENSOR_INPUT_U16(Usage, Exponent)                                 \
  HIDSENSOR_U16 (Usage),                                                     \
  0x15, 0x00,                                                                \
  0x26, 0xFF, 0xFF,                 /* Logical Maximum (0xFFFF)       */      \
  0x75, 0x10,                                                                \
  0x95, 0x01,                                                                \
  0x55, (Exponent),                                                          \
  0x81, 0x02

#include <pshpack1.h>
typedef struct {
  UCHAR   ReportId;
  UCHAR   ConnectionType;
  UCHAR   ReportingState;
  UCHAR   PowerState;
  UCHAR   SensorState;
  ULONG   IntervalMs;
  USHORT  Sensitivity;
} HIDSENSOR_FEATURE;

typedef struct {
  UCHAR   ReportId;
  UCHAR   SensorState;
  UCHAR   Event;
} HIDSENSOR_INPUT_HEADER;
#include <poppack.h>

typedef struct _HIDSENSOR HIDSENSOR;

//
// Called after Windows changed a property (interval, sensitivity,
// reporting or power state), at PASSIVE level.
//
typedef VOID HIDSENSOR_CONFIG_CHANGED (HIDSENSOR *Sensor);
//
// Fill a complete input report (header included) for a poll
// (IOCTL_HID_GET_INPUT_REPORT); return its length, 0 if none.
//
typedef ULONG HIDSENSOR_POLL (HIDSENSOR *Sensor, PVOID Report, ULONG Size);

struct _HIDSENSOR {
  const char                *Name;           // log prefix
  const UCHAR               *ReportDescriptor;
  USHORT                    ReportDescriptorSize;
  USHORT                    VendorId;
  USHORT                    ProductId;
  WDFQUEUE                  ReadQueue;       // pended IOCTL_HID_READ_REPORT
  HIDSENSOR_FEATURE         Feature;
  HIDSENSOR_CONFIG_CHANGED  *ConfigChanged;
  HIDSENSOR_POLL            *Poll;
  const WCHAR               *Manufacturer;   // HID strings (IOCTL_HID_GET_STRING)
  const WCHAR               *Product;
  ULONG                     Delivered;
  ULONG                     Dropped;
  ULONG                     IoctlsLogged;
  HID_DESCRIPTOR            HidDescriptor;
};

/**
  Defaults: integrated, no reporting until Windows asks, D0, ready.
**/
static __inline NTSTATUS
HidSensorInit (HIDSENSOR *Sensor, WDFDEVICE Device, ULONG DefaultIntervalMs, USHORT DefaultSensitivity)
{
  WDF_IO_QUEUE_CONFIG  QueueConfig;

  Sensor->Feature.ReportId       = HIDSENSOR_REPORT_ID;
  Sensor->Feature.ConnectionType = 0;                       // PC integrated
  Sensor->Feature.ReportingState = HIDSENSOR_REPORTING_NONE;
  Sensor->Feature.PowerState     = HIDSENSOR_POWER_D0;
  Sensor->Feature.SensorState    = HIDSENSOR_STATE_READY;
  Sensor->Feature.IntervalMs     = DefaultIntervalMs;
  Sensor->Feature.Sensitivity    = DefaultSensitivity;

  Sensor->HidDescriptor.bLength                         = sizeof (HID_DESCRIPTOR);
  Sensor->HidDescriptor.bDescriptorType                 = 0x21;
  Sensor->HidDescriptor.bcdHID                          = 0x0100;
  Sensor->HidDescriptor.bCountry                        = 0;
  Sensor->HidDescriptor.bNumDescriptors                 = 1;
  Sensor->HidDescriptor.DescriptorList[0].bReportType   = 0x22;
  Sensor->HidDescriptor.DescriptorList[0].wReportLength = Sensor->ReportDescriptorSize;

  WDF_IO_QUEUE_CONFIG_INIT (&QueueConfig, WdfIoQueueDispatchManual);
  return WdfIoQueueCreate (Device, &QueueConfig, WDF_NO_OBJECT_ATTRIBUTES, &Sensor->ReadQueue);
}

/**
  TRUE while Windows wants data (a reporting state other than "no events",
  and not powered down).
**/
static __inline BOOLEAN
HidSensorWanted (const HIDSENSOR *Sensor)
{
  return (Sensor->Feature.ReportingState != HIDSENSOR_REPORTING_NONE) &&
         (Sensor->Feature.ReportingState != HIDSENSOR_REPORTING_NONE_WAKE) &&
         (Sensor->Feature.PowerState != HIDSENSOR_POWER_D4);
}

static __inline NTSTATUS
HidSensorCopyToOutput (WDFREQUEST Request, const VOID *Data, size_t Length)
{
  WDFMEMORY  Memory;
  size_t     Size;
  NTSTATUS   Status;

  Status = WdfRequestRetrieveOutputMemory (Request, &Memory);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  WdfMemoryGetBuffer (Memory, &Size);
  if (Size < Length) {
    return STATUS_BUFFER_TOO_SMALL;
  }
  Status = WdfMemoryCopyFromBuffer (Memory, 0, (PVOID)Data, Length);
  if (NT_SUCCESS (Status)) {
    WdfRequestSetInformation (Request, Length);
  }
  return Status;
}

/**
  Complete one pended read with Report (header included).  No reader:
  dropped, the next report has the newer data.
**/
static __inline VOID
HidSensorSendInput (HIDSENSOR *Sensor, const VOID *Report, ULONG Length)
{
  WDFREQUEST  Request;
  NTSTATUS    Status;

  if (!NT_SUCCESS (WdfIoQueueRetrieveNextRequest (Sensor->ReadQueue, &Request))) {
    Sensor->Dropped++;
    return;
  }
  Status = HidSensorCopyToOutput (Request, Report, Length);
  WdfRequestComplete (Request, Status);
  if (NT_SUCCESS (Status) && (Sensor->Delivered++ == 0)) {
    OsLog ("%s: first report delivered", Sensor->Name);
  }
}

/**
  The input (Output = FALSE) or output buffer of a feature request; the
  caller takes the one that holds a whole report (report ID first).
**/
static __inline PUCHAR
HidSensorFeatureBuffer (WDFREQUEST Request, BOOLEAN Output, size_t *Size)
{
  WDFMEMORY  Memory;
  NTSTATUS   Status;

  Status = Output ? WdfRequestRetrieveOutputMemory (Request, &Memory) : WdfRequestRetrieveInputMemory (Request, &Memory);
  if (!NT_SUCCESS (Status)) {
    *Size = 0;
    return NULL;
  }
  return WdfMemoryGetBuffer (Memory, Size);
}

static __inline NTSTATUS
HidSensorSetFeature (HIDSENSOR *Sensor, WDFREQUEST Request)
{
  HIDSENSOR_FEATURE  New;
  PUCHAR             Buffer;
  size_t             Size;

  Buffer = HidSensorFeatureBuffer (Request, FALSE, &Size);
  if ((Buffer == NULL) || (Size < sizeof (New))) {
    Buffer = HidSensorFeatureBuffer (Request, TRUE, &Size);
  }
  if ((Buffer == NULL) || (Size < sizeof (New)) || (Buffer[0] != HIDSENSOR_REPORT_ID)) {
    OsLog ("%s: SET_FEATURE without a usable report (%Iu bytes)", Sensor->Name, Size);
    return STATUS_INVALID_PARAMETER;
  }
  RtlCopyMemory (&New, Buffer, sizeof (New));
  if ((New.ReportingState != Sensor->Feature.ReportingState) || (New.PowerState != Sensor->Feature.PowerState) ||
      (New.IntervalMs != Sensor->Feature.IntervalMs) || (New.Sensitivity != Sensor->Feature.Sensitivity)) {
    OsLog ("%s: reporting %u power %u interval %lu ms sensitivity %u", Sensor->Name, New.ReportingState,
           New.PowerState, New.IntervalMs, New.Sensitivity);
  }
  Sensor->Feature.ReportingState = New.ReportingState;
  Sensor->Feature.PowerState     = New.PowerState;
  Sensor->Feature.IntervalMs     = New.IntervalMs;
  Sensor->Feature.Sensitivity    = New.Sensitivity;
  if (Sensor->ConfigChanged != NULL) {
    Sensor->ConfigChanged (Sensor);
  }
  return STATUS_SUCCESS;
}

/**
  IOCTL_HID_GET_STRING / GET_INDEXED_STRING: under mshidkmdf the string ID
  (low word) comes in Type3InputBuffer, as in Microsoft's KMDF HID sample.
**/
static __inline NTSTATUS
HidSensorGetString (HIDSENSOR *Sensor, WDFREQUEST Request)
{
  WDF_REQUEST_PARAMETERS  Params;
  const WCHAR             *String;
  ULONG                   Id;
  size_t                  Length;

  WDF_REQUEST_PARAMETERS_INIT (&Params);
  WdfRequestGetParameters (Request, &Params);
  Id = PtrToUlong (Params.Parameters.DeviceIoControl.Type3InputBuffer) & 0xFFFF;
  switch (Id) {
    case HID_STRING_ID_IMANUFACTURER: String = Sensor->Manufacturer; break;
    case HID_STRING_ID_IPRODUCT:      String = Sensor->Product;      break;
    case HID_STRING_ID_ISERIALNUMBER: String = L"0001";              break;
    default:                          String = NULL;                 break;
  }
  if (String == NULL) {
    return STATUS_INVALID_PARAMETER;
  }
  for (Length = 0; String[Length] != 0; Length++) {
  }
  return HidSensorCopyToOutput (Request, String, (Length + 1) * sizeof (WCHAR));
}

/**
  The IOCTL_HID_* requests mshidkmdf passes down; completes the request,
  except a read, which waits in ReadQueue for HidSensorSendInput ().
**/
static __inline VOID
HidSensorInternalIoctl (HIDSENSOR *Sensor, WDFREQUEST Request, ULONG IoControlCode)
{
  HID_DEVICE_ATTRIBUTES  Attributes;
  UCHAR                  Report[64];
  ULONG                  Length;
  NTSTATUS               Status;

  switch (IoControlCode) {
    case IOCTL_HID_GET_DEVICE_DESCRIPTOR:
      Status = HidSensorCopyToOutput (Request, &Sensor->HidDescriptor, sizeof (Sensor->HidDescriptor));
      break;

    case IOCTL_HID_GET_REPORT_DESCRIPTOR:
      Status = HidSensorCopyToOutput (Request, Sensor->ReportDescriptor, Sensor->ReportDescriptorSize);
      break;

    case IOCTL_HID_GET_DEVICE_ATTRIBUTES:
      RtlZeroMemory (&Attributes, sizeof (Attributes));
      Attributes.Size          = sizeof (Attributes);
      Attributes.VendorID      = Sensor->VendorId;
      Attributes.ProductID     = Sensor->ProductId;
      Attributes.VersionNumber = 0x0100;
      Status = HidSensorCopyToOutput (Request, &Attributes, sizeof (Attributes));
      break;

    case IOCTL_HID_READ_REPORT:
      Status = WdfRequestForwardToIoQueue (Request, Sensor->ReadQueue);
      if (NT_SUCCESS (Status)) {
        if (Sensor->IoctlsLogged < 40) {
          Sensor->IoctlsLogged++;
          OsLog ("%s: HID read pended", Sensor->Name);
        }
        return;
      }
      break;

    case IOCTL_HID_GET_FEATURE:
      Status = HidSensorCopyToOutput (Request, &Sensor->Feature, sizeof (Sensor->Feature));
      if (!NT_SUCCESS (Status) && (Sensor->IoctlsLogged < 40)) {
        WDFMEMORY  Out;
        size_t     OutSize = 0;
        if (NT_SUCCESS (WdfRequestRetrieveOutputMemory (Request, &Out))) {
          WdfMemoryGetBuffer (Out, &OutSize);
        }
        OsLog ("%s: GET_FEATURE output buffer %Iu bytes, report is %Iu", Sensor->Name, OutSize, sizeof (Sensor->Feature));
      }
      break;

    case IOCTL_HID_SET_FEATURE:
      Status = HidSensorSetFeature (Sensor, Request);
      break;

    case IOCTL_HID_GET_INPUT_REPORT:
      Length = (Sensor->Poll != NULL) ? Sensor->Poll (Sensor, Report, sizeof (Report)) : 0;
      Status = (Length != 0) ? HidSensorCopyToOutput (Request, Report, Length) : STATUS_DEVICE_NOT_READY;
      break;

    case IOCTL_HID_GET_STRING:
    case IOCTL_HID_GET_INDEXED_STRING:
      Status = HidSensorGetString (Sensor, Request);
      break;

    case IOCTL_HID_ACTIVATE_DEVICE:
    case IOCTL_HID_DEACTIVATE_DEVICE:
      Status = STATUS_SUCCESS;
      break;

    default:
      Status = STATUS_NOT_SUPPORTED;
      break;
  }
  if (Sensor->IoctlsLogged < 40) {
    Sensor->IoctlsLogged++;
    OsLog ("%s: HID ioctl 0x%08lx - 0x%08lx", Sensor->Name, IoControlCode, Status);
  }
  WdfRequestComplete (Request, Status);
}

#endif // EXYNOS_HIDSENSOR_H_
