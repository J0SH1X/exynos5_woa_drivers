/** @file
  exynosaudio WaveRT miniport (48 kHz / 16-bit / stereo PCM both ways):
    pin 0  render (sink, in)     -> pin 1  bridge to the topology's speakers
    pin 2  bridge from the topology's microphone -> pin 3  capture (sink, out)
  Render buffers are physically contiguous so I2S0's IDMA can loop over
  them (position: IDMA); capture buffers loop through system DMA (ADMA0 via
  halextpl330; position: ReadDmaCounter).

  Pause keeps no hardware position: RUN after PAUSE restarts IDMA at the
  buffer start (the engine normally stops the stream instead of pausing).

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include "exynosaudio.h"

//
// ---------------------------------------------------------------------------
// Filter description.
// ---------------------------------------------------------------------------
//

static KSDATARANGE_AUDIO  mStreamRanges[] = {
  {
    {
      sizeof (KSDATARANGE_AUDIO), 0, 0, 0,
      STATICGUIDOF (KSDATAFORMAT_TYPE_AUDIO),
      STATICGUIDOF (KSDATAFORMAT_SUBTYPE_PCM),
      STATICGUIDOF (KSDATAFORMAT_SPECIFIER_WAVEFORMATEX)
    },
    AUDIO_CHANNELS, AUDIO_BITS, AUDIO_BITS, AUDIO_RATE, AUDIO_RATE
  }
};

static PKSDATARANGE  mStreamRangePointers[] = { PKSDATARANGE (&mStreamRanges[0]) };

static KSDATARANGE  mBridgeRanges[] = {
  {
    sizeof (KSDATARANGE), 0, 0, 0,
    STATICGUIDOF (KSDATAFORMAT_TYPE_AUDIO),
    STATICGUIDOF (KSDATAFORMAT_SUBTYPE_ANALOG),
    STATICGUIDOF (KSDATAFORMAT_SPECIFIER_NONE)
  }
};

static PKSDATARANGE  mBridgeRangePointers[] = { &mBridgeRanges[0] };

static PCPIN_DESCRIPTOR  mWavePins[] = {
  {
    1, 1, 0, NULL,
    {
      0, NULL, 0, NULL,
      SIZEOF_ARRAY (mStreamRangePointers), mStreamRangePointers,
      KSPIN_DATAFLOW_IN, KSPIN_COMMUNICATION_SINK,
      &KSCATEGORY_AUDIO, NULL, 0
    }
  },
  {
    0, 0, 0, NULL,
    {
      0, NULL, 0, NULL,
      SIZEOF_ARRAY (mBridgeRangePointers), mBridgeRangePointers,
      KSPIN_DATAFLOW_OUT, KSPIN_COMMUNICATION_NONE,
      &KSCATEGORY_AUDIO, NULL, 0
    }
  },
  {
    0, 0, 0, NULL,
    {
      0, NULL, 0, NULL,
      SIZEOF_ARRAY (mBridgeRangePointers), mBridgeRangePointers,
      KSPIN_DATAFLOW_IN, KSPIN_COMMUNICATION_NONE,
      &KSCATEGORY_AUDIO, NULL, 0
    }
  },
  {
    1, 1, 0, NULL,
    {
      0, NULL, 0, NULL,
      SIZEOF_ARRAY (mStreamRangePointers), mStreamRangePointers,
      KSPIN_DATAFLOW_OUT, KSPIN_COMMUNICATION_SINK,
      &KSCATEGORY_AUDIO, NULL, 0
    }
  },
};

#define WAVE_PIN_RENDER   0
#define WAVE_PIN_CAPTURE  3

static PCCONNECTION_DESCRIPTOR  mWaveConnections[] = {
  { PCFILTER_NODE, 0, PCFILTER_NODE, 1 },
  { PCFILTER_NODE, 2, PCFILTER_NODE, 3 },
};

static GUID  mWaveCategories[] = {
  STATICGUIDOF (KSCATEGORY_AUDIO),
  STATICGUIDOF (KSCATEGORY_RENDER),
  STATICGUIDOF (KSCATEGORY_CAPTURE),
  STATICGUIDOF (KSCATEGORY_REALTIME),
};

static PCFILTER_DESCRIPTOR  mWaveFilter = {
  0, NULL,
  sizeof (PCPIN_DESCRIPTOR), SIZEOF_ARRAY (mWavePins), mWavePins,
  sizeof (PCNODE_DESCRIPTOR), 0, NULL,
  SIZEOF_ARRAY (mWaveConnections), mWaveConnections,
  SIZEOF_ARRAY (mWaveCategories), mWaveCategories
};

static BOOLEAN
FormatSupported (PKSDATAFORMAT Format)
{
  PWAVEFORMATEX  Wave;

  if ((Format == NULL) || (Format->FormatSize < sizeof (KSDATAFORMAT_WAVEFORMATEX)) ||
      !IsEqualGUIDAligned (Format->MajorFormat, KSDATAFORMAT_TYPE_AUDIO) ||
      !IsEqualGUIDAligned (Format->Specifier, KSDATAFORMAT_SPECIFIER_WAVEFORMATEX))
  {
    return FALSE;
  }
  Wave = &((PKSDATAFORMAT_WAVEFORMATEX)Format)->WaveFormatEx;
  if ((Wave->wFormatTag != WAVE_FORMAT_PCM) && (Wave->wFormatTag != WAVE_FORMAT_EXTENSIBLE)) {
    return FALSE;
  }
  return (Wave->nChannels == AUDIO_CHANNELS) && (Wave->nSamplesPerSec == AUDIO_RATE) &&
         (Wave->wBitsPerSample == AUDIO_BITS);
}

//
// ---------------------------------------------------------------------------
// Classes.
// ---------------------------------------------------------------------------
//

class CMiniportWaveRT;

class CMiniportWaveRTStream : public IMiniportWaveRTStream, public CUnknown
{
public:
  DECLARE_STD_UNKNOWN ();
  DEFINE_STD_CONSTRUCTOR (CMiniportWaveRTStream);
  ~CMiniportWaveRTStream ();
  IMP_IMiniportWaveRTStream;

  NTSTATUS Init (CMiniportWaveRT *Miniport, PPORTWAVERTSTREAM PortStream, BOOLEAN Capture);

private:
  BOOLEAN            m_Capture;
  CMiniportWaveRT    *m_Miniport;
  PPORTWAVERTSTREAM  m_PortStream;
  PMDL               m_Mdl;
  ULONG              m_Physical;
  ULONG              m_Bytes;
  KSSTATE            m_State;
  BOOLEAN            m_Running;
};

class CMiniportWaveRT : public IMiniportWaveRT, public CUnknown
{
public:
  DECLARE_STD_UNKNOWN ();
  DEFINE_STD_CONSTRUCTOR (CMiniportWaveRT);
  ~CMiniportWaveRT ();
  IMP_IMiniportWaveRT;

  BOOLEAN  m_RenderOpen;
  BOOLEAN  m_CaptureOpen;

private:
  PPORTWAVERT  m_Port;
};

//
// ---------------------------------------------------------------------------
// Miniport.
// ---------------------------------------------------------------------------
//

NTSTATUS
CreateWaveMiniport (PUNKNOWN *Unknown, REFCLSID ClassId, PUNKNOWN UnknownOuter, POOL_TYPE PoolType)
{
  UNREFERENCED_PARAMETER (ClassId);
  STD_CREATE_BODY_ (CMiniportWaveRT, Unknown, UnknownOuter, PoolType, PMINIPORTWAVERT);
}

CMiniportWaveRT::~CMiniportWaveRT ()
{
  if (m_Port != NULL) {
    m_Port->Release ();
  }
}

STDMETHODIMP_(NTSTATUS)
CMiniportWaveRT::NonDelegatingQueryInterface (_In_ REFIID Interface, _COM_Outptr_ PVOID *Object)
{
  if (IsEqualGUIDAligned (Interface, IID_IUnknown)) {
    *Object = PVOID (PUNKNOWN (PMINIPORTWAVERT (this)));
  } else if (IsEqualGUIDAligned (Interface, IID_IMiniport)) {
    *Object = PVOID (PMINIPORT (this));
  } else if (IsEqualGUIDAligned (Interface, IID_IMiniportWaveRT)) {
    *Object = PVOID (PMINIPORTWAVERT (this));
  } else {
    *Object = NULL;
    return STATUS_INVALID_PARAMETER;
  }
  PUNKNOWN (*Object)->AddRef ();
  return STATUS_SUCCESS;
}

STDMETHODIMP_(NTSTATUS)
CMiniportWaveRT::Init (_In_ PUNKNOWN UnknownAdapter, _In_ PRESOURCELIST ResourceList, _In_ PPORTWAVERT Port)
{
  UNREFERENCED_PARAMETER (UnknownAdapter);
  UNREFERENCED_PARAMETER (ResourceList);

  m_Port = Port;
  m_Port->AddRef ();
  return STATUS_SUCCESS;
}

STDMETHODIMP_(NTSTATUS)
CMiniportWaveRT::GetDescription (_Out_ PPCFILTER_DESCRIPTOR *Description)
{
  *Description = &mWaveFilter;
  return STATUS_SUCCESS;
}

STDMETHODIMP_(NTSTATUS)
CMiniportWaveRT::DataRangeIntersection (_In_ ULONG PinId, _In_ PKSDATARANGE DataRange,
                                        _In_ PKSDATARANGE MatchingDataRange, _In_ ULONG OutputBufferLength,
                                        PVOID ResultantFormat, _Out_ PULONG ResultantFormatLength)
{
  UNREFERENCED_PARAMETER (PinId);
  UNREFERENCED_PARAMETER (DataRange);
  UNREFERENCED_PARAMETER (MatchingDataRange);
  UNREFERENCED_PARAMETER (OutputBufferLength);
  UNREFERENCED_PARAMETER (ResultantFormat);
  UNREFERENCED_PARAMETER (ResultantFormatLength);
  return STATUS_NOT_IMPLEMENTED;          // PortCls' default intersection
}

STDMETHODIMP_(NTSTATUS)
CMiniportWaveRT::GetDeviceDescription (_Out_ PDEVICE_DESCRIPTION DeviceDescription)
{
  RtlZeroMemory (DeviceDescription, sizeof (DEVICE_DESCRIPTION));
  DeviceDescription->Version           = DEVICE_DESCRIPTION_VERSION;
  DeviceDescription->Master            = TRUE;
  DeviceDescription->ScatterGather     = FALSE;
  DeviceDescription->Dma32BitAddresses = TRUE;
  DeviceDescription->InterfaceType     = Internal;
  DeviceDescription->MaximumLength     = AUDIO_MAX_BUFFER;
  return STATUS_SUCCESS;
}

STDMETHODIMP_(NTSTATUS)
CMiniportWaveRT::NewStream (_Out_ PMINIPORTWAVERTSTREAM *Stream, _In_ PPORTWAVERTSTREAM PortStream, _In_ ULONG Pin,
                            _In_ BOOLEAN Capture, _In_ PKSDATAFORMAT DataFormat)
{
  CMiniportWaveRTStream  *New;
  NTSTATUS               Status;

  *Stream = NULL;
  if ((Pin != (Capture ? WAVE_PIN_CAPTURE : WAVE_PIN_RENDER)) || !FormatSupported (DataFormat)) {
    return STATUS_INVALID_PARAMETER;
  }
  if (Capture ? m_CaptureOpen : m_RenderOpen) {
    return STATUS_INSUFFICIENT_RESOURCES;
  }
  New = new (NonPagedPool, AUDIO_TAG) CMiniportWaveRTStream (NULL);
  if (New == NULL) {
    return STATUS_INSUFFICIENT_RESOURCES;
  }
  New->AddRef ();
  Status = New->Init (this, PortStream, Capture);
  if (NT_SUCCESS (Status)) {
    *Stream = PMINIPORTWAVERTSTREAM (New);
    if (Capture) {
      m_CaptureOpen = TRUE;
    } else {
      m_RenderOpen = TRUE;
    }
    gAudio.Streams++;
  } else {
    New->Release ();
  }
  return Status;
}

//
// ---------------------------------------------------------------------------
// Stream.
// ---------------------------------------------------------------------------
//

NTSTATUS
CMiniportWaveRTStream::Init (CMiniportWaveRT *Miniport, PPORTWAVERTSTREAM PortStream, BOOLEAN Capture)
{
  m_Capture    = Capture;
  m_Miniport   = Miniport;
  m_Miniport->AddRef ();
  m_PortStream = PortStream;
  m_PortStream->AddRef ();
  m_State      = KSSTATE_STOP;
  return STATUS_SUCCESS;
}

CMiniportWaveRTStream::~CMiniportWaveRTStream ()
{
  if (m_Running) {
    if (m_Capture) {
      AudioCaptureStop ();
    } else {
      AudioRenderStop ();
    }
  }
  if (m_Mdl != NULL) {
    m_PortStream->FreePagesFromMdl (m_Mdl);
  }
  if (m_PortStream != NULL) {
    m_PortStream->Release ();
  }
  if (m_Miniport != NULL) {
    if (m_Capture) {
      m_Miniport->m_CaptureOpen = FALSE;
    } else {
      m_Miniport->m_RenderOpen = FALSE;
    }
    m_Miniport->Release ();
  }
}

STDMETHODIMP_(NTSTATUS)
CMiniportWaveRTStream::NonDelegatingQueryInterface (_In_ REFIID Interface, _COM_Outptr_ PVOID *Object)
{
  if (IsEqualGUIDAligned (Interface, IID_IUnknown)) {
    *Object = PVOID (PUNKNOWN (PMINIPORTWAVERTSTREAM (this)));
  } else if (IsEqualGUIDAligned (Interface, IID_IMiniportWaveRTStream)) {
    *Object = PVOID (PMINIPORTWAVERTSTREAM (this));
  } else {
    *Object = NULL;
    return STATUS_INVALID_PARAMETER;
  }
  PUNKNOWN (*Object)->AddRef ();
  return STATUS_SUCCESS;
}

STDMETHODIMP_(NTSTATUS)
CMiniportWaveRTStream::SetFormat (_In_ PKSDATAFORMAT DataFormat)
{
  return FormatSupported (DataFormat) ? STATUS_SUCCESS : STATUS_INVALID_PARAMETER;
}

/**
  Physically contiguous, below 4 GiB, a whole number of frames, at most
  what IDMA can count; write-combined for the engine's mapping (IDMA does
  not snoop the CPU caches).
**/
STDMETHODIMP_(NTSTATUS)
CMiniportWaveRTStream::AllocateAudioBuffer (_In_ ULONG RequestedSize, _Out_ PMDL *AudioBufferMdl,
                                            _Out_ ULONG *ActualSize, _Out_ ULONG *OffsetFromFirstPage,
                                            _Out_ MEMORY_CACHING_TYPE *CacheType)
{
  PHYSICAL_ADDRESS  Low;
  PHYSICAL_ADDRESS  High;
  ULONG             Bytes;

  if (m_Mdl != NULL) {
    return STATUS_DEVICE_BUSY;
  }
  Bytes = min (RequestedSize, (ULONG)AUDIO_MAX_BUFFER) & ~(ULONG)3;
  if (Bytes < 1024) {
    Bytes = 1024;
  }
  Low.QuadPart  = 0;
  High.QuadPart = 0xFFFFFFFF;
  m_Mdl = m_PortStream->AllocateContiguousPagesForMdl (Low, High, Bytes);
  if (m_Mdl == NULL) {
    OsLog ("exynosaudio: no contiguous buffer of %lu bytes", Bytes);
    return STATUS_INSUFFICIENT_RESOURCES;
  }
  m_Physical           = m_PortStream->GetPhysicalPageAddress (m_Mdl, 0).LowPart;
  m_Bytes              = Bytes;
  *AudioBufferMdl      = m_Mdl;
  *ActualSize          = Bytes;
  *OffsetFromFirstPage = 0;
  *CacheType           = MmWriteCombined;
  OsLog ("exynosaudio: buffer %lu bytes (requested %lu) at 0x%08lx", Bytes, RequestedSize, m_Physical);
  return STATUS_SUCCESS;
}

STDMETHODIMP_(VOID)
CMiniportWaveRTStream::FreeAudioBuffer (_In_opt_ PMDL AudioBufferMdl, _In_ ULONG BufferSize)
{
  UNREFERENCED_PARAMETER (BufferSize);

  if (m_Running) {
    if (m_Capture) {
      AudioCaptureStop ();
    } else {
      AudioRenderStop ();
    }
    m_Running = FALSE;
  }
  if ((AudioBufferMdl != NULL) && (AudioBufferMdl == m_Mdl)) {
    m_PortStream->FreePagesFromMdl (m_Mdl);
    m_Mdl   = NULL;
    m_Bytes = 0;
  }
}

STDMETHODIMP_(VOID)
CMiniportWaveRTStream::GetHWLatency (_Out_ KSRTAUDIO_HWLATENCY *Latency)
{
  Latency->FifoSize     = 64 * 4;          // I2S secondary TX FIFO
  Latency->ChipsetDelay = 0;
  Latency->CodecDelay   = 0;
}

STDMETHODIMP_(NTSTATUS)
CMiniportWaveRTStream::GetPositionRegister (_Out_ KSRTAUDIO_HWREGISTER *Register)
{
  UNREFERENCED_PARAMETER (Register);
  return STATUS_NOT_SUPPORTED;
}

STDMETHODIMP_(NTSTATUS)
CMiniportWaveRTStream::GetClockRegister (_Out_ KSRTAUDIO_HWREGISTER *Register)
{
  UNREFERENCED_PARAMETER (Register);
  return STATUS_NOT_SUPPORTED;
}

STDMETHODIMP_(NTSTATUS)
CMiniportWaveRTStream::SetState (_In_ KSSTATE State)
{
  NTSTATUS  Status;

  Status = STATUS_SUCCESS;
  if ((State == KSSTATE_RUN) && !m_Running) {
    if (m_Mdl == NULL) {
      return STATUS_INVALID_DEVICE_STATE;
    }
    Status    = m_Capture ? AudioCaptureStart (m_Mdl, m_Bytes) : AudioRenderStart (m_Physical, m_Bytes);
    m_Running = NT_SUCCESS (Status);
  } else if ((State != KSSTATE_RUN) && m_Running) {
    if (m_Capture) {
      AudioCaptureStop ();
    } else {
      AudioRenderStop ();
    }
    m_Running = FALSE;
  }
  if (NT_SUCCESS (Status)) {
    m_State = State;
  }
  return Status;
}

STDMETHODIMP_(NTSTATUS)
CMiniportWaveRTStream::GetPosition (_Out_ PKSAUDIO_POSITION Position)
{
  Position->PlayOffset  = !m_Running ? 0 : (m_Capture ? AudioCapturePosition () : AudioRenderPosition ());
  Position->WriteOffset = Position->PlayOffset;
  return STATUS_SUCCESS;
}
