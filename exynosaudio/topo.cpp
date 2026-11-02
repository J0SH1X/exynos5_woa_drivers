/** @file
  exynosaudio topology miniport: the wave filter's bridge pin in, the
  speakers out (KSNODETYPE_SPEAKER, which makes the endpoint "Speakers"),
  through a volume node and a mute node that drive the codec's speaker
  DACs (Windows then uses this hardware volume for the endpoint); and the
  microphone (KSNODETYPE_MICROPHONE: the "Microphone" endpoint) out to the
  wave filter's capture side.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include "exynosaudio.h"

static KSDATARANGE  mTopoBridgeRanges[] = {
  {
    sizeof (KSDATARANGE), 0, 0, 0,
    STATICGUIDOF (KSDATAFORMAT_TYPE_AUDIO),
    STATICGUIDOF (KSDATAFORMAT_SUBTYPE_ANALOG),
    STATICGUIDOF (KSDATAFORMAT_SPECIFIER_NONE)
  }
};

static PKSDATARANGE  mTopoBridgeRangePointers[] = { &mTopoBridgeRanges[0] };

static PCPIN_DESCRIPTOR  mTopoPins[] = {
  {
    0, 0, 0, NULL,
    {
      0, NULL, 0, NULL,
      SIZEOF_ARRAY (mTopoBridgeRangePointers), mTopoBridgeRangePointers,
      KSPIN_DATAFLOW_IN, KSPIN_COMMUNICATION_NONE,
      &KSCATEGORY_AUDIO, NULL, 0
    }
  },
  {
    0, 0, 0, NULL,
    {
      0, NULL, 0, NULL,
      SIZEOF_ARRAY (mTopoBridgeRangePointers), mTopoBridgeRangePointers,
      KSPIN_DATAFLOW_OUT, KSPIN_COMMUNICATION_NONE,
      &KSNODETYPE_SPEAKER, NULL, 0
    }
  },
  {
    0, 0, 0, NULL,                       // 2: the microphone (main, or the headset's)
    {
      0, NULL, 0, NULL,
      SIZEOF_ARRAY (mTopoBridgeRangePointers), mTopoBridgeRangePointers,
      KSPIN_DATAFLOW_IN, KSPIN_COMMUNICATION_NONE,
      &KSNODETYPE_MICROPHONE, NULL, 0
    }
  },
  {
    0, 0, 0, NULL,                       // 3: to the wave filter's capture side
    {
      0, NULL, 0, NULL,
      SIZEOF_ARRAY (mTopoBridgeRangePointers), mTopoBridgeRangePointers,
      KSPIN_DATAFLOW_OUT, KSPIN_COMMUNICATION_NONE,
      &KSCATEGORY_AUDIO, NULL, 0
    }
  },
};

//
// Volume / mute property handlers (KSPROPSETID_Audio, per channel; the
// channel is the KSNODEPROPERTY_AUDIO_CHANNEL instance, -1 = all).
//
static NTSTATUS
BasicSupport (PPCPROPERTY_REQUEST Request, BOOLEAN Volume)
{
  PKSPROPERTY_DESCRIPTION   Description;
  PKSPROPERTY_MEMBERSHEADER Members;
  PKSPROPERTY_STEPPING_LONG Range;
  ULONG                     Full;
  ULONG                     Channel;

  //
  // With KSPROPERTY_MEMBER_FLAG_BASICSUPPORT_MULTICHANNEL, MembersCount is
  // the channel count and there is one range per channel; a single entry
  // makes the control mono (Windows then only ever sets channel 0).
  //
  Full = sizeof (KSPROPERTY_DESCRIPTION) + sizeof (KSPROPERTY_MEMBERSHEADER) +
         AUDIO_CHANNELS * sizeof (KSPROPERTY_STEPPING_LONG);
  if (Request->ValueSize >= sizeof (KSPROPERTY_DESCRIPTION)) {
    Description = (PKSPROPERTY_DESCRIPTION)Request->Value;
    Description->AccessFlags       = KSPROPERTY_TYPE_BASICSUPPORT | KSPROPERTY_TYPE_GET | KSPROPERTY_TYPE_SET;
    Description->DescriptionSize   = Full;
    Description->PropTypeSet.Set   = KSPROPTYPESETID_General;
    Description->PropTypeSet.Id    = Volume ? VT_I4 : VT_BOOL;
    Description->PropTypeSet.Flags = 0;
    Description->MembersListCount  = 1;
    Description->Reserved          = 0;
    if (Request->ValueSize >= Full) {
      Members = (PKSPROPERTY_MEMBERSHEADER)(Description + 1);
      Members->MembersFlags = KSPROPERTY_MEMBER_STEPPEDRANGES;
      Members->MembersSize  = sizeof (KSPROPERTY_STEPPING_LONG);
      Members->MembersCount = AUDIO_CHANNELS;
      Members->Flags        = KSPROPERTY_MEMBER_FLAG_BASICSUPPORT_MULTICHANNEL;
      Range = (PKSPROPERTY_STEPPING_LONG)(Members + 1);
      for (Channel = 0; Channel < AUDIO_CHANNELS; Channel++, Range++) {
        Range->SteppingDelta        = Volume ? AUDIO_VOLUME_STEP : 1;
        Range->Reserved             = 0;
        Range->Bounds.SignedMinimum = Volume ? AUDIO_VOLUME_MIN : 0;
        Range->Bounds.SignedMaximum = Volume ? AUDIO_VOLUME_MAX : 1;
      }
      Request->ValueSize = Full;
    } else {
      Request->ValueSize = sizeof (KSPROPERTY_DESCRIPTION);
    }
    return STATUS_SUCCESS;
  }
  if (Request->ValueSize >= sizeof (ULONG)) {
    *(PULONG)Request->Value = KSPROPERTY_TYPE_BASICSUPPORT | KSPROPERTY_TYPE_GET | KSPROPERTY_TYPE_SET;
    Request->ValueSize      = sizeof (ULONG);
    return STATUS_SUCCESS;
  }
  Request->ValueSize = Full;
  return STATUS_BUFFER_OVERFLOW;
}

static LONG
ChannelOf (PPCPROPERTY_REQUEST Request)
{
  return ((Request->InstanceSize >= sizeof (LONG)) && (Request->Instance != NULL)) ? *(PLONG)Request->Instance : -1;
}

static NTSTATUS
PropertyHandlerVolume (PPCPROPERTY_REQUEST Request)
{
  LONG  Channel;

  if (Request->Verb & KSPROPERTY_TYPE_BASICSUPPORT) {
    return BasicSupport (Request, TRUE);
  }
  if (Request->ValueSize < sizeof (LONG)) {
    Request->ValueSize = sizeof (LONG);
    return STATUS_BUFFER_OVERFLOW;
  }
  Channel = ChannelOf (Request);
  if ((Channel >= AUDIO_CHANNELS) || (Channel < -1)) {
    return STATUS_INVALID_PARAMETER;
  }
  if (Request->Verb & KSPROPERTY_TYPE_GET) {
    *(PLONG)Request->Value = gAudio.Volume[(Channel < 0) ? 0 : Channel];
  } else if (Request->Verb & KSPROPERTY_TYPE_SET) {
    AudioSetVolume ((Channel < 0) ? (ULONG)~0 : (ULONG)Channel, *(PLONG)Request->Value);
  } else {
    return STATUS_INVALID_DEVICE_REQUEST;
  }
  Request->ValueSize = sizeof (LONG);
  return STATUS_SUCCESS;
}

static NTSTATUS
PropertyHandlerMute (PPCPROPERTY_REQUEST Request)
{
  if (Request->Verb & KSPROPERTY_TYPE_BASICSUPPORT) {
    return BasicSupport (Request, FALSE);
  }
  if (Request->ValueSize < sizeof (BOOL)) {
    Request->ValueSize = sizeof (BOOL);
    return STATUS_BUFFER_OVERFLOW;
  }
  if (Request->Verb & KSPROPERTY_TYPE_GET) {
    *(PBOOL)Request->Value = gAudio.Mute;
  } else if (Request->Verb & KSPROPERTY_TYPE_SET) {
    AudioSetMute (*(PBOOL)Request->Value != FALSE);
  } else {
    return STATUS_INVALID_DEVICE_REQUEST;
  }
  Request->ValueSize = sizeof (BOOL);
  return STATUS_SUCCESS;
}

static PCPROPERTY_ITEM  mVolumeProperties[] = {
  {
    &KSPROPSETID_Audio, KSPROPERTY_AUDIO_VOLUMELEVEL,
    KSPROPERTY_TYPE_GET | KSPROPERTY_TYPE_SET | KSPROPERTY_TYPE_BASICSUPPORT,
    PropertyHandlerVolume
  },
};

DEFINE_PCAUTOMATION_TABLE_PROP (mVolumeAutomation, mVolumeProperties);

static PCPROPERTY_ITEM  mMuteProperties[] = {
  {
    &KSPROPSETID_Audio, KSPROPERTY_AUDIO_MUTE,
    KSPROPERTY_TYPE_GET | KSPROPERTY_TYPE_SET | KSPROPERTY_TYPE_BASICSUPPORT,
    PropertyHandlerMute
  },
};

DEFINE_PCAUTOMATION_TABLE_PROP (mMuteAutomation, mMuteProperties);

static PCNODE_DESCRIPTOR  mTopoNodes[] = {
  { 0, &mVolumeAutomation, &KSNODETYPE_VOLUME, &KSAUDFNAME_MASTER_VOLUME },
  { 0, &mMuteAutomation,   &KSNODETYPE_MUTE,   &KSAUDFNAME_MASTER_MUTE },
};

static PCCONNECTION_DESCRIPTOR  mTopoConnections[] = {
  { PCFILTER_NODE, 0, 0, 1 },            // bridge in -> volume
  { 0,             0, 1, 1 },            // volume -> mute
  { 1,             0, PCFILTER_NODE, 1 },// mute -> speakers
  { PCFILTER_NODE, 2, PCFILTER_NODE, 3 },// microphone -> capture
};

static GUID  mTopoCategories[] = {
  STATICGUIDOF (KSCATEGORY_AUDIO),
  STATICGUIDOF (KSCATEGORY_TOPOLOGY),
};

static PCFILTER_DESCRIPTOR  mTopoFilter = {
  0, NULL,
  sizeof (PCPIN_DESCRIPTOR), SIZEOF_ARRAY (mTopoPins), mTopoPins,
  sizeof (PCNODE_DESCRIPTOR), SIZEOF_ARRAY (mTopoNodes), mTopoNodes,
  SIZEOF_ARRAY (mTopoConnections), mTopoConnections,
  SIZEOF_ARRAY (mTopoCategories), mTopoCategories
};

class CMiniportTopology : public IMiniportTopology, public CUnknown
{
public:
  DECLARE_STD_UNKNOWN ();
  DEFINE_STD_CONSTRUCTOR (CMiniportTopology);
  ~CMiniportTopology ();
  IMP_IMiniportTopology;

private:
  PPORTTOPOLOGY  m_Port;
};

NTSTATUS
CreateTopologyMiniport (PUNKNOWN *Unknown, REFCLSID ClassId, PUNKNOWN UnknownOuter, POOL_TYPE PoolType)
{
  UNREFERENCED_PARAMETER (ClassId);
  STD_CREATE_BODY_ (CMiniportTopology, Unknown, UnknownOuter, PoolType, PMINIPORTTOPOLOGY);
}

CMiniportTopology::~CMiniportTopology ()
{
  if (m_Port != NULL) {
    m_Port->Release ();
  }
}

STDMETHODIMP_(NTSTATUS)
CMiniportTopology::NonDelegatingQueryInterface (_In_ REFIID Interface, _COM_Outptr_ PVOID *Object)
{
  if (IsEqualGUIDAligned (Interface, IID_IUnknown)) {
    *Object = PVOID (PUNKNOWN (PMINIPORTTOPOLOGY (this)));
  } else if (IsEqualGUIDAligned (Interface, IID_IMiniport)) {
    *Object = PVOID (PMINIPORT (this));
  } else if (IsEqualGUIDAligned (Interface, IID_IMiniportTopology)) {
    *Object = PVOID (PMINIPORTTOPOLOGY (this));
  } else {
    *Object = NULL;
    return STATUS_INVALID_PARAMETER;
  }
  PUNKNOWN (*Object)->AddRef ();
  return STATUS_SUCCESS;
}

STDMETHODIMP_(NTSTATUS)
CMiniportTopology::Init (_In_ PUNKNOWN UnknownAdapter, _In_ PRESOURCELIST ResourceList, _In_ PPORTTOPOLOGY Port)
{
  UNREFERENCED_PARAMETER (UnknownAdapter);
  UNREFERENCED_PARAMETER (ResourceList);

  m_Port = Port;
  m_Port->AddRef ();
  return STATUS_SUCCESS;
}

STDMETHODIMP_(NTSTATUS)
CMiniportTopology::GetDescription (_Out_ PPCFILTER_DESCRIPTOR *Description)
{
  *Description = &mTopoFilter;
  return STATUS_SUCCESS;
}

STDMETHODIMP_(NTSTATUS)
CMiniportTopology::DataRangeIntersection (_In_ ULONG PinId, _In_ PKSDATARANGE DataRange,
                                          _In_ PKSDATARANGE MatchingDataRange, _In_ ULONG OutputBufferLength,
                                          PVOID ResultantFormat, _Out_ PULONG ResultantFormatLength)
{
  UNREFERENCED_PARAMETER (PinId);
  UNREFERENCED_PARAMETER (DataRange);
  UNREFERENCED_PARAMETER (MatchingDataRange);
  UNREFERENCED_PARAMETER (OutputBufferLength);
  UNREFERENCED_PARAMETER (ResultantFormat);
  UNREFERENCED_PARAMETER (ResultantFormatLength);
  return STATUS_NOT_IMPLEMENTED;
}
