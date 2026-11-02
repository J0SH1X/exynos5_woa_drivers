/** @file
  exynosaudio: v1awifi speakers as a PortCls WaveRT device (Windows RT 8.1).

    WM5102 codec      SPI2 through exynosspi (the SpiSerialBus connection),
                      LDOENA / RESET through exynosgpio (GpioIo connections);
                      Exynos5420Pkg's Wm5102Core.c, built unchanged
    I2S0 + IDMA       clock slave, IDMA looping over the WaveRT buffer
                      (physically contiguous DRAM): the hardware plays the
                      audio engine's buffer directly, the IDMA word counter
                      is the play position; Exynos5420Pkg's ExynosI2sCore.c

  One render stream, 48 kHz / 16-bit / stereo (the audio engine converts);
  no volume node (the engine's software volume), polling mode (no event
  notifications yet).

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef EXYNOSAUDIO_H_
#define EXYNOSAUDIO_H_

extern "C" {
#include <ntddk.h>
}
#include <portcls.h>
#include <stdunk.h>
#include <ksmedia.h>

extern "C" {
#include <gpio.h>
#include <spb.h>
#define RESHUB_USE_HELPER_ROUTINES      // RESOURCE_HUB_CREATE_PATH_FROM_ID
#include <reshub.h>
#include <ExynosEnv.h>
#include "../../uefi/Exynos5420Pkg/Drivers/Audio/Wm5102Core.h"
#include "../../uefi/Exynos5420Pkg/Drivers/Audio/ExynosI2sCore.h"
}

#define AUDIO_TAG          'oduA'
#define AUDIO_RATE         48000
#define AUDIO_CHANNELS     2
#define AUDIO_BITS         16
#define AUDIO_MAX_BUFFER   0x3FFFC    // IDMA counts 16 bits of 32-bit words
#define AUDIO_DAC_VOLUME   0x80       // 0 dB

//
// Speaker volume node: KS levels are dB in 1/65536 steps; the codec DAC
// goes from -64 dB (0x00) to 0 dB (0x80) in 0.5 dB (0x8000) steps.
//
#define AUDIO_VOLUME_MIN   (-64 * 65536)
#define AUDIO_VOLUME_MAX   0
#define AUDIO_VOLUME_STEP  0x8000

typedef struct {
  PDEVICE_OBJECT  Fdo;
  PUCHAR          I2sVa;
  PUCHAR          AudssVa;
  PUCHAR          GpzVa;
  PUCHAR          PmuVa;
  EXYNOS_I2S      I2s;
  WM5102          Codec;
  HANDLE          SpiHandle;
  PFILE_OBJECT    SpiFile;
  HANDLE          GpioHandle[3];        // 0: LDOENA, 1: /RESET, 2: MAINMIC_BIAS
  PFILE_OBJECT    GpioFile[3];
  KMUTEX          Lock;
  BOOLEAN         CodecReady;
  BOOLEAN         Rendering;
  ULONG           SpiErrors;
  ULONG           Streams;
  LONG            Volume[AUDIO_CHANNELS];   // KS dB, per channel
  BOOLEAN         Mute;
  PKINTERRUPT     CodecInterrupt;           // GpioInt GPX2(5), passive-level ISR
  ULONG           CodecIrqs;
  BOOLEAN         JackIn;

  //
  // Capture: I2S0 RX FIFO -> system DMA (ADMA0 through halextpl330, the
  // FixedDMA resource), looping over the WaveRT buffer.
  //
  PDMA_ADAPTER    DmaAdapter;
  ULONG           DmaMapRegisters;
  ULONG           DmaRequestLine;
  ULONG           DmaChannel;
  PVOID           DmaMapRegisterBase;
  ULONG_PTR       DmaContext[DMA_TRANSFER_CONTEXT_SIZE_V1 / sizeof (ULONG_PTR) + 1];
  PMDL            CaptureMdl;
  ULONG           CaptureBytes;
  BOOLEAN         Capturing;
} EXYNOS_AUDIO;

extern EXYNOS_AUDIO  gAudio;

NTSTATUS AudioRenderStart (ULONG Physical, ULONG Bytes);
VOID     AudioRenderStop (VOID);
ULONG    AudioRenderPosition (VOID);
VOID     AudioSetVolume (ULONG Channel, LONG Level);   // Channel ~0: all
VOID     AudioSetMute (BOOLEAN Mute);
NTSTATUS AudioCaptureStart (PMDL Mdl, ULONG Bytes);
VOID     AudioCaptureStop (VOID);
ULONG    AudioCapturePosition (VOID);

#define AUDIO_MIC_GAIN_DB  30                     // stock tinyucm "IN1L Volume"

NTSTATUS CreateWaveMiniport (PUNKNOWN *Unknown, REFCLSID ClassId, PUNKNOWN UnknownOuter, POOL_TYPE PoolType);
NTSTATUS CreateTopologyMiniport (PUNKNOWN *Unknown, REFCLSID ClassId, PUNKNOWN UnknownOuter, POOL_TYPE PoolType);

#endif
