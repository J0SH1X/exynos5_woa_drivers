/** @file
  exynosumd.dll: user-mode display driver for exynoswddm, milestone 1 stub.

  Exports the D3D9 (OpenAdapter) and D3D10/11 (OpenAdapter10,
  OpenAdapter10_2) entry points.  Each reports its call (entry point,
  interface and runtime versions) to the kernel driver's OS log through
  the sizes of adapter queries (see Log), reads the driver's caps, and
  fails with E_NOTIMPL: the first test only shows which runtime loads the
  driver and what it asks for.

  No CRT; the minimal D3D DDI declarations below follow d3dumddi.h /
  d3d10umddi.h (WDK 10.0.19041).

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

typedef unsigned int    UINT;
typedef unsigned short  WCHAR;
typedef long            HRESULT;
typedef void            *HANDLE;
typedef int             BOOL;
typedef unsigned long   DWORD;
typedef char            CHAR;
typedef unsigned int    UINT32;
typedef int             INT32;
typedef unsigned long long UINT64;

#define APIENTRY  __stdcall
#define E_NOTIMPL ((HRESULT)0x80004001L)

#include "../exynoswddm/ExCmd.h"

typedef struct {
  void  *pPrivateDriverData;
  UINT  PrivateDriverDataSize;
} D3DDDICB_QUERYADAPTERINFO;

typedef HRESULT (APIENTRY *PFND3DDDI_QUERYADAPTERINFOCB)(HANDLE hAdapter, const D3DDDICB_QUERYADAPTERINFO *);

typedef struct {
  PFND3DDDI_QUERYADAPTERINFOCB  pfnQueryAdapterInfoCb;
  void                          *pfnGetMultisampleMethodListCb;
} D3DDDI_ADAPTERCALLBACKS;

typedef struct {
  HANDLE                         hAdapter;
  UINT                           Interface;
  UINT                           Version;
  const D3DDDI_ADAPTERCALLBACKS  *pAdapterCallbacks;
  void                           *pAdapterFuncs;
  UINT                           DriverVersion;
} D3DDDIARG_OPENADAPTER;

typedef struct {
  void                           *hRTAdapter;
  void                           *hAdapter;
  UINT                           Interface;
  UINT                           Version;
  const D3DDDI_ADAPTERCALLBACKS  *pAdapterCallbacks;
  void                           *pAdapterFuncs;
} D3D10DDIARG_OPENADAPTER;


/*
  dxgkrnl hands the driver a fresh output buffer for the adapter query, so
  nothing written into it arrives.  The size does: one query per hex digit,
  size = EX_UMD_DIGIT_BASE + (index << 4) + digit; index 0 = entry point,
  1..8 = Interface, 9..16 = Version (most significant digit first), then
  the normal caps query.  The kernel driver reassembles and logs them.
*/
static void
Log (const D3DDDI_ADAPTERCALLBACKS *Cb, HANDLE hAdapter, UINT Entry, UINT Interface, UINT Version)
{
  static UINT32              Buffer[(EX_UMD_DIGIT_BASE + (EX_UMD_DIGITS << 4)) / 4];   // largest size encoded
  D3DDDICB_QUERYADAPTERINFO  Q;
  UINT                       Digit[EX_UMD_DIGITS];
  UINT                       I;

  if ((Cb == 0) || (Cb->pfnQueryAdapterInfoCb == 0)) {
    return;
  }
  Digit[0] = Entry & 15;
  for (I = 0; I < 8; I++) {
    Digit[1 + I] = (Interface >> (28 - 4 * I)) & 15;
    Digit[9 + I] = (Version >> (28 - 4 * I)) & 15;
  }
  Q.pPrivateDriverData = Buffer;
  for (I = 0; I < EX_UMD_DIGITS; I++) {
    Q.PrivateDriverDataSize = EX_UMD_DIGIT_BASE + (I << 4) + Digit[I];
    Cb->pfnQueryAdapterInfoCb (hAdapter, &Q);
  }
  Q.PrivateDriverDataSize = sizeof (EX_UMD_CAPS);
  Cb->pfnQueryAdapterInfoCb (hAdapter, &Q);
}

__declspec(dllexport) HRESULT APIENTRY
OpenAdapter (D3DDDIARG_OPENADAPTER *pOpenData)
{
  Log (pOpenData->pAdapterCallbacks, pOpenData->hAdapter, 9, pOpenData->Interface, pOpenData->Version);
  return E_NOTIMPL;
}

__declspec(dllexport) HRESULT APIENTRY
OpenAdapter10 (D3D10DDIARG_OPENADAPTER *pOpenData)
{
  Log (pOpenData->pAdapterCallbacks, pOpenData->hRTAdapter, 10, pOpenData->Interface, pOpenData->Version);
  return E_NOTIMPL;
}

__declspec(dllexport) HRESULT APIENTRY
OpenAdapter10_2 (D3D10DDIARG_OPENADAPTER *pOpenData)
{
  Log (pOpenData->pAdapterCallbacks, pOpenData->hRTAdapter, 11, pOpenData->Interface, pOpenData->Version);
  return E_NOTIMPL;
}

BOOL APIENTRY
_DllMainCRTStartup (HANDLE Instance, DWORD Reason, void *Reserved)
{
  (void)Instance;
  (void)Reason;
  (void)Reserved;
  return 1;
}
