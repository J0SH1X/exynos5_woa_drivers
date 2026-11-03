/** @file
  exynoswddm DDI trace: wraps the DRIVER_INITIALIZATION_DATA entries that
  dxgkrnl calls after start (not the ISR / DPC) and logs each call's
  status: the first DDTRACE_FIRST calls of every DDI and every failure,
  with the argument fields that tell the calls apart (escape code,
  adapter-info type, present flags, visibility, ...).  Diagnostic only.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include "exynoswddm.h"

#define DDTRACE_FIRST   25
#define DDTRACE_BUDGET  1500

typedef NTSTATUS (APIENTRY *DDI2)(HANDLE, PVOID);
typedef NTSTATUS (APIENTRY *DDI1)(HANDLE);

static volatile LONG  mBudget;

typedef enum {
  T_QueryAdapterInfo, T_SetPointerPosition, T_SetPointerShape, T_IsSupportedVidPn,
  T_RecommendFunctionalVidPn, T_EnumVidPnCofuncModality, T_SetVidPnSourceVisibility, T_CommitVidPn,
  T_UpdateActiveVidPnPresentPath, T_RecommendMonitorModes, T_GetScanLine, T_QueryVidPnHWCapability,
  T_CreateDevice, T_CreateContext, T_CreateAllocation, T_DestroyAllocation, T_DescribeAllocation,
  T_GetStandardAllocationDriverData, T_OpenAllocation, T_CloseAllocation, T_BuildPagingBuffer,
  T_Render, T_Present, T_Patch, T_SubmitCommand, T_PreemptCommand, T_QueryCurrentFence,
  T_CollectDbgInfo, T_Escape, T_SetPalette, T_SetVidPnSourceAddress, T_RecommendVidPnTopology,
  T_StopCapture, T_AcquireSwizzlingRange, T_ReleaseSwizzlingRange, T_QueryDependentEngineGroup,
  T_QueryEngineStatus, T_ResetEngine, T_CancelCommand,
  T_DestroyDevice, T_DestroyContext, T_ResetFromTimeout, T_RestartFromTimeout, T_ControlInterrupt,
  T_COUNT
} DDI_ID;

static const char * const  mName[T_COUNT] = {
  "QueryAdapterInfo", "SetPointerPosition", "SetPointerShape", "IsSupportedVidPn",
  "RecommendFunctionalVidPn", "EnumVidPnCofuncModality", "SetVidPnSourceVisibility", "CommitVidPn",
  "UpdateActiveVidPnPresentPath", "RecommendMonitorModes", "GetScanLine", "QueryVidPnHWCapability",
  "CreateDevice", "CreateContext", "CreateAllocation", "DestroyAllocation", "DescribeAllocation",
  "GetStandardAllocationDriverData", "OpenAllocation", "CloseAllocation", "BuildPagingBuffer",
  "Render", "Present", "Patch", "SubmitCommand", "PreemptCommand", "QueryCurrentFence",
  "CollectDbgInfo", "Escape", "SetPalette", "SetVidPnSourceAddress", "RecommendVidPnTopology",
  "StopCapture", "AcquireSwizzlingRange", "ReleaseSwizzlingRange", "QueryDependentEngineGroup",
  "QueryEngineStatus", "ResetEngine", "CancelCommand",
  "DestroyDevice", "DestroyContext", "ResetFromTimeout", "RestartFromTimeout", "ControlInterrupt",
};

static PVOID           mOrig[T_COUNT];
static volatile LONG   mCalls[T_COUNT];

//
// A few argument fields per DDI, so the log tells the calls apart.
//
static VOID
Detail (DDI_ID Id, PVOID Arg, ULONG *D0, ULONG *D1)
{
  *D0 = 0;
  *D1 = 0;
  if (Arg == NULL) {
    return;
  }
  switch (Id) {
    case T_QueryAdapterInfo:
      *D0 = (ULONG)((DXGKARG_QUERYADAPTERINFO *)Arg)->Type;
      *D1 = ((DXGKARG_QUERYADAPTERINFO *)Arg)->OutputDataSize;
      break;
    case T_Escape:
      *D0 = ((DXGKARG_ESCAPE *)Arg)->PrivateDriverDataSize;
      if ((((DXGKARG_ESCAPE *)Arg)->pPrivateDriverData != NULL) && (*D0 >= 4)) {
        *D1 = *(UINT32 *)((DXGKARG_ESCAPE *)Arg)->pPrivateDriverData;
      }
      break;
    case T_Present:
      *D0 = ((DXGKARG_PRESENT *)Arg)->Flags.Value;
      *D1 = ((DXGKARG_PRESENT *)Arg)->DmaSize;
      break;
    case T_Render:
      *D0 = ((DXGKARG_RENDER *)Arg)->CommandLength;
      *D1 = ((DXGKARG_RENDER *)Arg)->AllocationListSize;
      break;
    case T_SetVidPnSourceVisibility:
      *D0 = ((DXGKARG_SETVIDPNSOURCEVISIBILITY *)Arg)->VidPnSourceId;
      *D1 = ((DXGKARG_SETVIDPNSOURCEVISIBILITY *)Arg)->Visible;
      break;
    case T_SetVidPnSourceAddress:
      *D0 = ((DXGKARG_SETVIDPNSOURCEADDRESS *)Arg)->PrimaryAddress.LowPart;
      *D1 = ((DXGKARG_SETVIDPNSOURCEADDRESS *)Arg)->Flags.Value;
      break;
    case T_OpenAllocation:
      *D0 = ((DXGKARG_OPENALLOCATION *)Arg)->NumAllocations;
      *D1 = ((DXGKARG_OPENALLOCATION *)Arg)->PrivateDriverSize;
      break;
    case T_CreateAllocation:
      *D0 = ((DXGKARG_CREATEALLOCATION *)Arg)->NumAllocations;
      *D1 = ((DXGKARG_CREATEALLOCATION *)Arg)->Flags.Value;
      break;
    case T_DescribeAllocation:
      *D0 = (ULONG)(ULONG_PTR)((DXGKARG_DESCRIBEALLOCATION *)Arg)->hAllocation;
      break;
    case T_BuildPagingBuffer:
      *D0 = (ULONG)((DXGKARG_BUILDPAGINGBUFFER *)Arg)->Operation;
      break;
    case T_SubmitCommand:
      *D0 = ((DXGKARG_SUBMITCOMMAND *)Arg)->SubmissionFenceId;
      *D1 = ((DXGKARG_SUBMITCOMMAND *)Arg)->Flags.Value;
      break;
    case T_CreateContext:
      *D0 = ((DXGKARG_CREATECONTEXT *)Arg)->NodeOrdinal;
      *D1 = ((DXGKARG_CREATECONTEXT *)Arg)->Flags.Value;
      break;
    case T_GetStandardAllocationDriverData:
      *D0 = (ULONG)((DXGKARG_GETSTANDARDALLOCATIONDRIVERDATA *)Arg)->StandardAllocationType;
      break;
    default:
      break;
  }
}

static VOID
Report (DDI_ID Id, NTSTATUS Status, ULONG D0, ULONG D1)
{
  LONG  N = InterlockedIncrement (&mCalls[Id]);

  if ((N > DDTRACE_FIRST) && NT_SUCCESS (Status)) {
    return;
  }
  if (InterlockedIncrement (&mBudget) > DDTRACE_BUDGET) {
    return;
  }
  OsLog ("exynoswddm: ddi %s #%ld (0x%lx, 0x%lx) - 0x%08lx", mName[Id], N, D0, D1, (ULONG)Status);
}

#define WRAP2(n)                                                     \
  static NTSTATUS APIENTRY W_##n (HANDLE H, PVOID Arg)               \
  {                                                                  \
    ULONG     D0, D1;                                                \
    NTSTATUS  Status;                                                \
    Detail (T_##n, Arg, &D0, &D1);                                   \
    Status = ((DDI2)mOrig[T_##n]) (H, Arg);                          \
    Report (T_##n, Status, D0, D1);                                  \
    return Status;                                                   \
  }

#define WRAP1(n)                                                     \
  static NTSTATUS APIENTRY W_##n (HANDLE H)                          \
  {                                                                  \
    NTSTATUS  Status = ((DDI1)mOrig[T_##n]) (H);                     \
    Report (T_##n, Status, 0, 0);                                    \
    return Status;                                                   \
  }

WRAP2 (QueryAdapterInfo)
WRAP2 (SetPointerPosition)
WRAP2 (SetPointerShape)
WRAP2 (IsSupportedVidPn)
WRAP2 (RecommendFunctionalVidPn)
WRAP2 (EnumVidPnCofuncModality)
WRAP2 (SetVidPnSourceVisibility)
WRAP2 (CommitVidPn)
WRAP2 (UpdateActiveVidPnPresentPath)
WRAP2 (RecommendMonitorModes)
WRAP2 (GetScanLine)
WRAP2 (QueryVidPnHWCapability)
WRAP2 (CreateDevice)
WRAP2 (CreateContext)
WRAP2 (CreateAllocation)
WRAP2 (DestroyAllocation)
WRAP2 (DescribeAllocation)
WRAP2 (GetStandardAllocationDriverData)
WRAP2 (OpenAllocation)
WRAP2 (CloseAllocation)
WRAP2 (BuildPagingBuffer)
WRAP2 (Render)
WRAP2 (Present)
WRAP2 (Patch)
WRAP2 (SubmitCommand)
WRAP2 (PreemptCommand)
WRAP2 (QueryCurrentFence)
WRAP2 (CollectDbgInfo)
WRAP2 (Escape)
WRAP2 (SetPalette)
WRAP2 (SetVidPnSourceAddress)
WRAP2 (RecommendVidPnTopology)
WRAP2 (StopCapture)
WRAP2 (AcquireSwizzlingRange)
WRAP2 (ReleaseSwizzlingRange)
WRAP2 (QueryDependentEngineGroup)
WRAP2 (QueryEngineStatus)
WRAP2 (ResetEngine)
WRAP2 (CancelCommand)
WRAP1 (DestroyDevice)
WRAP1 (DestroyContext)
WRAP1 (ResetFromTimeout)
WRAP1 (RestartFromTimeout)

static NTSTATUS APIENTRY
W_ControlInterrupt (IN_CONST_HANDLE H, IN_CONST_DXGK_INTERRUPT_TYPE Type, IN_BOOLEAN Enable)
{
  NTSTATUS  Status = ((DXGKDDI_CONTROLINTERRUPT *)mOrig[T_ControlInterrupt]) (H, Type, Enable);

  Report (T_ControlInterrupt, Status, (ULONG)Type, Enable);
  return Status;
}

#define HOOK(n)                                                      \
  do {                                                               \
    if (Init->DxgkDdi##n != NULL) {                                  \
      mOrig[T_##n] = (PVOID)Init->DxgkDdi##n;                        \
      *(PVOID *)&Init->DxgkDdi##n = (PVOID)W_##n;                    \
    }                                                                \
  } while (0)

VOID
DdTraceInit (DRIVER_INITIALIZATION_DATA *Init)
{
  HOOK (QueryAdapterInfo);
  HOOK (SetPointerPosition);
  HOOK (SetPointerShape);
  HOOK (IsSupportedVidPn);
  HOOK (RecommendFunctionalVidPn);
  HOOK (EnumVidPnCofuncModality);
  HOOK (SetVidPnSourceVisibility);
  HOOK (CommitVidPn);
  HOOK (UpdateActiveVidPnPresentPath);
  HOOK (RecommendMonitorModes);
  HOOK (GetScanLine);
  HOOK (QueryVidPnHWCapability);
  HOOK (CreateDevice);
  HOOK (CreateContext);
  HOOK (CreateAllocation);
  HOOK (DestroyAllocation);
  HOOK (DescribeAllocation);
  HOOK (GetStandardAllocationDriverData);
  HOOK (OpenAllocation);
  HOOK (CloseAllocation);
  HOOK (BuildPagingBuffer);
  HOOK (Render);
  HOOK (Present);
  HOOK (Patch);
  HOOK (SubmitCommand);
  HOOK (PreemptCommand);
  HOOK (QueryCurrentFence);
  HOOK (CollectDbgInfo);
  HOOK (Escape);
  HOOK (SetPalette);
  HOOK (SetVidPnSourceAddress);
  HOOK (RecommendVidPnTopology);
  HOOK (StopCapture);
  HOOK (AcquireSwizzlingRange);
  HOOK (ReleaseSwizzlingRange);
  HOOK (QueryDependentEngineGroup);
  HOOK (QueryEngineStatus);
  HOOK (ResetEngine);
  HOOK (CancelCommand);
  HOOK (DestroyDevice);
  HOOK (DestroyContext);
  HOOK (ResetFromTimeout);
  HOOK (RestartFromTimeout);
  HOOK (ControlInterrupt);
}
