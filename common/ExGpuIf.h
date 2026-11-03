/** @file
  Direct-call interface of exynosgpu (Mali-T628) for exynoswddm.

  exynosgpu owns the GPU: registers, power, interrupts, the page tables of
  address space 0, the job slots.  exynoswddm finds it by its device
  interface (GUID_EXYNOS_GPU_INTERFACE), opens it and asks for this table
  with IRP_MN_QUERY_INTERFACE (InterfaceType = the same GUID).

  Calls are PASSIVE_LEVEL; the driver serializes them.  GPU addresses of
  callers are their business (exynoswddm keeps 0x10000000..0xF0000000);
  exynosgpu uses none of that range itself.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef __EX_GPU_IF_H__
#define __EX_GPU_IF_H__

// {6b7b7f1e-5420-4c5a-9b1d-3e2a0a6f7d01}
DEFINE_GUID (GUID_EXYNOS_GPU_INTERFACE, 0x6b7b7f1e, 0x5420, 0x4c5a, 0x9b, 0x1d, 0x3e, 0x2a, 0x0a, 0x6f, 0x7d, 0x01);

#define EX_GPU_IF_VERSION    1

#define EX_GPU_VA_START      0x10000000ull
#define EX_GPU_VA_END        0xF0000000ull

typedef struct {
  ULONG   GpuId;                 // GPU_ID (0x06200010 = T628 r0p1)
  ULONG   L2Features;
  ULONG   CoreFeatures;
  ULONG   TilerFeatures;
  ULONG   MemFeatures;
  ULONG   MmuFeatures;
  ULONG   AsPresent;
  ULONG   JsPresent;
  ULONG   JsFeatures[16];
  ULONG   TextureFeatures[4];
  ULONG   ThreadMaxThreads;
  ULONG   ThreadMaxWorkgroupSize;
  ULONG   ThreadMaxBarrierSize;
  ULONG   ThreadFeatures;
  ULONG   ThreadTlsAlloc;
  ULONG   CoherencyFeatures;
  ULONGLONG ShaderPresent;
  ULONGLONG TilerPresent;
  ULONGLONG L2Present;
  ULONGLONG StackPresent;
  ULONG   ClockMhz;
  ULONG   Up;                    // powered and address space 0 live
} EX_GPU_INFO;

//
// Job chain result: JS_STATUS of the slot (0x01 = done; 0x40-0x7F faults),
// or EX_GPU_TIMEOUT.
//
#define EX_GPU_TIMEOUT       0xFFFFFFFFu

typedef VOID     (*EX_GPU_GET_INFO)(PVOID Context, EX_GPU_INFO *Info);
typedef NTSTATUS (*EX_GPU_MAP_PAGES)(PVOID Context, ULONGLONG GpuVa, CONST PFN_NUMBER *Pfn, ULONG Pages);
typedef VOID     (*EX_GPU_UNMAP_PAGES)(PVOID Context, ULONGLONG GpuVa, ULONG Pages);
typedef ULONG    (*EX_GPU_RUN_CHAIN)(PVOID Context, ULONG Slot, ULONGLONG Chain, ULONG TimeoutMs);

typedef struct {
  INTERFACE           Header;    // Size, Version (EX_GPU_IF_VERSION), Context, Reference / Dereference
  EX_GPU_GET_INFO     GetInfo;
  EX_GPU_MAP_PAGES    MapPages;
  EX_GPU_UNMAP_PAGES  UnmapPages;
  EX_GPU_RUN_CHAIN    RunChain;  // runs one chain to completion (job interrupt), returns JS_STATUS
} EX_GPU_INTERFACE;

#endif // __EX_GPU_IF_H__
