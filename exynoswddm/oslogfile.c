/** @file
  exynoswddm: copies the OS log ring (every driver's lines) to
  C:\malikm\oslog-kmd.txt once a second, so the trace survives resets
  that lose RAM.  A system thread waits until the file can be opened (the
  boot volume may not be up yet), then appends what the ring gained.
  Diagnostic only.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include "exynoswddm.h"

NTSYSAPI NTSTATUS NTAPI ZwFlushBuffersFile (HANDLE FileHandle, PIO_STATUS_BLOCK IoStatusBlock);
NTSYSAPI NTSTATUS NTAPI ZwWaitForSingleObject (HANDLE Handle, BOOLEAN Alertable, PLARGE_INTEGER Timeout);

static HANDLE         mThread;
static volatile LONG  mStop;
static KEVENT         mStopEvent;

static HANDLE
OpenLog (VOID)
{
  UNICODE_STRING     Name;
  OBJECT_ATTRIBUTES  Oa;
  IO_STATUS_BLOCK    Io;
  HANDLE             File = NULL;
  NTSTATUS           Status;

  RtlInitUnicodeString (&Name, L"\\??\\C:\\malikm\\oslog-kmd.txt");
  InitializeObjectAttributes (&Oa, &Name, OBJ_KERNEL_HANDLE | OBJ_CASE_INSENSITIVE, NULL, NULL);
  Status = ZwCreateFile (&File, FILE_APPEND_DATA | SYNCHRONIZE, &Oa, &Io, NULL, FILE_ATTRIBUTE_NORMAL,
                         FILE_SHARE_READ, FILE_OVERWRITE_IF,
                         FILE_SYNCHRONOUS_IO_NONALERT | FILE_NON_DIRECTORY_FILE | FILE_WRITE_THROUGH, NULL, 0);
  return NT_SUCCESS (Status) ? File : NULL;
}

/**
  Flush the C: volume (all its cached data and metadata). Without it a
  forced reset loses the MFT records of files written in this session -
  the files Mesa and this driver log to then read as damaged from Linux
  (ntfs-3g does not replay the NTFS journal). Kernel-mode open: no access
  check.
**/
static VOID
FlushVolume (VOID)
{
  UNICODE_STRING     Name;
  OBJECT_ATTRIBUTES  Oa;
  IO_STATUS_BLOCK    Io;
  HANDLE             Volume = NULL;
  static ULONG       Logs;

  RtlInitUnicodeString (&Name, L"\\??\\C:");
  InitializeObjectAttributes (&Oa, &Name, OBJ_KERNEL_HANDLE | OBJ_CASE_INSENSITIVE, NULL, NULL);
  if (NT_SUCCESS (ZwCreateFile (&Volume, FILE_WRITE_DATA | SYNCHRONIZE, &Oa, &Io, NULL, 0,
                                FILE_SHARE_READ | FILE_SHARE_WRITE, FILE_OPEN, FILE_SYNCHRONOUS_IO_NONALERT, NULL, 0))) {
    NTSTATUS  Status = ZwFlushBuffersFile (Volume, &Io);

    if (Logs++ < 2) {
      OsLog ("exynoswddm: C: volume flush - 0x%08lx", (ULONG)Status);
    }
    ZwClose (Volume);
  }
}

static VOID
LogThread (PVOID Context)
{
  HANDLE           File = NULL;
  ULONG            Done = 0;
  ULONG            Ticks = 0;
  BOOLEAN          First = TRUE;
  LARGE_INTEGER    Period;
  IO_STATUS_BLOCK  Io;
  static UCHAR     Chunk[4096];

  UNREFERENCED_PARAMETER (Context);
  Period.QuadPart = -10000000;                     // 1 s
  while (!mStop) {
    volatile OS_LOG_RING  *Ring = mOsLog;

    if ((File == NULL) && ((File = OpenLog ()) == NULL)) {
      KeWaitForSingleObject (&mStopEvent, Executive, KernelMode, FALSE, &Period);
      continue;
    }
    if ((Ring != NULL) && (Ring->Magic == OS_LOG_MAGIC)) {
      ULONG  Head = Ring->Head;
      BOOLEAN Wrote = FALSE;

      if (First) {
        // the whole ring as it is now (it holds the boot so far)
        Done  = (Head > OS_LOG_DATA_SIZE) ? Head - OS_LOG_DATA_SIZE : 0;
        First = FALSE;
      }
      if (Head - Done > OS_LOG_DATA_SIZE) {
        Done = Head - OS_LOG_DATA_SIZE;            // lost to the ring
      }
      while (Done != Head) {
        ULONG  N = min (Head - Done, (ULONG)sizeof (Chunk));
        ULONG  I;

        for (I = 0; I < N; I++) {
          Chunk[I] = Ring->Data[(Done + I) % OS_LOG_DATA_SIZE];
        }
        if (!NT_SUCCESS (ZwWriteFile (File, NULL, NULL, NULL, &Io, Chunk, N, NULL, NULL))) {
          break;
        }
        Done += N;
        Wrote = TRUE;
      }
      if (Wrote) {
        ZwFlushBuffersFile (File, &Io);
      }
    }
    if ((++Ticks % 5) == 0) {
      FlushVolume ();
    }
    KeWaitForSingleObject (&mStopEvent, Executive, KernelMode, FALSE, &Period);
  }
  if (File != NULL) {
    ZwClose (File);
  }
  PsTerminateSystemThread (STATUS_SUCCESS);
}

VOID
OsLogFileStart (VOID)
{
  OBJECT_ATTRIBUTES  Oa;
  NTSTATUS           Status;

  if (mThread != NULL) {
    return;
  }
  KeInitializeEvent (&mStopEvent, NotificationEvent, FALSE);
  InitializeObjectAttributes (&Oa, NULL, OBJ_KERNEL_HANDLE, NULL, NULL);
  Status = PsCreateSystemThread (&mThread, THREAD_ALL_ACCESS, &Oa, NULL, NULL, LogThread, NULL);
  OsLog ("exynoswddm: OS log copy to C:\\malikm\\oslog-kmd.txt - 0x%08lx", (ULONG)Status);
  if (!NT_SUCCESS (Status)) {
    mThread = NULL;
  }
}

VOID
OsLogFileStop (VOID)
{
  if (mThread == NULL) {
    return;
  }
  mStop = 1;
  KeSetEvent (&mStopEvent, 0, FALSE);
  ZwWaitForSingleObject (mThread, FALSE, NULL);
  ZwClose (mThread);
  mThread = NULL;
}
