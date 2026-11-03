/*
  RT 8.1 user-mode code-integrity probe: does a test-signed (or unsigned)
  desktop EXE run, and does it load a test-signed DLL?  Result in a message
  box and in umcitest.txt next to the EXE.  Needed before writing a D3D
  user-mode driver (exynosgpu phase 3).
*/
typedef void *HANDLE; typedef unsigned long DWORD; typedef int BOOL; typedef unsigned short WCHAR;
typedef int (__stdcall *PROBE)(void);
__declspec(dllimport) HANDLE __stdcall LoadLibraryW(const WCHAR *);
__declspec(dllimport) void * __stdcall GetProcAddress(HANDLE, const char *);
__declspec(dllimport) DWORD __stdcall GetLastError(void);
__declspec(dllimport) DWORD __stdcall GetModuleFileNameW(HANDLE, WCHAR *, DWORD);
__declspec(dllimport) HANDLE __stdcall CreateFileW(const WCHAR *, DWORD, DWORD, void *, DWORD, DWORD, HANDLE);
__declspec(dllimport) BOOL __stdcall WriteFile(HANDLE, const void *, DWORD, DWORD *, void *);
__declspec(dllimport) BOOL __stdcall CloseHandle(HANDLE);
__declspec(dllimport) void __stdcall ExitProcess(unsigned);
__declspec(dllimport) int __stdcall MessageBoxW(HANDLE, const WCHAR *, const WCHAR *, unsigned);
__declspec(dllimport) int __cdecl wsprintfW(WCHAR *, const WCHAR *, ...);

void __stdcall mainCRTStartup(void)
{
  WCHAR Path[260], Msg[200];
  DWORD Len = GetModuleFileNameW(0, Path, 260), N, Err = 0;
  int Value = -1;
  HANDLE Dll, F;
  char Ascii[200];

  while (Len && Path[Len - 1] != '\\') Len--;
  Path[Len] = 0;
  /* umcidll.dll next to the EXE */
  { const WCHAR *D = L"umcidll.dll"; DWORD i = 0; while ((Path[Len + i] = D[i]) != 0) i++; }
  Dll = LoadLibraryW(Path);
  if (Dll) {
    PROBE P = (PROBE)GetProcAddress(Dll, "UmciProbe");
    Value = P ? P() : -2;
  } else {
    Err = GetLastError();
  }
  wsprintfW(Msg, L"EXE runs. umcidll.dll: %s (error %lu, probe 0x%x)", Dll ? L"loaded" : L"NOT loaded", Err, Value);
  for (N = 0; Msg[N] && N < 199; N++) Ascii[N] = (char)Msg[N];
  { const WCHAR *T = L"umcitest.txt"; DWORD i = 0; while ((Path[Len + i] = T[i]) != 0) i++; }
  F = CreateFileW(Path, 0x40000000, 0, 0, 2, 0x80, 0);
  if (F != (HANDLE)-1) { WriteFile(F, Ascii, N, &N, 0); CloseHandle(F); }
  MessageBoxW(0, Msg, L"umcitest", 0);
  ExitProcess(0);
}
