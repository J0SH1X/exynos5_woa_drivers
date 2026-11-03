/* Test-signed user-mode DLL for the RT 8.1 code-integrity probe. */
typedef int BOOL;
__declspec(dllexport) int __stdcall UmciProbe(void) { return 0x5420; }
BOOL __stdcall _DllMainCRTStartup(void *h, unsigned long r, void *p) { (void)h; (void)r; (void)p; return 1; }
