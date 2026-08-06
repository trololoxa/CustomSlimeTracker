// Host sanitizer policy only. AddressSanitizer on GCC links LeakSanitizer by
// default; keeping leak detection out of ASan/UBSan makes that gate runnable
// under debuggers/ptrace and reserves leak ownership for the explicit LSan run.

#if defined(__has_feature)
#if __has_feature(address_sanitizer)
#define TRACKER_HOST_HAS_ASAN 1
#endif
#endif

#if defined(__SANITIZE_ADDRESS__)
#define TRACKER_HOST_HAS_ASAN 1
#endif

#if defined(TRACKER_HOST_HAS_ASAN)
extern "C" const char* __asan_default_options() {
    return "detect_leaks=0:halt_on_error=1";
}

extern "C" const char* __lsan_default_options() {
    return "detect_leaks=0";
}
#elif defined(TRACKER_ENABLE_LSAN)
extern "C" const char* __lsan_default_options() {
    return "exitcode=23";
}
#endif
