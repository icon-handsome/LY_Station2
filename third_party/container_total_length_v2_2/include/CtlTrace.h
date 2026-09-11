#ifndef CONTAINER_TOTAL_LENGTH_CTL_TRACE_H_
#define CONTAINER_TOTAL_LENGTH_CTL_TRACE_H_

// Step-by-step crash localization for ContainerTotalLength.
// Writes to %TEMP%\ctl_v22_trace.log (and stderr), fflush after every line.
// Override path with env CTL_TRACE_LOG. Disable with CTL_TRACE=0.

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#if defined(_MSC_VER) && (_MSC_VER < 1900)
#define CTL_SNPRINTF _snprintf
#define CTL_VSNPRINTF _vsnprintf
#else
#define CTL_SNPRINTF std::snprintf
#define CTL_VSNPRINTF std::vsnprintf
#endif

namespace ctl_trace {

inline bool Enabled()
{
    static int cached = -1;
    if (cached >= 0) return cached != 0;
    const char* env = std::getenv("CTL_TRACE");
    if (env != nullptr && (env[0] == '0') && env[1] == '\0') {
        cached = 0;
        return false;
    }
    cached = 1;
    return true;
}

inline FILE* File()
{
    static FILE* file = nullptr;
    static bool initialized = false;
    if (initialized) return file;
    initialized = true;
    if (!Enabled()) return nullptr;

    char path[1024] = {0};
    const char* overridePath = std::getenv("CTL_TRACE_LOG");
    if (overridePath != nullptr && overridePath[0] != '\0') {
        std::strncpy(path, overridePath, sizeof(path) - 1);
    } else {
#ifdef _WIN32
        char tempDir[MAX_PATH] = {0};
        const DWORD n = ::GetTempPathA(MAX_PATH, tempDir);
        if (n > 0 && n < MAX_PATH) {
            CTL_SNPRINTF(path, sizeof(path), "%sctl_v22_trace.log", tempDir);
        } else {
            std::strncpy(path, "ctl_v22_trace.log", sizeof(path) - 1);
        }
#else
        std::strncpy(path, "ctl_v22_trace.log", sizeof(path) - 1);
#endif
    }

    file = std::fopen(path, "a");
    if (file != nullptr) {
        std::fprintf(file, "\n===== ctl_v22_trace open path=%s =====\n", path);
        std::fflush(file);
        std::fprintf(stderr, "[CTL] trace file: %s\n", path);
        std::fflush(stderr);
    }
    return file;
}

inline void Write(const char* fmt, ...)
{
    if (!Enabled()) return;

    char timeBuf[32] = {0};
    const std::time_t now = std::time(nullptr);
    std::tm localTm;
#ifdef _WIN32
    localtime_s(&localTm, &now);
#else
    localTm = *std::localtime(&now);
#endif
    std::strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", &localTm);

    char body[2048] = {0};
    va_list args;
    va_start(args, fmt);
    CTL_VSNPRINTF(body, sizeof(body), fmt, args);
    va_end(args);

    FILE* file = File();
    if (file != nullptr) {
        std::fprintf(file, "[%s] %s\n", timeBuf, body);
        std::fflush(file);
    }
    std::fprintf(stderr, "[CTL %s] %s\n", timeBuf, body);
    std::fflush(stderr);
}

inline void Mem(const char* tag)
{
#ifdef _WIN32
    MEMORYSTATUSEX status;
    status.dwLength = sizeof(status);
    if (::GlobalMemoryStatusEx(&status)) {
        Write("%s mem: load=%lu%% availPhys=%.1fMB totalPhys=%.1fMB availVirt=%.1fMB",
              tag,
              static_cast<unsigned long>(status.dwMemoryLoad),
              status.ullAvailPhys / (1024.0 * 1024.0),
              status.ullTotalPhys / (1024.0 * 1024.0),
              status.ullAvailVirtual / (1024.0 * 1024.0));
    } else {
        Write("%s mem: GlobalMemoryStatusEx failed", tag);
    }
#else
    Write("%s mem: n/a", tag);
#endif
}

inline double MsSince(unsigned long long startTick)
{
#ifdef _WIN32
    return static_cast<double>(::GetTickCount64() - startTick);
#else
    (void)startTick;
    return 0.0;
#endif
}

inline unsigned long long NowTick()
{
#ifdef _WIN32
    return ::GetTickCount64();
#else
    return 0;
#endif
}

}  // namespace ctl_trace

#define CTL_TRACE(...) ::ctl_trace::Write(__VA_ARGS__)
#define CTL_TRACE_MEM(tag) ::ctl_trace::Mem(tag)

#endif
