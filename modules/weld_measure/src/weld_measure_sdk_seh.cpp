#include "weld_measure_sdk_seh.h"

#include <atomic>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace scan_tracking::weld_measure::sdk_seh {
namespace {

std::atomic<unsigned>& nativeFaultCode()
{
    static std::atomic<unsigned> code{0};
    return code;
}

unsigned recordFault(unsigned code)
{
    nativeFaultCode().store(code, std::memory_order_release);
    return code;
}

#ifdef _MSC_VER
int filterNativeFault(unsigned code)
{
    // Let normal MSVC C++ exceptions unwind into the caller's try/catch.
    if (code == 0xE06D7363u) {
        return EXCEPTION_CONTINUE_SEARCH;
    }
    return EXCEPTION_EXECUTE_HANDLER;
}
#endif

void createBody(
    const wm_onnx_config* config,
    wm_context** outContext,
    wm_status* outStatus)
{
    *outStatus = wm_create(config, outContext);
}

void createFromIniBody(const char* iniPath, wm_context** outContext, wm_status* outStatus)
{
    *outStatus = wm_create_from_ini(iniPath, outContext);
}

void measureFrameBody(
    wm_context* context,
    int frameIndex1Based,
    const float* xyz,
    size_t pointCount,
    const wm_options* options,
    wm_frame_result* outResult,
    char* message,
    size_t messageCapacity,
    wm_status* outStatus)
{
    *outStatus = wm_measure_frame(
        context,
        frameIndex1Based,
        xyz,
        pointCount,
        options,
        outResult,
        message,
        messageCapacity);
}

void measureSectionBody(
    wm_context* context,
    const float* xyz,
    size_t pointCount,
    const wm_options* options,
    wm_section_result* outResult,
    char* message,
    size_t messageCapacity,
    wm_status* outStatus)
{
    *outStatus = wm_measure_section(
        context,
        xyz,
        pointCount,
        options,
        outResult,
        message,
        messageCapacity);
}

void destroyBody(wm_context* context)
{
    wm_destroy(context);
}

}  // namespace

bool isProcessIsolated()
{
    return nativeFaultCode().load(std::memory_order_acquire) != 0;
}

unsigned lastNativeFaultCode()
{
    return nativeFaultCode().load(std::memory_order_acquire);
}

unsigned create(const wm_onnx_config* config, wm_context** outContext, wm_status* outStatus)
{
    if (outContext == nullptr || outStatus == nullptr) {
        return 0xFFFFFFFFu;
    }
    *outContext = nullptr;
    *outStatus = WM_ERR_INTERNAL;
    if (isProcessIsolated()) {
        return lastNativeFaultCode();
    }
#ifdef _MSC_VER
    __try {
        createBody(config, outContext, outStatus);
        return 0;
    } __except (filterNativeFault(static_cast<unsigned>(GetExceptionCode()))) {
        *outContext = nullptr;
        return recordFault(static_cast<unsigned>(GetExceptionCode()));
    }
#else
    createBody(config, outContext, outStatus);
    return 0;
#endif
}

unsigned createFromIni(const char* iniPath, wm_context** outContext, wm_status* outStatus)
{
    if (outContext == nullptr || outStatus == nullptr) {
        return 0xFFFFFFFFu;
    }
    *outContext = nullptr;
    *outStatus = WM_ERR_INTERNAL;
    if (isProcessIsolated()) {
        return lastNativeFaultCode();
    }
#ifdef _MSC_VER
    __try {
        createFromIniBody(iniPath, outContext, outStatus);
        return 0;
    } __except (filterNativeFault(static_cast<unsigned>(GetExceptionCode()))) {
        *outContext = nullptr;
        return recordFault(static_cast<unsigned>(GetExceptionCode()));
    }
#else
    createFromIniBody(iniPath, outContext, outStatus);
    return 0;
#endif
}

unsigned measureFrame(
    wm_context* context,
    int frameIndex1Based,
    const float* xyz,
    size_t pointCount,
    const wm_options* options,
    wm_frame_result* outResult,
    char* message,
    size_t messageCapacity,
    wm_status* outStatus)
{
    if (outStatus == nullptr) {
        return 0xFFFFFFFFu;
    }
    *outStatus = WM_ERR_INTERNAL;
    if (isProcessIsolated()) {
        return lastNativeFaultCode();
    }
#ifdef _MSC_VER
    __try {
        measureFrameBody(
            context, frameIndex1Based, xyz, pointCount, options,
            outResult, message, messageCapacity, outStatus);
        return 0;
    } __except (filterNativeFault(static_cast<unsigned>(GetExceptionCode()))) {
        return recordFault(static_cast<unsigned>(GetExceptionCode()));
    }
#else
    measureFrameBody(
        context, frameIndex1Based, xyz, pointCount, options,
        outResult, message, messageCapacity, outStatus);
    return 0;
#endif
}

unsigned measureSection(
    wm_context* context,
    const float* xyz,
    size_t pointCount,
    const wm_options* options,
    wm_section_result* outResult,
    char* message,
    size_t messageCapacity,
    wm_status* outStatus)
{
    if (outStatus == nullptr) {
        return 0xFFFFFFFFu;
    }
    *outStatus = WM_ERR_INTERNAL;
    if (isProcessIsolated()) {
        return lastNativeFaultCode();
    }
#ifdef _MSC_VER
    __try {
        measureSectionBody(
            context, xyz, pointCount, options,
            outResult, message, messageCapacity, outStatus);
        return 0;
    } __except (filterNativeFault(static_cast<unsigned>(GetExceptionCode()))) {
        return recordFault(static_cast<unsigned>(GetExceptionCode()));
    }
#else
    measureSectionBody(
        context, xyz, pointCount, options,
        outResult, message, messageCapacity, outStatus);
    return 0;
#endif
}

unsigned destroy(wm_context* context)
{
    if (context == nullptr || isProcessIsolated()) {
        return isProcessIsolated() ? lastNativeFaultCode() : 0;
    }
#ifdef _MSC_VER
    __try {
        destroyBody(context);
        return 0;
    } __except (filterNativeFault(static_cast<unsigned>(GetExceptionCode()))) {
        return recordFault(static_cast<unsigned>(GetExceptionCode()));
    }
#else
    destroyBody(context);
    return 0;
#endif
}

}  // namespace scan_tracking::weld_measure::sdk_seh
