#include "scan_tracking/mech_eye/mech_eye_qt_seh.h"

#include <new>
#include <string>
#include <utility>

#include "scan_tracking/mech_eye/mech_eye_sdk_seh.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace scan_tracking::mech_eye {
namespace {

// Bodies must stay free of C++ objects that need unwinding (MSVC C2712 with __try).

struct ExtractQStringCtx {
    const QString* src = nullptr;
    std::wstring* out = nullptr;
    int ok = 0;
};

void extractQStringBody(void* raw)
{
    auto* ctx = static_cast<ExtractQStringCtx*>(raw);
    if (ctx == nullptr || ctx->src == nullptr || ctx->out == nullptr) {
        return;
    }
    const int n = ctx->src->size();
    if (n < 0) {
        return;
    }
    const ushort* utf16 = ctx->src->utf16();
    if (utf16 == nullptr && n > 0) {
        return;
    }
    ctx->out->assign(
        reinterpret_cast<const wchar_t*>(utf16),
        reinterpret_cast<const wchar_t*>(utf16) + n);
    ctx->ok = 1;
}

struct DestroyCameraInfoCtx {
    CameraInfoSnapshot* info = nullptr;
};

void destroyCameraInfoBody(void* raw)
{
    auto* ctx = static_cast<DestroyCameraInfoCtx*>(raw);
    if (ctx == nullptr || ctx->info == nullptr) {
        return;
    }
    ctx->info->~CameraInfoSnapshot();
}

}  // namespace

QString safeCopyQString(const QString& src)
{
    std::wstring buffer;
    ExtractQStringCtx ctx{&src, &buffer, 0};
    const unsigned seh = sdk_seh::invokeVoid(&extractQStringBody, &ctx);
    if (seh != 0 || ctx.ok == 0) {
        return QString();
    }
    if (buffer.empty()) {
        return QString();
    }
    return QString::fromWCharArray(buffer.data(), static_cast<int>(buffer.size()));
}

CameraInfoSnapshot safeCopyCameraInfo(const CameraInfoSnapshot& src)
{
    CameraInfoSnapshot out;
    out.model = safeCopyQString(src.model);
    out.serialNumber = safeCopyQString(src.serialNumber);
    out.ipAddress = safeCopyQString(src.ipAddress);
    out.firmwareVersion = safeCopyQString(src.firmwareVersion);
    out.connected = src.connected;
    return out;
}

void forceResetCameraInfo(CameraInfoSnapshot* info)
{
    if (info == nullptr) {
        return;
    }
    DestroyCameraInfoCtx ctx{info};
    (void)sdk_seh::invokeVoid(&destroyCameraInfoBody, &ctx);
    new (info) CameraInfoSnapshot{};
}

void replaceCameraInfo(CameraInfoSnapshot* dst, CameraInfoSnapshot src)
{
    if (dst == nullptr) {
        return;
    }
    forceResetCameraInfo(dst);
    *dst = std::move(src);
}

}  // namespace scan_tracking::mech_eye
