#pragma once

#include <cstddef>

#include "weld_measure_c_api.h"

namespace scan_tracking::weld_measure::sdk_seh {

bool isProcessIsolated();
unsigned lastNativeFaultCode();

unsigned create(const wm_onnx_config* config, wm_context** outContext, wm_status* outStatus);
unsigned createFromIni(const char* iniPath, wm_context** outContext, wm_status* outStatus);
unsigned measureFrame(
    wm_context* context,
    int frameIndex1Based,
    const float* xyz,
    size_t pointCount,
    const wm_options* options,
    wm_frame_result* outResult,
    char* message,
    size_t messageCapacity,
    wm_status* outStatus);
unsigned measureSection(
    wm_context* context,
    const float* xyz,
    size_t pointCount,
    const wm_options* options,
    wm_section_result* outResult,
    char* message,
    size_t messageCapacity,
    wm_status* outStatus);
unsigned destroy(wm_context* context);

}  // namespace scan_tracking::weld_measure::sdk_seh
