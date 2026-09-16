#ifndef WELD_MEASURE_C_API_H_
#define WELD_MEASURE_C_API_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#  if defined(WELD_MEASURE_EXPORTS)
#    define WELD_MEASURE_API __declspec(dllexport)
#  else
#    define WELD_MEASURE_API __declspec(dllimport)
#  endif
#else
#  define WELD_MEASURE_API
#endif

typedef enum wm_status {
    WM_OK = 0,
    WM_ERR_INVALID_ARG = 1,
    WM_ERR_NOT_INITIALIZED = 2,
    WM_ERR_MODEL = 3,
    WM_ERR_TOE = 4,
    WM_ERR_METRICS = 5,
    WM_ERR_UNDERCUT = 6,
    WM_ERR_INTERNAL = 7,
    WM_ERR_REGISTRATION = 8,
    WM_ERR_SECTION = 9
} wm_status;

typedef struct wm_context wm_context;

typedef struct wm_onnx_config {
    const char* model_path;
    const char* input_raw_name;
    const char* input_smooth_name;
    const char* output_name;
    int target_point_count;
    int median_filter_window;
    int seam_class_id;
    int num_classes;
    int min_seam_points;
} wm_onnx_config;

typedef struct wm_options {
    double mismatch_fit_length_mm;
    double angularity_half_span_mm;
    double min_fit_points;
    double undercut_section_interval_mm;
    double undercut_depth_threshold_mm;
    int undercut_median_filter_window;
    double undercut_toe_search_width_mm;
    double undercut_base_fit_start_from_toe_mm;
    double undercut_base_fit_end_from_toe_mm;
    double undercut_min_groove_width_mm;
    int undercut_min_groove_points;
} wm_options;

typedef struct wm_section_result {
    double mismatch_mm;
    double reinforcement_mm;
    double angularity_mm;
    double included_angle_rad;
    double toe_center_x;
    double left_toe_x;
    double left_toe_y;
    double left_toe_z;
    double right_toe_x;
    double right_toe_y;
    double right_toe_z;
    double left_undercut_mm;
    double right_undercut_mm;
    double max_undercut_mm;
    int undercut_raw_flag;
    int valid;
} wm_section_result;

/**
 * One-frame formal-pipeline result.
 * V2.0 kernel reports per-frame maxima after median filter; the C API maps those
 * maxima into average.* fields so existing IPC callers keep a stable layout.
 */
typedef struct wm_frame_result {
    wm_section_result average;
    int valid_sections;
    int total_sections;
    double left_undercut_length_mm;
    double right_undercut_length_mm;
    double left_max_undercut_depth_mm;
    double right_max_undercut_depth_mm;
} wm_frame_result;

WELD_MEASURE_API void wm_onnx_config_default(wm_onnx_config* config);
WELD_MEASURE_API void wm_options_default(wm_options* options);

WELD_MEASURE_API wm_status wm_create(const wm_onnx_config* onnx, wm_context** out_ctx);
WELD_MEASURE_API wm_status wm_create_from_ini(const char* ini_path, wm_context** out_ctx);

WELD_MEASURE_API wm_status wm_measure_section(
    wm_context* ctx,
    const float* xyz,
    size_t point_count,
    const wm_options* options,
    wm_section_result* out_result,
    char* message,
    size_t message_capacity);

WELD_MEASURE_API wm_status wm_measure_frame(
    wm_context* ctx,
    int frame_index_1based,
    const float* scan_xyz,
    size_t point_count,
    const wm_options* options,
    wm_frame_result* out_result,
    char* message,
    size_t message_capacity);

WELD_MEASURE_API void wm_destroy(wm_context* ctx);
WELD_MEASURE_API const char* wm_status_string(wm_status status);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* WELD_MEASURE_C_API_H_ */
