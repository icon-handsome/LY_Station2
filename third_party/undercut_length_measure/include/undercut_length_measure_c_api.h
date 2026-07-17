#ifndef UNDERCUT_LENGTH_MEASURE_C_API_H_
#define UNDERCUT_LENGTH_MEASURE_C_API_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#  if defined(UNDERCUT_LENGTH_MEASURE_EXPORTS)
#    define UNDERCUT_LENGTH_MEASURE_API __declspec(dllexport)
#  else
#    define UNDERCUT_LENGTH_MEASURE_API __declspec(dllimport)
#  endif
#else
#  define UNDERCUT_LENGTH_MEASURE_API
#endif

typedef enum ulm_status {
    ULM_OK = 0,
    ULM_ERR_INVALID_ARG = 1,
    ULM_ERR_NOT_INITIALIZED = 2,
    ULM_ERR_CONFIG = 3,
    ULM_ERR_SECTIONS = 4,
    ULM_ERR_MEASURE = 5,
    ULM_ERR_INTERNAL = 6
} ulm_status;

typedef struct ulm_context ulm_context;

typedef struct ulm_point3d {
    double x;
    double y;
    double z;
} ulm_point3d;

typedef struct ulm_options {
    double section_interval_mm;
    double depth_threshold_mm;
    int median_filter_window;
} ulm_options;

/** One weld section from upstream weld measure (toe + undercut depths). */
typedef struct ulm_section {
    double position; /* optional section index / arc position; unused by length calc */
    ulm_point3d left_toe;
    double left_depth_mm;
    ulm_point3d right_toe;
    double right_depth_mm;
} ulm_section;

typedef struct ulm_side_result {
    double max_depth_mm;
    double length_mm;
    int max_continuous_sections;
} ulm_side_result;

typedef struct ulm_result {
    ulm_side_result left;
    ulm_side_result right;
    int valid;
} ulm_result;

/** Fill options with algorithm defaults. */
UNDERCUT_LENGTH_MEASURE_API void ulm_options_default(ulm_options* options);

/**
 * Create context with measurement options.
 * Options are copied; call ulm_set_options to change later.
 */
UNDERCUT_LENGTH_MEASURE_API ulm_status ulm_create(
    const ulm_options* options,
    ulm_context** out_ctx);

/**
 * Create context from undercut_length.ini ([Undercut] keys only).
 * [Input] sections_path is ignored (use memory / file measure APIs).
 */
UNDERCUT_LENGTH_MEASURE_API ulm_status ulm_create_from_ini(
    const char* ini_path,
    ulm_context** out_ctx);

UNDERCUT_LENGTH_MEASURE_API ulm_status ulm_set_options(
    ulm_context* ctx,
    const ulm_options* options);

UNDERCUT_LENGTH_MEASURE_API ulm_status ulm_get_options(
    const ulm_context* ctx,
    ulm_options* out_options);

/**
 * Measure undercut lengths from an in-memory section sequence.
 * Left/right sides are evaluated independently.
 */
UNDERCUT_LENGTH_MEASURE_API ulm_status ulm_measure(
    ulm_context* ctx,
    const ulm_section* sections,
    size_t section_count,
    ulm_result* out_result,
    char* message,
    size_t message_capacity);

/**
 * Load sections from txt (8 or 9 numeric columns per line) then measure.
 * See project README for file format.
 */
UNDERCUT_LENGTH_MEASURE_API ulm_status ulm_measure_from_file(
    ulm_context* ctx,
    const char* sections_path,
    ulm_result* out_result,
    char* message,
    size_t message_capacity);

UNDERCUT_LENGTH_MEASURE_API void ulm_destroy(ulm_context* ctx);

UNDERCUT_LENGTH_MEASURE_API const char* ulm_status_string(ulm_status status);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* UNDERCUT_LENGTH_MEASURE_C_API_H_ */
