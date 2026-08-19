#ifndef CONTAINER_TOTAL_LENGTH_API_H_
#define CONTAINER_TOTAL_LENGTH_API_H_

#include <stddef.h>

#ifdef _WIN32
#  ifdef CONTAINER_TOTAL_LENGTH_BUILD_DLL
#    define CONTAINER_TOTAL_LENGTH_API __declspec(dllexport)
#  else
#    define CONTAINER_TOTAL_LENGTH_API __declspec(dllimport)
#  endif
#else
#  define CONTAINER_TOTAL_LENGTH_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ctl_context ctl_context;

typedef enum ctl_status {
    CTL_OK = 0,
    CTL_ERR_INVALID_ARGUMENT = 1,
    CTL_ERR_CONFIG = 2,
    CTL_ERR_TEMPLATE = 3,
    CTL_ERR_INPUT = 4,
    CTL_ERR_MEASURE = 5,
    CTL_ERR_NOT_INITIALIZED = 6
} ctl_status;

typedef struct ctl_result {
    double length_mm;
    double left_end_position;
    double right_end_position;
    double icp_fitness;
    float fitted_radius_mm;
    float cylinder_point_x;
    float cylinder_point_y;
    float cylinder_point_z;
    float cylinder_axis_x;
    float cylinder_axis_y;
    float cylinder_axis_z;
    int icp_converged;
    int input_point_count;
    int valid;
} ctl_result;

/* Create a context from an INI file and load its template cloud.
 * Relative templateCloud paths are resolved relative to the INI directory. */
CONTAINER_TOTAL_LENGTH_API ctl_status ctl_create_from_ini(
    const char* config_path,
    ctl_context** out_context,
    char* message,
    size_t message_size);

/* Measure an interleaved XYZ buffer: x0,y0,z0,x1,y1,z1,... */
CONTAINER_TOTAL_LENGTH_API ctl_status ctl_measure(
    ctl_context* context,
    const float* xyz,
    size_t point_count,
    ctl_result* result,
    char* message,
    size_t message_size);

CONTAINER_TOTAL_LENGTH_API void ctl_destroy(ctl_context* context);

CONTAINER_TOTAL_LENGTH_API const char* ctl_status_string(ctl_status status);

#ifdef __cplusplus
}
#endif

#endif
