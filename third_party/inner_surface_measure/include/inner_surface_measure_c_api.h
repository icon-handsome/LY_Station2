#ifndef INNER_SURFACE_MEASURE_C_API_H_
#define INNER_SURFACE_MEASURE_C_API_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#  if defined(INNER_SURFACE_MEASURE_EXPORTS)
#    define INNER_SURFACE_MEASURE_API __declspec(dllexport)
#  else
#    define INNER_SURFACE_MEASURE_API __declspec(dllimport)
#  endif
#else
#  define INNER_SURFACE_MEASURE_API
#endif

typedef enum ism_status {
    ISM_OK = 0,
    ISM_ERR_INVALID_ARG = 1,
    ISM_ERR_NOT_INITIALIZED = 2,
    ISM_ERR_CONFIG = 3,
    ISM_ERR_CLOUD = 4,
    ISM_ERR_ICP = 5,
    ISM_ERR_MEASURE = 6,
    ISM_ERR_INTERNAL = 7
} ism_status;

typedef struct ism_context ism_context;

typedef struct ism_config {
    float voxel_size;
    int outlier_k; /* <=1 skips SOR (matches path4 console default behavior) */
    double outlier_std;

    int fit_iterations;
    float cylinder_inlier_band;
    float section_half_width;

    int icp_max_iterations;
    float icp_max_correspondence_distance;
    double icp_transformation_epsilon;
    double icp_euclidean_fitness_epsilon;

    double cylinder_point_x;
    double cylinder_point_y;
    double cylinder_point_z;
    double cylinder_axis_x;
    double cylinder_axis_y;
    double cylinder_axis_z;
    float cylinder_radius;

    /* Effective inner length (mm) for volume = pi*(d/2)^2*L / 1e6 */
    double container_length_mm;
} ism_config;

typedef struct ism_frame_result {
    double diameter_mm;
    double circumference_mm;
    double section_roundness[3];
    double average_roundness;
    double icp_fitness_score;
    int icp_converged; /* 0/1 */
    int used_point_count;
    int valid; /* 0/1 */
} ism_frame_result;

typedef struct ism_average_result {
    double diameter_mm;
    double circumference_mm;
    double roundness; /* mean of two frame average roundness values */
    int valid; /* 0/1 */
    double volume_liters; /* pi*(d/2)^2*container_length / 1e6; 0 if length unset */
    double container_length_mm;
} ism_average_result;

INNER_SURFACE_MEASURE_API void ism_config_default(ism_config* config);

INNER_SURFACE_MEASURE_API ism_status ism_create(
    const ism_config* config,
    const float* template_xyz,
    size_t template_count,
    ism_context** out_ctx);

INNER_SURFACE_MEASURE_API ism_status ism_create_from_ini(
    const char* ini_path,
    ism_context** out_ctx);

INNER_SURFACE_MEASURE_API ism_status ism_measure_frame(
    ism_context* ctx,
    const float* scan_xyz,
    size_t scan_count,
    ism_frame_result* out_result,
    char* message,
    size_t message_capacity);

INNER_SURFACE_MEASURE_API ism_status ism_measure_two_frames_average(
    ism_context* ctx,
    const float* frame1_xyz,
    size_t frame1_count,
    const float* frame2_xyz,
    size_t frame2_count,
    ism_average_result* out_average,
    ism_frame_result* out_frame1,
    ism_frame_result* out_frame2,
    char* message,
    size_t message_capacity);

/* Same as ism_measure_two_frames_average, but uses measured_length_mm when > 0. */
INNER_SURFACE_MEASURE_API ism_status ism_measure_two_frames_average_with_length(
    ism_context* ctx,
    const float* frame1_xyz,
    size_t frame1_count,
    const float* frame2_xyz,
    size_t frame2_count,
    double measured_length_mm,
    ism_average_result* out_average,
    ism_frame_result* out_frame1,
    ism_frame_result* out_frame2,
    char* message,
    size_t message_capacity);

INNER_SURFACE_MEASURE_API void ism_destroy(ism_context* ctx);

INNER_SURFACE_MEASURE_API const char* ism_status_string(ism_status status);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* INNER_SURFACE_MEASURE_C_API_H_ */
