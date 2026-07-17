#ifndef LENGTH_VOLUME_MEASURE_C_API_H_
#define LENGTH_VOLUME_MEASURE_C_API_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#  if defined(LENGTH_VOLUME_MEASURE_EXPORTS)
#    define LENGTH_VOLUME_MEASURE_API __declspec(dllexport)
#  else
#    define LENGTH_VOLUME_MEASURE_API __declspec(dllimport)
#  endif
#else
#  define LENGTH_VOLUME_MEASURE_API
#endif

typedef enum lvm_status {
    LVM_OK = 0,
    LVM_ERR_INVALID_ARG = 1,
    LVM_ERR_NOT_INITIALIZED = 2,
    LVM_ERR_CONFIG = 3,
    LVM_ERR_CLOUD = 4,
    LVM_ERR_ICP = 5,
    LVM_ERR_MEASURE = 6,
    LVM_ERR_INTERNAL = 7
} lvm_status;

typedef struct lvm_context lvm_context;

typedef struct lvm_config {
    int crop_input_cloud; /* 0/1 */
    float crop_min_x;
    float crop_min_y;
    float crop_min_z;
    float crop_max_x;
    float crop_max_y;
    float crop_max_z;

    float axis_direction_x;
    float axis_direction_y;
    float axis_direction_z;
    float end_point1_x;
    float end_point1_y;
    float end_point1_z;
    float end_point2_x;
    float end_point2_y;
    float end_point2_z;

    float voxel_size;
    int outlier_k;
    double outlier_std;

    int icp_max_iterations;
    float icp_max_correspondence_distance;
    double icp_transformation_epsilon;
    double icp_euclidean_fitness_epsilon;

    int cylinder_fit_iterations;
    float cylinder_inlier_band;
    int update_cylinder_axis; /* 0/1 */

    int normal_k;
    double end_normal_min_abs_dot;

    /* "outsideScan" or "templateWindow" */
    char endpoint_detection_method[32];
    float axial_bin_width;
    float end_search_half_width;
    float outside_scan_peak_search_width;
    int min_points_per_bin;
    int min_consecutive_inside_bins;
    int outside_check_bins;
    float refine_half_width;
    double refine_edge_percentile;
    float endpoint_max_radius;
} lvm_config;

typedef struct lvm_result {
    double length_mm;
    double volume_liters;
    double volume_radius_mm; /* radius used for volume (from caller) */
    double fitted_outer_radius_mm;

    double cylinder_point_x;
    double cylinder_point_y;
    double cylinder_point_z;
    double cylinder_axis_x;
    double cylinder_axis_y;
    double cylinder_axis_z;

    double end1_template_pos;
    double end1_coarse_pos;
    double end1_refined_pos;
    int end1_used_end_normals; /* 0/1 */
    int end1_coarse_point_count;
    int end1_refined_point_count;

    double end2_template_pos;
    double end2_coarse_pos;
    double end2_refined_pos;
    int end2_used_end_normals; /* 0/1 */
    int end2_coarse_point_count;
    int end2_refined_point_count;

    double icp_fitness_score;
    int icp_converged; /* 0/1 */
    int valid; /* 0/1 */
} lvm_result;

/** Fill config with algorithm defaults (matches console Config defaults). */
LENGTH_VOLUME_MEASURE_API void lvm_config_default(lvm_config* config);

/**
 * Create context with in-memory template cloud.
 * Template is copied internally; xyz is interleaved float x,y,z.
 */
LENGTH_VOLUME_MEASURE_API lvm_status lvm_create(
    const lvm_config* config,
    const float* template_xyz,
    size_t template_count,
    lvm_context** out_ctx);

/**
 * Create context from config.ini.
 * Loads template PCD path from [Template]/templateCloud.
 * [Input] inputCloud is ignored (scan clouds come from memory APIs).
 */
LENGTH_VOLUME_MEASURE_API lvm_status lvm_create_from_ini(
    const char* ini_path,
    lvm_context** out_ctx);

/**
 * Measure length from outer-surface scan cloud; volume from volume_radius_mm.
 * volume_radius_mm is typically half of inner-surface diameter (mm).
 * Non-finite scan points are skipped.
 */
LENGTH_VOLUME_MEASURE_API lvm_status lvm_measure(
    lvm_context* ctx,
    const float* scan_xyz,
    size_t scan_count,
    double volume_radius_mm,
    lvm_result* out_result,
    char* message,
    size_t message_capacity);

LENGTH_VOLUME_MEASURE_API void lvm_destroy(lvm_context* ctx);

LENGTH_VOLUME_MEASURE_API const char* lvm_status_string(lvm_status status);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LENGTH_VOLUME_MEASURE_C_API_H_ */
