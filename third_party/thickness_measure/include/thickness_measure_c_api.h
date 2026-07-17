#ifndef THICKNESS_MEASURE_C_API_H_
#define THICKNESS_MEASURE_C_API_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#  if defined(THICKNESS_MEASURE_EXPORTS)
#    define THICKNESS_MEASURE_API __declspec(dllexport)
#  else
#    define THICKNESS_MEASURE_API __declspec(dllimport)
#  endif
#else
#  define THICKNESS_MEASURE_API
#endif

typedef enum tm_status {
    TM_OK = 0,
    TM_ERR_INVALID_ARG = 1,
    TM_ERR_NOT_INITIALIZED = 2,
    TM_ERR_CONFIG = 3,
    TM_ERR_CLOUD = 4,
    TM_ERR_ICP = 5,
    TM_ERR_MEASURE = 6,
    TM_ERR_INTERNAL = 7
} tm_status;

typedef enum tm_thickness_method {
    TM_METHOD_NEAREST_BETWEEN_SURFACES = 0,
    TM_METHOD_TANGENT_PLANE_PROJECTION = 1
} tm_thickness_method;

typedef struct tm_context tm_context;

typedef struct tm_point3d {
    double x;
    double y;
    double z;
} tm_point3d;

typedef struct tm_preprocess_config {
    int enable_outlier_removal; /* 0/1 */
    int mean_k;
    double stddev_mul_thresh;
    int enable_voxel_downsample; /* 0/1 */
    double leaf_size;
} tm_preprocess_config;

typedef struct tm_icp_config {
    int max_iterations;
    double max_correspondence_distance;
    double transformation_epsilon;
    double euclidean_fitness_epsilon;
} tm_icp_config;

typedef struct tm_config {
    tm_preprocess_config preprocess;
    tm_icp_config icp;
    tm_thickness_method thickness_method;
    tm_point3d axis_point;
    tm_point3d axis_direction;
    tm_point3d template_feature_points[2];
} tm_config;

typedef struct tm_cloud_view {
    const float* xyz; /* interleaved x,y,z,... */
    size_t point_count;
} tm_cloud_view;

typedef struct tm_pair_clouds {
    tm_cloud_view inner;
    tm_cloud_view outer;
} tm_pair_clouds;

typedef struct tm_pair_result {
    double inner_icp_fitness_score;
    double outer_icp_fitness_score;
    double thickness_mm;
    int thickness_method; /* tm_thickness_method */
    tm_point3d template_feature_points[2];
    tm_point3d nearest_scan_points[2]; /* [0]=outer, [1]=inner */
    tm_point3d projected_points[2];
    int valid;
} tm_pair_result;

typedef struct tm_average_result {
    double thickness_mm; /* mean of successful pairs */
    size_t pair_count;
    size_t success_count;
    int valid;
} tm_average_result;

/** Fill config with algorithm defaults (feature points / axis left zero — must be set). */
THICKNESS_MEASURE_API void tm_config_default(tm_config* config);

/**
 * Create context with in-memory template clouds.
 * Templates are copied internally; xyz is interleaved float x,y,z.
 */
THICKNESS_MEASURE_API tm_status tm_create(
    const tm_config* config,
    const float* inner_template_xyz,
    size_t inner_template_count,
    const float* outer_template_xyz,
    size_t outer_template_count,
    tm_context** out_ctx);

/**
 * Create context from thickness_config.json.
 * Loads inner/outer template PCD paths from the JSON (scan paths ignored).
 */
THICKNESS_MEASURE_API tm_status tm_create_from_json(
    const char* json_path,
    tm_context** out_ctx);

/**
 * Measure one inner/outer scan pair (memory point clouds).
 * Non-finite points are skipped.
 */
THICKNESS_MEASURE_API tm_status tm_measure_pair(
    tm_context* ctx,
    const float* inner_scan_xyz,
    size_t inner_scan_count,
    const float* outer_scan_xyz,
    size_t outer_scan_count,
    tm_pair_result* out_result,
    char* message,
    size_t message_capacity);

/**
 * Measure multiple pairs and return the average thickness of successful pairs.
 * On partial failure, still returns TM_OK if at least one pair succeeded;
 * details go into message.
 */
THICKNESS_MEASURE_API tm_status tm_measure_pairs_average(
    tm_context* ctx,
    const tm_pair_clouds* pairs,
    size_t pair_count,
    tm_average_result* out_result,
    char* message,
    size_t message_capacity);

THICKNESS_MEASURE_API void tm_destroy(tm_context* ctx);

THICKNESS_MEASURE_API const char* tm_status_string(tm_status status);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* THICKNESS_MEASURE_C_API_H_ */
