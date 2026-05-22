#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "FastGeoHash.h"

namespace lbn_pose {

enum class ErrorCode {
    Ok = 0,
    InvalidInput = 100,
    EmptyImage = 101,
    MarkerDetectionFailed = 102,
    Insufficient3dPoints = 103,
    TemplateLoadFailed = 1,
    HashBuildFailed = 401,
    PoseTrackFailed = 500,
};

struct Config {
    // GeoHash 三角形边长范围 [minDistance, maxDistance]（mm）
    float maxDistance = 650.0f;
    float minDistance = 30.0f;
    // 匹配投票：角度余弦容差、有效票占比下限（须被 FastGeoHash::getResult 使用）
    float cosTolerance = 0.015f;
    float minPercent = 0.5f;
    int cloudSearchRadiusPx = 20;
    // 2D 标记点检测（MarkPointDetector）；生产值见 config.ini，勿盲目用离线 runner 极值
    int markerMinArea = 400;
    int markerMaxArea = 30000;
    int markerIntensityThreshold = 50;
    float markerDebscanDistPx = DEBSCAN_FILTER_DIST_MAX;
    std::string templateFilePath;
    cv::Mat scanToMarkerRt;
};

struct Result {
    bool success = false;
    ErrorCode errorCode = ErrorCode::Ok;
    std::string message;
    cv::Mat rtGlobal;
    std::vector<cv::Point2f> centers2d;
    std::vector<cv::Point3f> centers3d;
    int matchedPointCount = 0;
};

/**
 * 与 Mech-Eye 纹理点云对齐的组织化点云视图（行优先，与 2D 图像同分辨率）。
 * 索引与 third_party/LBN/main.cpp 中 TexturedPointCloud::at(row, col) 一致。
 */
struct AlignedOrganizedCloud {
    int width = 0;
    int height = 0;
    const cv::Point3f* points = nullptr;
};

bool detectMarkerCenters2d(
    const cv::Mat& grayImage,
    std::vector<cv::Point2f>& centers2d,
    int markerMinArea = 400,
    int markerMaxArea = 30000,
    int markerIntensityThreshold = 50,
    float markerDebscanDistPx = DEBSCAN_FILTER_DIST_MAX);

/** 2D 圆心 → 3D，逻辑同 main.cpp mark_centers_3D 填充（仅插值成功则 push）。 */
int liftCentersTo3d(
    const std::vector<cv::Point2f>& centers2d,
    const AlignedOrganizedCloud& cloud,
    int searchRadiusPx,
    std::vector<cv::Point3f>& centers3d);

/** 将 8-bit 灰度图转为 CV_8UC1，等同 main.cpp frame2dToCvMat 输出。 */
bool grayImageToCvMat(const uint8_t* pixels, int width, int height, cv::Mat& grayImage);

cv::Mat makeIdentityRt4x64();

/** 将 4x4 CV_64F 矩阵转为行优先 16 元数组（供 IPC PoseMatrix4x4 使用）。 */
bool rtGlobalToRowMajor16(const cv::Mat& rtGlobal, float out16[16]);

class Estimator {
public:
    explicit Estimator(const Config& config = Config());

    const Config& config() const { return m_config; }

    int reloadTemplate(const std::string& templateFilePath);

    Result estimateFrom3d(const std::vector<cv::Point3f>& frame3dPoints);

    Result estimate(const cv::Mat& grayImage, const AlignedOrganizedCloud& alignedCloud);

private:
    Result makeFailure(ErrorCode code, const std::string& message) const;
    void applyScanToMarkerCalibration();

    Config m_config;
    FastGeoHash m_geoHash;
    bool m_templateReady = false;
};

}  // namespace lbn_pose
