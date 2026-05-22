#include "lbn_pose_core.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace lbn_pose {
namespace {

int clampInt(int value, int low, int high)
{
    return std::max(low, std::min(value, high));
}

bool isFinitePoint(const cv::Point3f& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

const cv::Point3f* cloudAt(const AlignedOrganizedCloud& cloud, int row, int col)
{
    if (cloud.points == nullptr || cloud.width <= 0 || cloud.height <= 0) {
        return nullptr;
    }
    if (row < 0 || col < 0 || row >= cloud.height || col >= cloud.width) {
        return nullptr;
    }
    return &cloud.points[row * cloud.width + col];
}

// 与 third_party/LBN/main.cpp ::queryPointInterpolated(TexturedPointCloud) 相同逻辑：
// cloud.at(row,col) ↔ points[row*width+col]；仅 isFinitePoint 过滤；半径内反距离加权回退。
bool queryPointInterpolated(
    const AlignedOrganizedCloud& cloud,
    double x,
    double y,
    cv::Point3f& out,
    int radius)
{
    if (cloud.points == nullptr || cloud.width <= 0 || cloud.height <= 0) {
        return false;
    }

    const int width = cloud.width;
    const int height = cloud.height;
    if (x < 0.0 || y < 0.0 || x > width - 1 || y > height - 1) {
        return false;
    }

    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, width - 1);
    const int y1 = std::min(y0 + 1, height - 1);
    const double dx = x - x0;
    const double dy = y - y0;

    const cv::Point3f p00 = *cloudAt(cloud, y0, x0);
    const cv::Point3f p10 = *cloudAt(cloud, y0, x1);
    const cv::Point3f p01 = *cloudAt(cloud, y1, x0);
    const cv::Point3f p11 = *cloudAt(cloud, y1, x1);
    if (isFinitePoint(p00) && isFinitePoint(p10) && isFinitePoint(p01) && isFinitePoint(p11)) {
        const double w00 = (1.0 - dx) * (1.0 - dy);
        const double w10 = dx * (1.0 - dy);
        const double w01 = (1.0 - dx) * dy;
        const double w11 = dx * dy;
        out.x = static_cast<float>(w00 * p00.x + w10 * p10.x + w01 * p01.x + w11 * p11.x);
        out.y = static_cast<float>(w00 * p00.y + w10 * p10.y + w01 * p01.y + w11 * p11.y);
        out.z = static_cast<float>(w00 * p00.z + w10 * p10.z + w01 * p01.z + w11 * p11.z);
        return true;
    }

    double sumW = 0.0;
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    for (int yy = clampInt(y0 - radius, 0, height - 1); yy <= clampInt(y0 + radius, 0, height - 1);
         ++yy) {
        for (int xx = clampInt(x0 - radius, 0, width - 1); xx <= clampInt(x0 + radius, 0, width - 1);
             ++xx) {
            const cv::Point3f p = *cloudAt(cloud, yy, xx);
            if (!isFinitePoint(p)) {
                continue;
            }
            const double dist2 = (xx - x) * (xx - x) + (yy - y) * (yy - y);
            const double w = 1.0 / std::max(dist2, 1e-6);
            sumW += w;
            sx += w * p.x;
            sy += w * p.y;
            sz += w * p.z;
        }
    }

    if (sumW <= 0.0) {
        return false;
    }

    out.x = static_cast<float>(sx / sumW);
    out.y = static_cast<float>(sy / sumW);
    out.z = static_cast<float>(sz / sumW);
    return true;
}

}  // namespace

bool grayImageToCvMat(const uint8_t* pixels, int width, int height, cv::Mat& grayImage)
{
    if (pixels == nullptr || width <= 0 || height <= 0) {
        return false;
    }

    grayImage.create(height, width, CV_8UC1);
    for (int row = 0; row < height; ++row) {
        uint8_t* dst = grayImage.ptr<uint8_t>(row);
        const uint8_t* src = pixels + static_cast<std::size_t>(row * width);
        std::memcpy(dst, src, static_cast<std::size_t>(width));
    }
    return true;
}

bool detectMarkerCenters2d(
    const cv::Mat& grayImage,
    std::vector<cv::Point2f>& centers2d,
    int markerMinArea,
    int markerMaxArea,
    int markerIntensityThreshold,
    float markerDebscanDistPx)
{
    centers2d.clear();
    if (grayImage.empty()) {
        return false;
    }
    if (grayImage.type() != CV_8UC1) {
        return false;
    }

    MarkPointDetector detector;
    // 检测参数来自 LbnPoseConfig / config.ini，非固定硬编码（原 intensity=50、debscan=300）
    detector.config.min_area = std::max(1, markerMinArea);
    detector.config.max_area = std::max(detector.config.min_area + 1, markerMaxArea);
    detector.config.intensityThreshold = std::max(1, markerIntensityThreshold);
    detector.config.debscanFilterDistPx = std::max(10.0f, markerDebscanDistPx);
    return detector.ProcessFrame(grayImage, centers2d);
}

// 与 main.cpp 中 for (mark_centers) { queryPointInterpolated(...); mark_centers_3D.push_back(...); } 一致
int liftCentersTo3d(
    const std::vector<cv::Point2f>& centers2d,
    const AlignedOrganizedCloud& cloud,
    int searchRadiusPx,
    std::vector<cv::Point3f>& centers3d)
{
    centers3d.clear();
    if (centers2d.empty() || cloud.points == nullptr || cloud.width <= 0 || cloud.height <= 0) {
        return 0;
    }

    const int radius = std::max(0, searchRadiusPx);
    centers3d.reserve(centers2d.size());
    for (const cv::Point2f& center2d : centers2d) {
        cv::Point3f center3d;
        if (queryPointInterpolated(cloud, center2d.x, center2d.y, center3d, radius)) {
            centers3d.push_back(cv::Point3f(center3d.x, center3d.y, center3d.z));
        }
    }
    return static_cast<int>(centers3d.size());
}

cv::Mat makeIdentityRt4x64()
{
    return cv::Mat::eye(4, 4, CV_64F);
}

bool rtGlobalToRowMajor16(const cv::Mat& rtGlobal, float out16[16])
{
    if (out16 == nullptr || rtGlobal.empty() || rtGlobal.rows != 4 || rtGlobal.cols != 4) {
        return false;
    }

    cv::Mat rt64;
    if (rtGlobal.type() == CV_64F) {
        rt64 = rtGlobal;
    } else {
        rtGlobal.convertTo(rt64, CV_64F);
    }

    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            out16[row * 4 + col] = static_cast<float>(rt64.at<double>(row, col));
        }
    }
    return true;
}

Estimator::Estimator(const Config& config)
    : m_config(config)
    , m_geoHash(config.maxDistance, config.minDistance)
{
    m_geoHash.set_template_config(config.minDistance, config.maxDistance);
    m_geoHash.set_query_config(config.cosTolerance, config.minPercent);
    applyScanToMarkerCalibration();

    if (!m_config.templateFilePath.empty()) {
        reloadTemplate(m_config.templateFilePath);
    }
}

int Estimator::reloadTemplate(const std::string& templateFilePath)
{
    m_templateReady = false;
    if (templateFilePath.empty()) {
        return static_cast<int>(ErrorCode::InvalidInput);
    }

    std::vector<char> pathBuffer(templateFilePath.begin(), templateFilePath.end());
    pathBuffer.push_back('\0');

    const int loadStatus = m_geoHash.read_template_pnts(pathBuffer.data());
    if (loadStatus != 0) {
        return loadStatus;
    }

    const int buildStatus = m_geoHash.build();
    if (buildStatus != 0) {
        return buildStatus;
    }

    m_config.templateFilePath = templateFilePath;
    m_templateReady = true;
    return 0;
}

void Estimator::applyScanToMarkerCalibration()
{
    cv::Mat scanRt64 = makeIdentityRt4x64();
    if (!m_config.scanToMarkerRt.empty()) {
        if (m_config.scanToMarkerRt.type() == CV_64F) {
            scanRt64 = m_config.scanToMarkerRt.clone();
        } else {
            m_config.scanToMarkerRt.convertTo(scanRt64, CV_64F);
        }
    }
    m_geoHash.set_scan_to_marker_RT(scanRt64);
}

Result Estimator::makeFailure(ErrorCode code, const std::string& message) const
{
    Result result;
    result.success = false;
    result.errorCode = code;
    result.message = message;
    return result;
}

Result Estimator::estimateFrom3d(const std::vector<cv::Point3f>& frame3dPoints)
{
    if (!m_templateReady) {
        return makeFailure(ErrorCode::TemplateLoadFailed, "LBN template is not loaded.");
    }
    if (frame3dPoints.size() < 3) {
        return makeFailure(ErrorCode::Insufficient3dPoints, "Need at least 3 marker 3D points.");
    }

    std::vector<cv::Point3f> frame3d = frame3dPoints;
    const int trackStatus = m_geoHash.Get_Track_Pose(
        frame3d,
        m_config.cosTolerance,
        m_config.minPercent);
    if (trackStatus != 0) {
        return makeFailure(
            ErrorCode::PoseTrackFailed,
            "FastGeoHash::Get_Track_Pose failed, code=" + std::to_string(trackStatus));
    }

    Result result;
    result.success = true;
    result.errorCode = ErrorCode::Ok;
    result.message = "LBN pose estimation succeeded.";
    result.rtGlobal = m_geoHash.Rt_global.clone();
    result.matchedPointCount = static_cast<int>(m_geoHash.filtered_frame_3d_points.size());
    return result;
}

Result Estimator::estimate(const cv::Mat& grayImage, const AlignedOrganizedCloud& alignedCloud)
{
    Result result;
    result.centers2d.clear();
    result.centers3d.clear();

    if (grayImage.empty()) {
        return makeFailure(ErrorCode::EmptyImage, "Gray image is empty.");
    }
    if (alignedCloud.points == nullptr || alignedCloud.width <= 0 || alignedCloud.height <= 0) {
        return makeFailure(ErrorCode::InvalidInput, "Aligned organized cloud is invalid.");
    }

    if (!detectMarkerCenters2d(
            grayImage,
            result.centers2d,
            m_config.markerMinArea,
            m_config.markerMaxArea,
            m_config.markerIntensityThreshold,
            m_config.markerDebscanDistPx)) {
        return makeFailure(ErrorCode::MarkerDetectionFailed, "MarkPointDetector::ProcessFrame failed.");
    }

    liftCentersTo3d(
        result.centers2d,
        alignedCloud,
        m_config.cloudSearchRadiusPx,
        result.centers3d);
    if (result.centers3d.size() < 3) {
        Result failure = makeFailure(
            ErrorCode::Insufficient3dPoints,
            "Less than 3 valid 3D marker points after 2D-to-3D lifting.");
        failure.centers2d = std::move(result.centers2d);
        failure.centers3d = std::move(result.centers3d);
        return failure;
    }

    Result poseResult = estimateFrom3d(result.centers3d);
    if (!poseResult.success) {
        poseResult.centers2d = std::move(result.centers2d);
        poseResult.centers3d = std::move(result.centers3d);
        return poseResult;
    }

    poseResult.centers2d = std::move(result.centers2d);
    poseResult.centers3d = std::move(result.centers3d);
    return poseResult;
}

}  // namespace lbn_pose
